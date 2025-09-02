import time
import math as m
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose
from movement_platform_if.srv import Command
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import euler2quat, quat2euler
import toml, json
from platform_audio import PlatformAudio

unload_y_offset = 0


class GoalPoint:
    def __init__(self, position: list[float, float], angle: float):
        '''
        position: [x,y] in meters\n
        angle: degree
        '''
        self.position = position
        self.angle = angle


class PlatformNode(Node):
    def __init__(self):
        super().__init__('PlatformNode')
        self.cmd_srv = self.create_service(Command, '/platform/command', self.cmd_callback)
        self.create_subscription(Pose, '/text_coordinate', self.text_coord_callback, 2)
        self.move_pub = self.create_publisher(Twist, '/cmd_vel', 2)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.audio = PlatformAudio(self)
        self.get_logger().info("node init")
        ## Variable
        self.waiting_unload_pose = False
    
    def text_coord_callback(self, msg: Pose) -> None:
        global unload_y_offset
        if self.waiting_unload_pose:
            self.get_logger().info(f"[Text Coord] Received: {msg.position}")
            unload_y_offset = msg.position.x + 0.02
            self.get_logger().info(f"unload offset: {unload_y_offset}")
            if unload_y_offset >= 0:
                goal_target("current", "y_positive")
                goal_target("current", "unload")
            else:
                goal_target("current", "y_negitive")
                goal_target("current", "unload")
            self.waiting_unload_pose = False

    def cmd_callback(self, request: Command.Request, response: Command.Response):
        self.get_logger().info(request.cmd)
        cmd: dict = json.loads(request.cmd)
        ## (src,dst) cmd
        if cmd.get('src') is not None:
            self.get_logger().info("(src,dst) cmd")
            goal_target(cmd['src'], cmd['dst'])
            self.audio.beep_step()
            response.result = 1
        ## (offset) cmd
        elif cmd.get('offset') is not None:
            ## request offset
            if cmd['offset']:
                self.get_logger().info("offset")
                self.waiting_unload_pose = True
                response.result = 2
            ## get offset status (finish)
            elif not self.waiting_unload_pose:
                response.result = 2
            ## get offset status (busy)
            else:
                response.result = -2
                
        ## unknown
        else:
            self.get_logger().info("unknown cmd")
            response.result = -1
        # time.sleep(3)
        return response

    def get_current_pose(self) -> GoalPoint | None:
        pose_tf = None
        try:
            pose_tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()).transform
            # self.get_logger().info(pose_tf)
        except:
            return None
        quaternion = [pose_tf.rotation.w, pose_tf.rotation.x, pose_tf.rotation.y, pose_tf.rotation.z]
        roll, pitch, yaw = quat2euler(quaternion, axes='sxyz')
        return GoalPoint([pose_tf.translation.x, pose_tf.translation.y], yaw)

    def set_rotate(self, angular_speed: float) -> None:
        msg = Twist()
        msg.angular.z = float(angular_speed)
        self.move_pub.publish(msg)


target_points_cfg = toml.load('./config/target_points.toml')
navigator: BasicNavigator = None
node: PlatformNode = None


def goal_special_word(point):
    if point[0] == "FAST_ROTATE":
        target_angle = point[1]
        while True:
            rclpy.spin_once(node)
            current_pose = node.get_current_pose()
            if current_pose is not None:
                current_angle = m.degrees(current_pose.angle)
                diff_angle = current_angle - target_angle
                # print(f"curr: {current_angle}, target_angle: {target_angle}, diff: {diff_angle}")
                ## goal finish
                if abs(diff_angle) < 0.5:
                    node.set_rotate(0)
                    # node.get_logger().info("FAST ROTATE Success")
                    break
                ## rotate
                else:
                    if abs(diff_angle) < 10:
                        speed = 0.3
                    elif abs(diff_angle) < 20:
                        speed = 0.6
                    else:
                        speed = 1
                    if diff_angle >= 0:
                        node.set_rotate(-speed)
                    else:
                        node.set_rotate(speed)
    else:
        node.get_logger().warn("Unknown special word")


def goal(points: list[GoalPoint | list]):
    print(points)
    waypoints = []
    tmp_points = []
    for point in points:
        ## is special word
        if isinstance(point, list):
            ## run buffered normal points first
            goal(tmp_points)
            tmp_points = []
            waypoints = []
            goal_special_word(point)
            continue
        node.get_logger().info(f"target: (x,y): {point.position}, angle: {point.angle}")
        # Set the robot's goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(point.position[0])
        goal_pose.pose.position.y = float(point.position[1])
        goal_pose.pose.position.z = 0.0
        quat = euler2quat(0, 0, m.radians(point.angle))
        goal_pose.pose.orientation.x = quat[1]
        goal_pose.pose.orientation.y = quat[2]
        goal_pose.pose.orientation.z = quat[3]
        goal_pose.pose.orientation.w = quat[0]
        # node.get_logger().info(f"raw: {goal_pose}")
        tmp_points.append(point)
        waypoints.append(goal_pose)
    # Go to the goal pose
    # navigator.goToPose(goal_pose)
    navigator.followWaypoints(waypoints)
    # Keep doing stuff as long as the robot is moving towards the goal
    while not navigator.isTaskComplete():
        time.sleep(0.1)
        pass
    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        node.get_logger().info('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        node.get_logger().warn('Goal was canceled!')
    elif result == TaskResult.FAILED:
        node.get_logger().error('Goal failed!')
    else:
        node.get_logger().error('Goal has an invalid return status!')


def parse_points_cfg(points: list[list], named_point_cfg: dict) -> list[GoalPoint | list]:
    '''
    input points list with variable and special words\n
    input named point config of points list\n
    return points list after variable replacement (special words won't change)
    '''
    goal_points = []
    ## run for every point
    for point in points:
        params = []
        special_word_flag = False
        # run for param in point
        for param in point:
            param = param[0]
            ## is variable or special words
            if isinstance(param, str):
                ## try to get variable from named point config
                if named_point_cfg.get(param):
                    params.append(named_point_cfg[param])
                ## try to get variable from top level config
                elif target_points_cfg.get(param):
                    params.append(target_points_cfg[param])
                ## is special word
                else:
                    params.append(param)
                    special_word_flag = True
            ## number only
            else:
                params.append(param)
        ## convert to goal point when not a special word
        if not special_word_flag:
            goal_points.append(GoalPoint([params[0], params[1]], params[2]))
        else:
            goal_points.append(params)
    return goal_points


def goal_target(src: str, dst: str):
    '''
    goal from src named point to dst named point
    '''
    global unload_y_offset
    ## clear unload offset when init
    if src == "init" and dst == "init":
        unload_y_offset = 0
    src_cfg: dict = target_points_cfg[src]
    dst_cfg: dict = target_points_cfg[dst]
    src_points_cfg: dict = None
    dst_points_cfg: dict = None
    ## src
    ## get exit_{dst} from src
    src_key = f"exit_{dst}"
    if src_cfg.get(src_key) is not None:
        src_points_cfg = src_cfg[src_key]
        node.get_logger().info(f'src key: {src_key}')
    ## get exit_default from src
    else:
        src_points_cfg = src_cfg['exit_default']
        node.get_logger().info(f'src key: default')
    ## dst
    ## get enter_{src} from dst
    dst_key = f"enter_{src}"
    if dst_cfg.get(dst_key) is not None:
        dst_points_cfg = dst_cfg[dst_key]
        node.get_logger().info(f'dst key: {dst_key}')
    ## get enter_default from dst
    else:
        dst_points_cfg = dst_cfg['enter_default']
        node.get_logger().info(f'dst key: default')
    ## get points list from config
    src_points = parse_points_cfg(src_points_cfg, src_cfg)
    dst_points = parse_points_cfg(dst_points_cfg, dst_cfg)
    ## unload offset
    if dst == "unload":
        dst_points[-1].position[1] += unload_y_offset
    ## combine points list and goal
    goal(src_points + dst_points)


def main():
    global navigator, node
    rclpy.init() 
    node = PlatformNode()
    node.create_rate(10)
    navigator = BasicNavigator()

    # Set the robot's initial pose if necessary
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.position.z = 0.0
    # initial_pose.pose.orientation.x = 0.0
    # initial_pose.pose.orientation.y = 0.0
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0
    # navigator.setInitialPose(initial_pose)

    ## Wait for navigation to fully activate. Use this line if autostart is set to true.
    # node.get_logger().info("wait active")
    # navigator.waitUntilNav2Active()
    time.sleep(1)
    node.get_logger().info("active")

    # goal_target('init', 'init')

    while rclpy.ok():
        rclpy.spin_once(node)
        time.sleep(0.1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()