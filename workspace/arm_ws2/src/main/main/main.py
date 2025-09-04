import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from movement_platform_if.srv import GoalRequest, GoalStatus
import time, yaml, json
from python_moveit_interface.srv import PoseRequest
from std_srvs.srv import Trigger
from enum import Enum
# from platform_audio import PlatformAudio
# from platform_button import PlatformButton
import math as m
from geometry_msgs.msg import PoseStamped, Pose
from tf_transformations import quaternion_from_euler

class ArmStateCmd(Enum):
    INITIAL_POSE        = "home"
    DETECT_POSE         = "detect"
    GRAB_OPEN           = "grab_open"
    GRAB_CLOSE          = "grab_close"

class PlatformCmd(Enum):
    HOME    = "home"
    LEFT    = "left"
    TOP     = "top"
    RIGHT   = "right"
    UNLOAD  = "unload"

class MainNode(Node):
    def __init__(self):
        super().__init__('Main')
        self.platform_goal_request_cli = self.create_client(GoalRequest, 'goal_request')
        self.platform_goal_status_cli = self.create_client(GoalStatus, 'goal_status')
        self.arm_pose_request_cli = self.create_client(PoseRequest, 'arm_goal_pose')
        self.arm_pose_finish_cli = self.create_client(Trigger, 'arm_goal_finish')
        self.create_subscription(String, "/cube_info", self.cube_info_callback, 2)
        self.create_subscription(Int32, "/speech_recognition", self.speech_recognition_callback, 2)
        # self.audio = PlatformAudio(self)
        # self.button = PlatformButton(self, self.audio)
        self.get_logger().info("node init")
        ## Variable
        self.cube_info: List[dict] = []
        self.speech_recognition: int = None
        ## Config
        with open('platform.yaml', 'r') as file:
            self.platform_config = yaml.safe_load(file)

    def platform_goal(self, dst: PlatformCmd) -> bool:
        pose_raw = self.platform_config[dst.value]
        quat = quaternion_from_euler(0.0, 0.0, m.radians(pose_raw[2]))
        req = GoalRequest.Request()
        req.goal_pose.position.x = pose_raw[0]
        req.goal_pose.position.y = pose_raw[1]
        req.goal_pose.orientation.x = quat[0]
        req.goal_pose.orientation.y = quat[1]
        req.goal_pose.orientation.z = quat[2]
        req.goal_pose.orientation.w = quat[3]
        self.get_logger().info(f"[Platform] request: {req.goal_pose}")
        future = self.platform_goal_request_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if not result.success:
            self.get_logger().error(f"Platform Goal failed: {result.message}")
            return False
        self.get_logger().info(f"Platform Goal success: {result.message}")
        ## spin until IDLE
        status = 1
        while status != 0:
            req = GoalStatus.Request()
            future = self.platform_goal_status_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            status = result.status
            time.sleep(0.2)
            
    def arm_goal(self, name: str = "", pose: Pose = None):
        if pose is None:
            pose = Pose()
        # Fill request
        req = PoseRequest.Request()
        req.target_pose = pose
        req.message = name
        future = self.arm_pose_request_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Arm Goal request success: {response.message}")
            else:
                self.get_logger().error(f"Arm Goal request failed: {response.message}")
        # spin until goal finish
        while True:
            req = Trigger.Request()
            future = self.arm_pose_finish_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                response = future.result()
                if response.success:
                    break
                else:
                    pass
                    # self.get_logger().info(f"status: {response.message}")
            time.sleep(0.2)


    def cube_info_callback(self, msg: String) -> None:
        pass
        # self.get_logger().info(f"[Color] Detected: {msg.data}")

    def speech_recognition_callback(self, msg: Int32) -> None:
        self.speech_recognition = msg.data


def spin_for_time(node: Node, sec: float) -> None:
    start_time = time.time()
    while time.time() - start_time < sec:
        rclpy.spin_once(node)


def try_grab_up(node: MainNode, operation_char: str) -> bool:
    '''
    return True when success
    '''
    node.arm_select_text(operation_char)
    spin_for_time(node, 1)
    ## Grab
    for retry in range(3):
        node.arm_state_request(ArmStateCmd.GRAB_UP)
        node.arm_state_request(ArmStateCmd.DETECT_POSE)
        spin_for_time(node, 1.5)
        ## Grab success
        if not node.is_text_exist(operation_char):
            return True
    return False
        

def try_putdown(node: MainNode, operation_char: str) -> bool:
    '''
    return True when success
    '''
    node.arm_select_text(operation_char + 'G')
    spin_for_time(node, 1.5)
    node.arm_state_request(ArmStateCmd.PUT_DOWN)
    return True


def offset_unload(node: MainNode) -> None:
    '''
    call this after arm detect pose and platform unload
    '''
    node.arm_select_text("IG")
    spin_for_time(node, 1.0)
    node.platform_request(offset=True)
    while(node.platform_request(offset=False) < 0):
        spin_for_time(node, 0.1)
    node.unload_offseted = True
    # spin_for_time(node, 3.0)
    # node.platform_request(PlatformCmd.UNLOAD)




def main():
    rclpy.init()
    node = MainNode()
    node.create_rate(100)

    tasks = [
        PlatformCmd.LEFT,
        PlatformCmd.TOP,
        PlatformCmd.RIGHT
    ]
    task_finish = False

    ## init
    node.platform_goal(PlatformCmd.HOME)
    # node.audio.beep_ready()
    # node.button.wait_until_start()
    time.sleep(1)
  
    for task in tasks:
        node.platform_goal(task)
        time.sleep(1)

    node.platform_goal(PlatformCmd.HOME)


if __name__ == '__main__':
    main()

