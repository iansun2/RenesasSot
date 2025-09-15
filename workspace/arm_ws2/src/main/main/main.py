import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from movement_platform_if.srv import GoalRequest, GoalStatus
import time
import yaml
import json
from python_moveit_interface.srv import PoseRequest
from std_srvs.srv import Trigger
from enum import Enum
from .redis_receive import RedisReceiver
from threading import Thread
# from .moveit_control import ArmController

from pymoveit2 import MoveIt2, MoveIt2State
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
# from platform_audio import PlatformAudio
# from platform_button import PlatformButton
import math as m
from geometry_msgs.msg import PoseStamped, Pose
from tf_transformations import quaternion_from_euler
import redis


class PlatformCmd(Enum):
    HOME = "home"
    LEFT = "left"
    TOP = "top"
    RIGHT = "right"
    UNLOAD = "unload"


def spin_for_time(node: Node, sec: float) -> None:
    start_time = time.time()
    while time.time() - start_time < sec:
        rclpy.spin_once(node)
        time.sleep(0.05)


class MainNode(Node):
    def __init__(self):
        super().__init__("Main")
        self.platform_goal_request_cli = self.create_client(GoalRequest, "goal_request")
        self.platform_goal_status_cli = self.create_client(GoalStatus, "goal_status")
        self.create_subscription(
            Int32, "/speech_recognition", self.speech_recognition_callback, 2
        )
        self.redis_receiver = RedisReceiver()
        self.init_moveit()
        # self.arm_controller = ArmController()
        # self.audio = PlatformAudio(self)
        # self.button = PlatformButton(self, self.audio)
        # Variable
        self.cube_status: dict[int, PlatformCmd] = {
            0: PlatformCmd.HOME,
            1: PlatformCmd.HOME,
            2: PlatformCmd.HOME,
            3: PlatformCmd.HOME,
        }
        self.speech_recognition: int = None
        # Config
        with open("platform.yaml", "r") as file:
            self.platform_config = yaml.safe_load(file)
        # Ready
        self.get_logger().info("node init")

    def init_moveit(self):
        self._arm = MoveIt2(
            node=self,
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            base_link_name="base_arm",
            end_effector_name="link6",              
            group_name="small_arm",
            use_move_group_action=True
        )
        self._gripper = MoveIt2(
            node=self,
            joint_names=["gripper_joint1"],
            base_link_name="base_arm",        
            group_name="gripper",
            end_effector_name="link6",     
            use_move_group_action=True
        )
        # Config
        srdf_package_dir = get_package_share_directory('small_arm_moveit_config')
        tree = ET.parse(srdf_package_dir + '/config/small_arm.srdf')
        root = tree.getroot()
        self._named_poses = {}
        for group_state in root.findall('group_state'):
            ## ignore when not small_arm group
            if group_state.attrib['group'] != 'small_arm' and group_state.attrib['group'] != 'gripper':
                continue
            named_pose = {'name':[], 'pose':[]}
            ## read joint
            for joint in group_state.findall('joint'):
                # print(joint.attrib)
                named_pose['name'].append(joint.attrib['name'])
                named_pose['pose'].append(float(joint.attrib['value']))
            self._named_poses[group_state.attrib['name']] = named_pose    

    def goal_request_with_retry(self, pose: str | Pose) -> bool:
        retry = 0
        while not self.goal_request(pose):
            retry += 1
            self.get_logger().info(f"goal retry: {retry}")
            time.sleep(0.5)
            if retry >= 5:
                return False
        return True

    def goal_request(self, pose: str | Pose) -> bool:
        controller: MoveIt2 = None
        # Named pose
        if isinstance(pose, str):
            joint_state = self._named_poses[pose]
            if pose == "gripper_open" or pose == "gripper_close":
                self._gripper.move_to_configuration(joint_state['pose'], joint_state['name'])
                controller = self._gripper
            else:
                self._arm.move_to_configuration(joint_state['pose'], joint_state['name'])
                controller = self._arm
        # Normal pose
        else:
            self._arm.move_to_pose(
                position=pose.position,
                quat_xyzw=pose.orientation,
                frame_id="base_arm",             # Reference frame
                tolerance_position=0.001,         # Position tolerance
                tolerance_orientation=0.001,      # Orientation tolerance
                # cartesian=cartesian,
                # cartesian_max_step=cartesian_max_step,
                # cartesian_fraction_threshold=cartesian_fraction_threshold,
            )
            controller = self._arm
        # Wait until finish
        controller.wait_until_executed()
        error_code = controller.get_last_execution_error_code()
        if error_code.val != 1:
            self.get_logger().error(f"Execution error code: {error_code}")
            return False
        else:
            return True

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
        # spin until IDLE
        status = 1
        while status != 0:
            req = GoalStatus.Request()
            future = self.platform_goal_status_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            status = result.status
            time.sleep(0.2)

    def speech_recognition_callback(self, msg: Int32) -> None:
        self.speech_recognition = msg.data

    def spin_until_camera_drop(
        self, drop: int, timeout: float = 5
    ) -> dict[int, Pose] | None:
        ret = None
        self.redis_receiver.set_captur_en(True)
        st = time.time()
        while time.time() - st < timeout:
            rclpy.spin_once(self.redis_receiver)
            poses = self.redis_receiver.poll_redis()
            if poses is not None:
                drop -= 1
            # use frame [drop+1]
            if drop < 0:
                ret = poses
                break
            time.sleep(0.05)
        self.redis_receiver.set_captur_en(False)
        return ret

    def update_cube_status_from_camera(
        self, platform_locate: PlatformCmd, drop: int
    ) -> dict[int, Pose] | None:
        poses = self.spin_until_camera_drop(drop)
        if poses is None:
            self.get_logger().error("camera get pose timeout")
            return None
        for id in poses:
            if self.cube_status[id] != PlatformCmd.UNLOAD:
                self.cube_status[id] = platform_locate
        return poses

    def set_cube_status_finish(self, id: int) -> None:
        self.cube_status[id] = PlatformCmd.UNLOAD

    def get_cube_status(self, id: int) -> PlatformCmd:
        return self.cube_status[id]

    def spin_until_cube_pose(self, id: int, drop: int, timeout: float) -> Pose | None:
        poses = self.spin_until_camera_drop(drop)
        pose = None
        self.redis_receiver.set_captur_en(True)
        st = time.time()
        while time.time() - st < timeout:
            rclpy.spin_once(self.redis_receiver)
            if poses is not None and id in poses:
                pose = poses[id]
                break
            else:
                poses = self.redis_receiver.poll_redis()
            time.sleep(0.05)
        self.redis_receiver.set_captur_en(False)
        return pose

    def spin_until_speech_cmd(self) -> int:
        while self.speech_recognition is None:
            rclpy.spin_once(self)
            time.sleep(0.1)
        ret = self.speech_recognition
        self.speech_recognition = None
        return ret

    def pose_compensate(self, pose: Pose) -> Pose:
        distance = (pose.position.x**2 + pose.position.y**2) ** 0.5
        pose.position.x *= 1.07
        pose.position.y *= 1.13
        pose.position.x -= 0.015
        # msg.pose.position.y += 0.02
        pose.position.z += 0.17
        pose.position.z += (distance - 0.19) * 0.2
        return pose


def grab_up(node: MainNode, pose: Pose):
    pose = node.pose_compensate(pose)
    node.goal_request_with_retry("gripper_open")
    node.goal_request_with_retry(pose)
    pose.position.z -= 0.1
    node.goal_request_with_retry(pose)
    node.goal_request_with_retry("gripper_close")
    node.goal_request_with_retry("detect")


def put_down(node: MainNode, pose: Pose):
    pose = node.pose_compensate(pose)
    pose.position.z += 0.05
    node.goal_request_with_retry(pose)
    pose.position.z -= 0.1
    node.goal_request_with_retry(pose)
    node.goal_request_with_retry("gripper_open")
    node.goal_request_with_retry("detect")
    node.goal_request_with_retry("gripper_close")


def main():
    main2()
    rclpy.init()
    node = MainNode()
    node.create_rate(100)

    platform_points = [PlatformCmd.LEFT, PlatformCmd.TOP, PlatformCmd.RIGHT]

    # init
    node.get_logger().info("Start in 3 sec")
    node.redis_receiver.set_captur_en(False)
    time.sleep(3)
    node.goal_request("home")
    node.goal_request("gripper_open")
    node.goal_request("gripper_close")
    #input("press to detect")
    #node.arm_controller.goal_request("detect")
    #input("press to continue")
    node.platform_goal(PlatformCmd.HOME)
    node.goal_request("detect")
    # node.audio.beep_ready()
    # node.button.wait_until_start()
    time.sleep(1)
    node.get_logger().info("Ready to receive command")

    # node.speech_recognition = 2
    while rclpy.ok():
        target_cube = node.spin_until_speech_cmd() - 2  # map (1,5) to (-1,3)
        cube_status = node.get_cube_status(target_cube)
        skip_move = False
        poses: dict[int, Pose] = {}
        # cube is finished
        if cube_status == PlatformCmd.UNLOAD:
            node.get_logger().warn("cube is finished")
            continue
        # need to find cube
        if cube_status == PlatformCmd.HOME:
            skip_move = True
            # platform go to each point to find cube
            for point in platform_points:
                node.platform_goal(point)
                poses = node.update_cube_status_from_camera(point, 5)
                cube_status = node.get_cube_status(target_cube)
                if cube_status != PlatformCmd.HOME:
                    break
        # still not found
        if cube_status == PlatformCmd.HOME:
            node.get_logger().error("cube not found")
            continue
        # need to grab
        if cube_status != PlatformCmd.UNLOAD:
            # platform move to cube
            if not skip_move:
                node.platform_goal(cube_status)
                poses = node.update_cube_status_from_camera(cube_status, 5)
            # grab up
            pose = poses[target_cube]
            grab_up(node, pose)
            # platform move to unload
            node.platform_goal(PlatformCmd.UNLOAD)
            # put down
            pose = node.spin_until_cube_pose(target_cube, 5, 5)
            if not pose:
                node.get_logger().error("failed to get put down pose")
                break
            put_down(node, pose)
            # update cube status
            node.set_cube_status_finish(target_cube)
    node.get_logger().info("all down")
    node.arm_controller.goal_request(name="top")
    node.arm_controller.goal_request(name="home")
    node.platform_goal(PlatformCmd.HOME)


def main2():
    rclpy.init()
    node = MainNode()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # init
    node.get_logger().info("Start in 3 sec")
    node.redis_receiver.set_captur_en(False)
    time.sleep(3)
    node.goal_request("home")
    node.goal_request("gripper_open")
    node.goal_request("gripper_close")
    node.goal_request("detect")
    time.sleep(1)

    # node.speech_recognition = 2
    while rclpy.ok():
        node.get_logger().info("Ready to receive command")
        target_cube = node.spin_until_speech_cmd() - 2  # map (1,5) to (-1,3)
        cube_status = node.get_cube_status(target_cube)
        skip_move = False
        poses: dict[int, Pose] = {}
        # cube is finished
        if cube_status == PlatformCmd.UNLOAD:
            node.get_logger().warn("cube is finished")
            continue
        poses = node.update_cube_status_from_camera(PlatformCmd.TOP, 5)
        cube_status = node.get_cube_status(target_cube)
        # still not found
        if cube_status == PlatformCmd.HOME:
            node.get_logger().error("cube not found")
            continue
        # need to grab
        if cube_status != PlatformCmd.UNLOAD:
            # grab up
            pose = poses[target_cube]
            node.get_logger().info(f"grab up: {pose}")
            grab_up(node, pose)
            # spin_for_time(node, 3)
            # put down
            pose = node.spin_until_cube_pose(target_cube, 5, 10)
            if not pose:
                node.get_logger().error("failed to get put down pose")
                break
            node.get_logger().info(f"put down: {pose}")
            put_down(node, pose)
            # update cube status
            node.set_cube_status_finish(target_cube)
    node.get_logger().info("all down")
    node.goal_request("home")
    exit()




if __name__ == "__main__":
    main()
