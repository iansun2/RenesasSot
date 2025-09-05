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


class RedisCtrl:
    def __init__(self):
        self.rds = redis.Redis(host="127.0.0.1", port=6379, db=0)

    def set_captur_en(self, enable: bool):
        en_str = "1" if enable else "0"
        self.rds.set("capture_en", en_str)


class MainNode(Node):
    def __init__(self):
        super().__init__("Main")
        self.platform_goal_request_cli = self.create_client(GoalRequest, "goal_request")
        self.platform_goal_status_cli = self.create_client(GoalStatus, "goal_status")
        self.arm_pose_request_cli = self.create_client(PoseRequest, "arm_goal_pose")
        self.arm_pose_finish_cli = self.create_client(Trigger, "arm_goal_finish")
        self.create_subscription(
            String, "/cube_pose_json", self.cube_pose_json_callback, 2
        )
        self.create_subscription(
            Int32, "/speech_recognition", self.speech_recognition_callback, 2
        )
        self.redis_ctrl = RedisCtrl()
        # self.audio = PlatformAudio(self)
        # self.button = PlatformButton(self, self.audio)
        self.get_logger().info("node init")
        # Variable
        self.cube_pose: dict = {}
        self.cube_status: dict = {
            0: PlatformCmd.HOME,
            1: PlatformCmd.HOME,
            2: PlatformCmd.HOME,
            3: PlatformCmd.HOME,
        }
        self.speech_recognition: int = None
        # Config
        with open("platform.yaml", "r") as file:
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
        # spin until IDLE
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
        self.get_logger().info("arm goal request")
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

    def cube_pose_json_callback(self, msg: String) -> None:
        self.cube_pose = json.loads(msg.data)

    def speech_recognition_callback(self, msg: Int32) -> None:
        self.speech_recognition = msg.data

    def update_cube_status_from_camera(self, platform_locate: PlatformCmd):
        self.redis_ctrl.set_captur_en(True)
        spin_for_time(self, 1)
        for id in self.cube_pose:
            if self.cube_info[id] != PlatformCmd.UNLOAD:
                self.cube_status[id] = platform_locate
        self.redis_ctrl.set_captur_en(False)

    def set_cube_status_finish(self, id: int) -> None:
        self.cube_status[id] = PlatformCmd.UNLOAD

    def get_cube_status(self, id: int) -> PlatformCmd:
        return self.cube_status[id]

    def get_cube_pose(self, id: int) -> Pose | None:
        if not id in self.cube_pose:
            return None
        info = self.cube_pose[id]
        pose = Pose()
        pose.position.x = info[0]
        pose.position.y = info[1]
        pose.position.z = info[2]
        pose.orientation.x = info[3]
        pose.orientation.y = info[4]
        pose.orientation.z = info[5]
        pose.orientation.w = info[6]
        return pose

    def spin_until_speech_cmd(self) -> int:
        while self.speech_recognition is None:
            rclpy.spin_once(self)
            time.sleep(0.1)
        ret = self.speech_recognition
        self.speech_recognition = None
        return ret


def grab_up(node: MainNode, pose: Pose):
    node.arm_goal(name="gripper_open")
    node.arm_goal(pose=pose)
    pose.position.z -= 0.1
    node.arm_goal(pose=pose)
    node.arm_goal(name="gripper_close")
    node.arm_goal(name="detect")


def put_down(node: MainNode, pose: Pose):
    pose.position.z += 0.05
    node.arm_goal(pose=pose)
    pose.position.z -= 0.1
    node.arm_goal(pose=pose)
    node.arm_goal(name="gripper_open")
    node.arm_goal(name="detect")
    node.arm_goal(name="gripper_close")


def main():
    rclpy.init()
    node = MainNode()
    node.create_rate(100)

    platform_points = [PlatformCmd.LEFT, PlatformCmd.TOP, PlatformCmd.RIGHT]

    # init
    node.redis_ctrl.set_captur_en(False)
    node.arm_goal(name="home")
    node.platform_goal(PlatformCmd.HOME)
    node.arm_goal(name="detect")
    # node.audio.beep_ready()
    # node.button.wait_until_start()
    time.sleep(1)

    node.speech_recognition = 2
    while rclpy.ok():
        target_cube = node.spin_until_speech_cmd() - 2  # map (1,5) to (-1,3)
        cube_status = node.get_cube_status(target_cube)
        skip_move = False
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
                node.update_cube_status_from_camera(point)
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
                node.update_cube_status_from_camera(cube_status)
            # grab up
            pose = node.get_cube_pose(target_cube)
            grab_up(node, pose)
            # platform move to unload
            node.platform_goal(PlatformCmd.UNLOAD)
            # put down
            node.redis_ctrl.set_captur_en(True)
            spin_for_time(node, 1)
            node.redis_ctrl.set_captur_en(False)
            pose = node.get_cube_pose(target_cube)
            put_down(node, pose)
            # update cube status
            node.set_cube_status_finish(target_cube)
    node.get_logger().info("all down")
    node.arm_goal(name="home")
    node.platform_goal(PlatformCmd.HOME)


if __name__ == "__main__":
    main()
