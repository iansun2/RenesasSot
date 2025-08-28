import rclpy
from rclpy.node import Node
from std_msgs.msg import String, String
from movement_platform_if.srv import Command
import time, json
# from python_moveit_interface.srv import ArmControl
from enum import Enum
from platform_audio import PlatformAudio
from platform_button import PlatformButton

class Target(Enum):
    PLATFORM    = 1
    ARM         = 2


# class ArmStateCmd(Enum):
#     INITIAL_POSE        = "initial_pose"
#     DETECT_POSE         = "detect_pose"
#     GRAB_UP             = "grab_up"
#     PUT_DOWN            = "put_down"

class PlatformCmd(Enum):
    INIT    = "init"
    HOME    = "home"
    LEFT    = "left_c"
    TOP     = "top_c"
    RIGHT   = "right_c"


class MainNode(Node):
    def __init__(self):
        super().__init__('Main')
        self.platform_cmd_cli = self.create_client(Command, '/platform/command')
        # self.arm_state_ctrl_cli = self.create_client(ArmControl, 'arm_control')
        # self.create_subscription(String, "/color/detect", self.color_callback, 2)
        self.audio = PlatformAudio(self)
        self.button = PlatformButton(self, self.audio)
        self.get_logger().info("node init")
        ## Variable
        self.current_platform_pose = PlatformCmd.INIT
        self.current_color = ['Y', 'B', 'G']


    def platform_request(self, dst: PlatformCmd) -> int:
        '''
        call with dst to move
        '''
        req = Command.Request()
        req_dict = {
            "src": self.current_platform_pose.value,
            "dst": dst.value
        }
        req.cmd = json.dumps(req_dict)
        self.get_logger().info(f"[Platform] request: {req.cmd}")
        future = self.platform_cmd_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.current_platform_pose = dst
        return future.result()

    # def arm_state_request(self, command: ArmStateCmd) -> bool:
    #     req = ArmControl.Request()
    #     req.task_name = command.value
    #     self.get_logger().info(f"[Arm] state request: {req.task_name}")
    #     future = self.arm_state_ctrl_cli.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() is not None:
    #         self.get_logger().info(f"[Arm] <Success> Arm state: {command}")
    #         return True
    #     else:
    #         self.get_logger().error(f"[Arm] <Fail> Arm state: {command}")
    #         return False

    # def color_callback(self, msg: String) -> None:
    #     # self.get_logger().info(f"[Color] Detected: {msg.data}")
    #     self.current_color = json.loads(msg.data)


def spin_for_time(node: Node, sec: float) -> None:
    start_time = time.time()
    while time.time() - start_time < sec:
        rclpy.spin_once(node)


def main():
    rclpy.init()
    node = MainNode()
    node.create_rate(10)
    tasks = []
    ## init
    # node.arm_state_request(ArmStateCmd.INITIAL_POSE)
    node.platform_request(PlatformCmd.INIT)
    # node.arm_state_request(ArmStateCmd.INITIAL_POSE)
    node.audio.beep_ready()
    # node.button.wait_until_start()
    spin_for_time(node, 1)
    for color in node.current_color:
        if color == 'Y':
            tasks.append(PlatformCmd.LEFT)
        elif color == 'B':
            tasks.append(PlatformCmd.TOP)
        elif color == 'G':
            tasks.append(PlatformCmd.RIGHT)
    ## task
    for task in tasks:
        ## go to task point
        node.platform_request(task)
        time.sleep(3)
    ## home
    # node.arm_state_request(ArmStateCmd.INITIAL_POSE)
    node.platform_request(PlatformCmd.HOME)


if __name__ == '__main__':
    main()