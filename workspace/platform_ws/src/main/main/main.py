import rclpy
from rclpy.node import Node
from std_msgs.msg import String, String
from movement_platform_if.srv import Command
import time, json
from python_moveit_interface.srv import ArmControl
from enum import Enum
from platform_audio import PlatformAudio
from platform_button import PlatformButton

class Target(Enum):
    PLATFORM    = 1
    ARM         = 2


class ArmStateCmd(Enum):
    INITIAL_POSE        = "initial_pose"
    DETECT_POSE         = "detect_pose"
    GRAB_UP             = "grab_up"
    PUT_DOWN            = "put_down"
    # GRAB_DETECT_POSE_LEFT_UP    = "grab_detect_pose_left_up"
    # PUT_LEFT_UP_BLOCK   = "put_left_up_block"

class PlatformCmd(Enum):
    INIT    = "init"
    HOME    = "home"
    LEFT    = "left"
    TOP     = "top"
    RIGHT   = "right"
    UNLOAD  = "unload"
    # UNLOAD  = "arm_putdown"
    PICKUP  = "arm_pickup"
    FAILED  = "failed"  # for block state machine


class MainNode(Node):
    def __init__(self):
        super().__init__('Main')
        self.platform_cmd_cli = self.create_client(Command, '/platform/command')
        self.arm_state_ctrl_cli = self.create_client(ArmControl, 'arm_control')
        self.create_subscription(String, "/color/detect", self.color_callback, 2)
        self.create_subscription(String, "/ocr_result/priority", self.text_callback, 2)
        self.arm_text_sel_pub = self.create_publisher(String, "/ocr_selected_text", 2)
        self.audio = PlatformAudio(self)
        self.button = PlatformButton(self, self.audio)
        self.get_logger().info("node init")
        ## Variable
        self.current_platform_pose = PlatformCmd.INIT
        self.current_text = "" ## "FIRA" format
        self.current_level_text = "" ## "FIRA" format
        self.text_detect_buffer = []
        self.text_info = {
            'F': PlatformCmd.INIT,
            'I': PlatformCmd.INIT,
            'R': PlatformCmd.INIT,
            'A': PlatformCmd.INIT
        }
        self.unload_offseted = False


    def platform_request(self, dst: PlatformCmd | None = None, offset: bool | None = None) -> int:
        '''
        call with dst to move
        call with offset = true to request platform offset at unload and move to unload
        call with offset = false to get platform offset status (finish: return > 0)
        '''
        req = Command.Request()
        ## Request (offset) cmd
        if dst is None and offset is not None:
            req_dict = {
                "offset": offset
            }
        ## Request (src,dst) cmd
        else:
            req_dict = {
                "src": self.current_platform_pose.value,
                "dst": dst.value
            }
            self.current_platform_pose = dst

        req.cmd = json.dumps(req_dict)
        self.get_logger().info(f"[Platform] request: {req.cmd}")
        future = self.platform_cmd_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().result

    def arm_state_request(self, command: ArmStateCmd, blocking = True) -> bool:
        req = ArmControl.Request()
        req.task_name = command.value
        self.get_logger().info(f"[Arm] state request: {req.task_name}")
        future = self.arm_state_ctrl_cli.call_async(req)
        if blocking:
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f"[Arm] <Success> Arm state: {command}")
                return True
            else:
                self.get_logger().error(f"[Arm] <Fail> Arm state: {command}")
                return False
        else:
            return True

    def arm_select_text(self, text: str):
        self.get_logger().info(f"[Arm] Select text: {text}")
        msg = String()
        msg.data = text
        self.arm_text_sel_pub.publish(msg)

    def color_callback(self, msg: String) -> None:
        self.get_logger().info(f"[Color] Detected: {msg.data}")


    def text_callback(self, msg: String) -> None:
        # self.get_logger().info(f"[Text] Detected: {msg.data}") 
        # self.text_detect_buffer.append(msg.data)
        # if(len(self.text_detect_buffer) >= 3):
        #     compare = self.text_detect_buffer[0]
        #     ## compare buffered text, return when not match
        #     for text_detect in self.text_detect_buffer:
        #         if compare != text_detect:
        #             return
        #     self.text_detect_buffer = []
        # ## sample not enough
        # else:
        #     return
        # ## set filted text
        self.current_text = msg.data

    def update_text_info(self, level: PlatformCmd = PlatformCmd.INIT, put: bool = False, failed: bool = False) -> str:
        '''
            call put=False after arm detect pose, will store current_level_text and set info to level\n
            call put=True after arm put_down pose, will pop current_level_text and set info to unload\n
            call put=True and failed=True when grab failed, will pop current level_text and set info to failed\n
            return first char of current_text
        '''
        self.get_logger().info(f"current text: {self.current_text}")
        self.get_logger().info(f"before current level: {self.current_level_text}")
        return_char = ""
        ## grab mode
        if not put:
            ## set current_level_text
            self.current_level_text = self.current_text
            ## set text_info to current_level_text
            for c in self.current_level_text:
                ## only update not failed char
                if self.text_info[c] != PlatformCmd.FAILED:
                    self.text_info[c] = level
                    print(f"update char: {c}")
                    ## set return char when not set
                    if return_char == "":
                        print(f"set return char: {return_char}")
                        return_char = c
        ## put mode
        elif self.current_level_text != "":
            if len(self.current_level_text):
                ## pop from current_level_text
                c = self.current_level_text[0]
                self.current_level_text = self.current_level_text[1:]
                ## set pop char in text_info's state 
                if not failed:
                    self.text_info[c] = level
                else:
                    self.text_info[c] = PlatformCmd.FAILED
                return_char = c
            else:
                self.get_logger().error("(update text info): current level text is empty")
        self.get_logger().info(f"after current level: {self.current_level_text}")
        self.get_logger().info(f"text info: {self.text_info}")
        self.get_logger().info(f"first char: {return_char}")
        return return_char
    
    def is_text_exist(self, text: str) -> bool:
        return text in self.current_text

    def is_level_text_empty(self) -> bool:
        '''
            True: current level text is empty
        '''
        print(f"current level text count: {len(self.current_level_text)}")
        return len(self.current_level_text) == 0

    def is_task_finish(self) -> bool:
        '''
            all text in unload
        '''
        end_condi = [PlatformCmd.UNLOAD, PlatformCmd.FAILED]
        return self.text_info['F'] in end_condi \
            and self.text_info['I'] in end_condi \
            and self.text_info['R'] in end_condi \
            and self.text_info['A'] in end_condi


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
    node.arm_state_request(ArmStateCmd.INITIAL_POSE)
    node.platform_request(PlatformCmd.INIT)
    node.audio.beep_ready()
    node.button.wait_until_start()
    node.arm_state_request(ArmStateCmd.DETECT_POSE)
    for task in tasks:
        ## go to task point
        node.platform_request(task)
        ## loop
        while True:
            ## Detect
            node.arm_state_request(ArmStateCmd.DETECT_POSE)
            spin_for_time(node, 1)
            operation_char = node.update_text_info(level = task)
            ## has text in this level
            if operation_char != "":
                grab_succ = try_grab_up(node, operation_char)
                ## grab success
                if grab_succ:
                    ## Unload
                    node.platform_request(PlatformCmd.UNLOAD)
                    if not node.unload_offseted:
                        offset_unload(node)
                    putdown_succ = try_putdown(node, operation_char)
                    node.update_text_info(level = PlatformCmd.UNLOAD, put = True)
                ## grab failed
                else:
                    node.get_logger().warn(f"grab up: {operation_char} failed")
                    node.update_text_info(put = True, failed = True)
            ## finish when all text at unload
            if node.is_task_finish():
                task_finish = True
                break
            ## this level finish
            if node.is_level_text_empty():
                break
            ## back to level
            node.platform_request(task)
        ## finish
        if task_finish:
            break
    ## home
    node.arm_state_request(ArmStateCmd.INITIAL_POSE, blocking = False)
    node.platform_request(PlatformCmd.HOME)


if __name__ == '__main__':
    main()