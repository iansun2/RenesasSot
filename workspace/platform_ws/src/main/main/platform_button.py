from irobot_create_msgs.msg import InterfaceButtons
from rclpy.node import Node
import time, rclpy
from platform_audio import PlatformAudio

class PlatformButton:
    def __init__(self, node: Node, audio: PlatformAudio):
        self.node = node
        self.audio = audio
        self.button_sub = self.node.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self.button_callback,
            2
        )
        self.start = False
    
    def button_callback(self, msg: InterfaceButtons):
        btn1 = msg.button_1
        btn2 = msg.button_2
        # print(f"Button1: {btn1}")
        if (btn1.is_pressed \
                and btn1.last_pressed_duration.sec == 0 \
                and btn1.last_pressed_duration.nanosec == 0) \
            or (btn2.is_pressed \
                and btn2.last_pressed_duration.sec == 0 \
                and btn2.last_pressed_duration.nanosec == 0):
            self.node.get_logger().info("button start")
            self.audio.beep_start()
            time.sleep(2)
            self.start = True
    
    def wait_until_start(self):
        while not self.start:
            rclpy.spin_once(self.node)