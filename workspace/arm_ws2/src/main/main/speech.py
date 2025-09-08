#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int32

class UARTReaderNode(Node):
    def __init__(self):
        super().__init__('uart_reader_node')

        self.port = '/dev/ttyUSB2'
        self.baudrate = 460800
        self.timeout = 0.1

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.get_logger().info(f'success to open UART: {self.port} @ {self.baudrate} bps')
        except Exception as e:
            self.get_logger().error(f'cannot open UART: {e}')
            self.ser = None

        # 建立 Publisher
        self.publisher_ = self.create_publisher(Int32, '/speech_recognition', 10)

        # 建立 Timer
        self.timer = self.create_timer(0.01, self.read_uart)

    def read_uart(self):
        if self.ser is None:
            return
        try:
            data = self.ser.read(1024)
            if data:
                num = self.parse_data(data)
                if num is not None:
                    self.get_logger().info(f'result: {num}')
                    
                    # 發布訊息
                    msg = Int32()
                    msg.data = num
                    self.publisher_.publish(msg)
                    
                else:
                    self.get_logger().warn(f'unknown data: {data}')
        except Exception as e:
            self.get_logger().error(f'UART read failed: {e}')

    def parse_data(self, data: bytes):
        """identify base on tail pattern 1~5"""
        if data.endswith(b'\x00\x80'):
            return 1
        elif data.endswith(b'x\xf8\x80'):
            return 2
        elif data.endswith(b'\x80\xf8\x80'):
            return 3
        elif data.endswith(b'\xf8\x80\x80'):
            return 4
        elif data.endswith(b'\x00\xf8\x80'):
            return 5
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = UARTReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("interrupt by user")
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
