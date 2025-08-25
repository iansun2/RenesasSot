import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler
import math as m

import json


class FilterNode(Node):
    def __init__(self):
        super().__init__('FilterNode')
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            2
        )
        self.laser_pub = self.create_publisher(
            LaserScan,
            '/scan_filted',
            2
        )
        self.get_logger().info("running")


    def laser_callback(self, msg: LaserScan):
        current_angle = msg.angle_min
        angle_step = msg.angle_increment
        # print("frmae")
        for idx in range(len(msg.ranges)):
            if msg.ranges[idx] < 0.16 and msg.ranges[idx] > 0:
                # print(current_angle)
                msg.ranges[idx] = -1
                # print(msg.ranges[idx])
            current_angle += angle_step
        self.laser_pub.publish(msg)


def main():
    rclpy.init()
    ## ros node
    node = FilterNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
