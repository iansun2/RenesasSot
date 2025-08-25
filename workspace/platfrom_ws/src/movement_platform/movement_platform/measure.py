import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import math as m
import numpy as np


class RosNode(Node):
    def __init__(self):
        super().__init__('Measure')
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan_filted',
            self.laser_callback,
            2
        )
    
    def laser_callback(self, msg: LaserScan):
        min_dist = [1e12, 0] # dist, angle
        for idx in range(len(msg.ranges)):
            angle = msg.angle_min + idx * msg._angle_increment
            distance = msg.ranges[idx]
            if distance > 0 and distance < min_dist[0]:
                min_dist = [distance, angle]
        print(f"angle: {m.degrees(min_dist[1])}, distance: {min_dist[0]}")


def main():
    rclpy.init()
    ## ros node
    node = RosNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
