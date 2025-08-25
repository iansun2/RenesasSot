import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import math as m
import numpy as np


class RosNode(Node):
    def __init__(self):
        super().__init__('initial calibration')
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan_filted',
            self.laser_callback,
            2
        )
        self.init_pose_pub = self.create_publisher(
            Pose,
            '/initial_pose',
            2
        )
        ## Variable
        self.current_sample = 0
        self.total_sample = 10
        self.back_fov = m.radians(120)
        self.back_dir = m.radians(-90)
        self.left_fov = m.radians(90)
        self.left_dir = m.radians(180)
        self.back_sample = [] # [distance min, angle]*N
        self.left_sample = [] # [distance min, angle]*N


    def laser_callback(self, msg: LaserScan):
        back_min = [1e12, 0] # distance min, angle
        left_min = [1e12, 0] # distance min, angle
        ## get point from scan
        for idx in range(len(msg.ranges)):
            angle = msg.angle_min + idx * msg._angle_increment
            if angle > self.back_dir - self.back_fov / 2 \
                    and angle < self.back_dir + self.back_fov / 2:
                distance = msg.ranges[idx]
                if distance < back_min[0]:
                    back_min[0] = distance
                    back_min[1] = angle
            if angle > self.left_dir - self.left_fov / 2 \
                    and angle < self.left_dir + self.left_fov / 2:
                distance = msg.ranges[idx]
                if distance < left_min[0]:
                    left_min[0] = distance
                    left_min[1] = angle
        self.back_sample.append(back_min)
        self.left_sample.append(left_min)
        ## end sample
        self.current_sample += 1
        if self.current_sample >= self.total_sample:
            avg_back_angle = np.mean(np.array(self.back_sample[:,1]))
            avg_left_angle = np.mean(np.array(self.left_sample[:,1]))
            print(f"avg back angle: {avg_back_angle}")
            print(f"avg left angle: {avg_left_angle}")
            avg_back_distance = np.mean(np.array(self.back_sample[:,0]))
            avg_left_distance = np.mean(np.array(self.left_sample[:,0]))
            print(f"avg back distance: {avg_back_distance}")
            print(f"avg left distance: {avg_left_distance}")
        


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
