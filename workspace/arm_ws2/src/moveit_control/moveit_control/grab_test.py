#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Pose
from python_moveit_interface.srv import PoseRequest   # adjust to your actual package
import time

class PoseRelayClient(Node):
    def __init__(self):
        super().__init__('pose_relay_client')
        # Subscribe to PoseStamped topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/grab_pose',
            self.pose_callback,
            2
        )
        # Create client for PoseRequest service
        self.cli = self.create_client(PoseRequest, 'arm_goal_pose')
        self.finish_cli = self.create_client(Trigger, 'arm_goal_finish')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.target_pose: Pose = None

    def pose_callback(self, msg: PoseStamped):
        # self.get_logger().info(f"Received pose")
        distance = (msg.pose.position.x ** 2 + msg.pose.position.y ** 2) ** 0.5
        msg.pose.position.x *= 1.07
        msg.pose.position.y *= 1.13
        msg.pose.position.x -= 0.015
        # msg.pose.position.y += 0.02
        msg.pose.position.z += 0.15
        msg.pose.position.z += (distance - 0.19) * 0.2
        self.target_pose = msg.pose
    
    def goal_pose(self, named_pose: str, pose: Pose = None):
        if pose is None:
            pose = Pose()
        # Fill request
        req = PoseRequest.Request()
        req.target_pose = pose
        req.message = named_pose
        # Call service asynchronously
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Goal request success: {response.message}")
            else:
                self.get_logger().warn(f"Goal request failed: {response.message}")


    def spin_until_goal_finish(self):
        while True:
            req = Trigger.Request()
            future = self.finish_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                response = future.result()
                if response.success:
                    break
                else:
                    self.get_logger().info(f"status: {response.message}")
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = PoseRelayClient()
    node.create_rate(10)
    while rclpy.ok():
        rclpy.spin_once(node)
        time.sleep(0.1)
        if node.target_pose is not None:
            target_pose = node.target_pose
            node.goal_pose("gripper_open")
            node.spin_until_goal_finish()
            node.goal_pose("", target_pose)
            node.spin_until_goal_finish()
            target_pose.position.z -= 0.1
            node.goal_pose("", target_pose)
            node.spin_until_goal_finish()
            node.goal_pose("gripper_close")
            node.spin_until_goal_finish()
            node.goal_pose("detect")
            node.spin_until_goal_finish()
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
