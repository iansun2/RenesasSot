#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
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
            10
        )

        # Create client for PoseRequest service
        self.cli = self.create_client(PoseRequest, 'arm_goal_pose')
        self.finish_cli = self.create_client(Trigger, 'arm_goal_finish')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.status = 0

    def pose_callback(self, msg: PoseStamped):
        if self.status != 0:
            return
        self.get_logger().info(f"Received pose, sending to service...")
        msg.pose.position.x *= 1.05
        msg.pose.position.y *= 1.17
        msg.pose.position.x -= 0.005
        # msg.pose.position.y += 0.02
        msg.pose.position.z += 0.12
        # Fill request
        req = PoseRequest.Request()
        req.target_pose = msg.pose
        req.message = ""

        # Call service asynchronously
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_response_callback)
        self.status = 1

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service success: {response.message}")
            else:
                self.get_logger().warn(f"Service failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        self.status = 2
    
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
        if node.status == 2:
            node.spin_until_goal_finish()
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
