#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import redis
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Pose
from python_moveit_interface.srv import PoseRequest   # adjust to your actual package
import time

class PoseRelayClient(Node):
    def __init__(self):
        super().__init__('pose_relay_client')
        self.rds = redis.Redis(host='127.0.0.1', port=6379, db=0)
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
        self.skip_pose_cnt = 0
        self.status = 0 # 0: grab, 1: put

    def pose_callback(self, msg: PoseStamped):
        if self.skip_pose_cnt > 0:
            self.skip_pose_cnt -= 1
            return
        # self.get_logger().info(f"Received pose")
        distance = (msg.pose.position.x ** 2 + msg.pose.position.y ** 2) ** 0.5
        msg.pose.position.x *= 1.07
        msg.pose.position.y *= 1.13
        msg.pose.position.x -= 0.015
        # msg.pose.position.y += 0.02
        msg.pose.position.z += 0.17
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
    
    def redis_set_captur_en(self, enable: bool):
        en_str = '1' if enable else '0'
        self.rds.set('capture_en', en_str)

    def get_grab_pose(self) -> Pose | None:
        if self.status == 0 and self.target_pose is not None:
            self.status = 1
            pose = self.target_pose
            self.target_pose = None
            return pose
        return None
    
    def get_put_pose(self) -> Pose | None:
        if self.status == 1 and self.target_pose is not None:
            self.status = 0
            pose = self.target_pose
            self.target_pose = None
            return pose
        return None
    
    def spin_until_poses_drop(self, drop_num: int) -> Pose:
        self.skip_pose_cnt = drop_num
        while rclpy.ok():
            rclpy.spin_once(self)
            time.sleep(0.1)
            if self.skip_pose_cnt <= 0 and self.target_pose is not None:
                return self.target_pose



def main(args=None):
    rclpy.init(args=args)
    node = PoseRelayClient()
    node.create_rate(10)
    node.goal_pose("detect")
    node.spin_until_goal_finish()
    node.redis_set_captur_en(True)
    while rclpy.ok():
        rclpy.spin_once(node)
        time.sleep(0.1)
        if (target_pose := node.get_grab_pose()) is not None:
            target_pose = node.spin_until_poses_drop(3)
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
        elif (target_pose := node.get_put_pose()) is not None:
            target_pose = node.spin_until_poses_drop(3)
            target_pose.position.z += 0.05
            node.goal_pose("", target_pose)
            node.spin_until_goal_finish()
            target_pose.position.z -= 0.1
            node.goal_pose("", target_pose)
            node.spin_until_goal_finish()
            node.goal_pose("gripper_open")
            node.spin_until_goal_finish()
            node.goal_pose("detect")
            node.spin_until_goal_finish()
            node.redis_set_captur_en(False)
            break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
