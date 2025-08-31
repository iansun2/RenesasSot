#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs

from python_moveit_interface.srv import PoseTransform
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf_transformations


def tf_to_matrix(tf: TransformStamped):
    t = trans.transform.translation
    q = trans.transform.rotation
    return np.dot(
        tf_transformations.translation_matrix([t.x, t.y, t.z]),
        tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    )


def pose_to_matrix(pose: PoseStamped):
    """Convert geometry_msgs/PoseStamped into 4x4 matrix."""
    t = pose.pose.position
    q = pose.pose.orientation
    return np.dot(
        tf_transformations.translation_matrix([t.x, t.y, t.z]),
        tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    )


def matrix_to_pose(matrix, header):
    """Convert 4x4 matrix back to PoseStamped."""
    trans = matrix[0:3, 3]
    quat = tf_transformations.quaternion_from_matrix(matrix)
    pose = PoseStamped()
    pose.header = header
    pose.header.frame_id = "base_link"
    pose.pose.position.x = trans[0]
    pose.pose.position.y = trans[1]
    pose.pose.position.z = trans[2]
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    return pose


class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer')

        # Listen to TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.srv = self.create_service(PoseTransform, 'camera_pose_to_base', self.handle_request)
        self.get_logger().info("Ready")

    def handle_request(self, request, response):
        try:
            src_pose = request.pose
            self.get_logger().info(f"tranfer request: {src_pose}")
            src_mat = pose_to_matrix(src_pose)
            # Get transform link6 -> base_link
            link6_base_tf = self.tf_buffer.lookup_transform(target_frame = "base_link", source_frame = "camera", time = src_pose.header.stamp)
            link6_base_mat = tf_to_matrix(link6_base_tf)
            pose_in_base_matrix = link6_base_mat @ src_mat
            response.pose = matrix_to_pose(pose_in_base_matrix)
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {str(e)}")
            response.pose = PoseStamped()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()
    rate = node.create_rate(100)
    while rclpy.ok():
        rclpy.spin_once(node)
        # rate.sleep()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
