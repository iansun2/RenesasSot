#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
import tf_transformations
import redis
import numpy as np
import time
import json
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray
from std_msgs.msg import String


class PoseTransformer(Node):
    def __init__(self):
        super().__init__("pose_transformer")

        # Listen to TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("Ready")

    def listen_and_transform(self, src_pose) -> PoseStamped:
        try:
            # self.get_logger().info(f"tranfer request: {src_pose}")
            src_mat = self.pose_to_matrix(src_pose)
            # Get transform link6 -> base_link
            # link6_base_tf = self.tf_buffer.lookup_transform(target_frame = "base_link", source_frame = "camera", time = src_pose.header.stamp)
            link6_base_tf = self.tf_buffer.lookup_transform(
                target_frame="base_link", source_frame="camera", time=rclpy.time.Time()
            )
            link6_base_mat = self.tf_to_matrix(link6_base_tf)
            pose_in_base_matrix = link6_base_mat @ src_mat
            return self.matrix_to_pose(pose_in_base_matrix, src_pose.header)
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {str(e)}")
            return PoseStamped()

    def tf_to_matrix(self, tf: TransformStamped):
        t = tf.transform.translation
        q = tf.transform.rotation
        return np.dot(
            tf_transformations.translation_matrix([t.x, t.y, t.z]),
            tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w]),
        )

    def pose_to_matrix(self, pose: PoseStamped):
        """Convert geometry_msgs/PoseStamped into 4x4 matrix."""
        t = pose.pose.position
        q = pose.pose.orientation
        return np.dot(
            tf_transformations.translation_matrix([t.x, t.y, t.z]),
            tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w]),
        )

    def matrix_to_pose(self, matrix, header):
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


class RedisReceive(Node):
    def __init__(self, tf_node):
        super().__init__("redis_receive_node")
        # Redis connection
        self.rds = redis.Redis(host="127.0.0.1", port=6379, db=0)
        # State
        self.last_detection_id = None
        self.frame_cnt = 0
        self.last_print = time.time()
        self.tf_node = tf_node
        # Topic
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, "/grab_pose", 2)
        self.cube_pose_json_pub = self.create_publisher(String, "/cube_pose_json", 2)
        self.get_logger().info("Ready")

    def send_pose_as_tf(self, pose: PoseStamped, name: str):
        t = TransformStamped()
        t.header = pose.header
        t.child_frame_id = name
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def transfrom_to_base(self, position, quat) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "camera"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.send_pose_as_tf(pose, "tag_cam")
        # self.get_logger().info(f"send req: {pose}")
        return self.tf_node.listen_and_transform(pose)

    def poll_redis(self):
        detection_id = self.rds.get("detection_id")
        if detection_id is None:
            return
        # new img
        if detection_id != self.last_detection_id:
            self.last_detection_id = detection_id
            detections = self.rds.get("detections")
            # Process detections
            detection_obj = json.loads(detections)
            output = {}
            for idx, detection in enumerate(detection_obj):
                corner = np.array(detection["corner"], np.int32)
                center = np.array(detection["center"], np.int32)
                position = np.array(detection["position"])
                rotation = np.array(detection["rotation"]).reshape(3, 3)
                id = int(detection["id"])

                Rx180 = R.from_euler("xz", [180, 90], degrees=True)
                quat = (R.from_matrix(rotation) * Rx180).as_quat()
                self.get_logger().info(f"camera: {position}, {quat}")
                pose_in_base = self.transfrom_to_base(position, quat)
                self.get_logger().info(f"base: {pose_in_base.pose}")
                self.pose_pub.publish(pose_in_base)
                position = pose_in_base.pose.position
                orientation = pose_in_base.pose.orientation
                output[id] = [
                    position.x,
                    position.y,
                    position.z,
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ]
                # self.send_pose_as_tf(pose_in_base, "tag")
                # time.sleep(0.5)
            cube_info = String()
            cube_info.data = json.dumps(output)
            self.cube_pose_json_pub.publish(cube_info)
            # FPS counter
            self.frame_cnt += 1
            if time.time() - self.last_print >= 1:
                self.get_logger().info(
                    f"{self.frame_cnt} fps, {1000 / max(self.frame_cnt, 1):.1f} ms"
                )
                self.last_print = time.time()
                self.frame_cnt = 0


def main(args=None):
    rclpy.init(args=args)
    tf_node = PoseTransformer()
    node = RedisReceive(tf_node)
    node.create_rate(20)
    while rclpy.ok():
        rclpy.spin_once(tf_node)
        node.poll_redis()
        rclpy.spin_once(node)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
