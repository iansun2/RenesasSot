import rclpy
from rclpy.time import Duration, Time
from rclpy.node import Node
import tf2_ros
import tf_transformations
import redis
import numpy as np
import time
import json
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose


class RedisReceiver(Node):
    def __init__(self):
        super().__init__("redis_receiver")
        # Redis connection
        self._rds = redis.Redis(host="127.0.0.1", port=6379, db=0)
        # State
        self._last_detection_id = None
        # Listen to TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        # Ready
        self.create_rate(100)
        self.get_logger().info("Ready")

    def _transfrom_to_base(self, position: list[float], quat: list[float], image_time: Time) -> Pose | None:
        # Array to T matrix
        T_pose_in_camera = np.dot(
            tf_transformations.translation_matrix(position),
            tf_transformations.quaternion_matrix(quat),
        )
        try:
            # Get camera to base TF
            #camera_to_base_tf = self._tf_buffer.lookup_transform(
            #    target_frame="base_arm", source_frame="camera", time=Time() )
            # TF to T matrix
            #t = camera_to_base_tf.transform.translation
            #q = camera_to_base_tf.transform.rotation
            #T_cam_base = np.dot(
            #    tf_transformations.translation_matrix([t.x, t.y, t.z]),
            #    tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w]),
            #)
            #self.get_logger().info(f"T cam base: {t}, {q}")
            T_cam_base = np.dot(
                tf_transformations.translation_matrix([-0.00013807144901078946, 0.17668304508742544, 0.2615420050909273]),
                tf_transformations.quaternion_matrix([0.9997942354691138, -0.0034103228221833283, 0.001433961791665533, 0.019944928542328964]),
            )
            # Transform pose to base_arm frame
            T_pose_in_base = T_cam_base @ T_pose_in_camera
            # T matrix to Pose
            position = tf_transformations.translation_from_matrix(T_pose_in_base)
            quat = tf_transformations.quaternion_from_matrix(T_pose_in_base)
            pose_in_base = Pose()
            pose_in_base.position.x = position[0]
            pose_in_base.position.y = position[1]
            pose_in_base.position.z = position[2]
            pose_in_base.orientation.x = quat[0]
            pose_in_base.orientation.y = quat[1]
            pose_in_base.orientation.z = quat[2]
            pose_in_base.orientation.w = quat[3]
            return pose_in_base
        # Exception
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {str(e)}")
            return None

    def poll_redis(self) -> dict[int, Pose] | None:
        detection_id = self._rds.get("detection_id")
        if detection_id is None or detection_id == self._last_detection_id:
            return None
        # Receive new detection
        self._last_detection_id = detection_id
        detections = self._rds.get("detections")
        # Process detections
        detection_obj = json.loads(detections)
        output = {}
        for idx, detection in enumerate(detection_obj):
            position = np.array(detection["position"])
            rotation = np.array(detection["rotation"]).reshape(3, 3)
            id = int(detection["id"])
            # Calculate pose in camera
            Roffset = R.from_euler("xz", [180, 90], degrees=True)
            quat = (R.from_matrix(rotation) * Roffset).as_quat()
            self.get_logger().info(f"camera: {id},  {position}, {quat}")
            # Calculate pose in base
            current_time = self.get_clock().now()
            image_time = current_time - Duration(nanoseconds=300_000_000)
            pose_in_base = self._transfrom_to_base(position, quat, image_time)
            if pose_in_base is None:
                self.get_logger().warn("drop detection because transform failed")
                return None
            #self.get_logger().info(f"base: {pose_in_base}")
            output[id] = pose_in_base
        return output

    def set_captur_en(self, enable: bool):
        en_str = "1" if enable else "0"
        self._rds.set("capture_en", en_str)
