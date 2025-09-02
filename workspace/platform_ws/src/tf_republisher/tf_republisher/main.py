import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import time

class TfRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.on_timer)

        self.tf_cnt = 0
        self.last_print_tf_cnt = time.time()
        self.get_logger().info(f"start")

    def on_timer(self):
        try:
            # Look up the transform from the 'odom' frame to the 'base_link' frame
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            transform.child_frame_id += '_debug'
            self.tf_broadcaster.sendTransform(transform)
            self.tf_cnt += 1
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")
        if time.time() - self.last_print_tf_cnt >= 1:
            self.last_print_tf_cnt = time.time()
            self.get_logger().info(f"tf re-publish/sec: {self.tf_cnt}")
            self.tf_cnt = 0
        


def main(args=None):
    rclpy.init(args=args)
    node = TfRepublisher()
    node.create_rate(50)
    while rclpy.ok():
        rclpy.spin_once(node)
        time.sleep(0.02)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
