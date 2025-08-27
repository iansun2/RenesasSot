import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import time

class TfRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')

        # Subscriber to original tf
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)

        # Publisher to republished tf
        self.tf_pub = self.create_publisher(
            TFMessage,
            '/tf',
            10)

        # Prevent loop: track frames we already republished
        self.tf_tracker: dict = {"odom->base_link":None, "base_link->laser":None}
        self.tf_cnt = 0
        self.last_print_tf_cnt = time.time()
        self.get_logger().info(f"start")

    def tf_callback(self, msg: TFMessage):
        filtered_transforms = []
        for t in msg.transforms:
            # Create a unique key for each transform
            key = t.header.frame_id + "->" + t.child_frame_id
            # self.get_logger().info(f"key: {key}")
            if key in self.tf_tracker:
                if self.tf_tracker[key] is None or self.tf_tracker[key] != t:
                    # self.get_logger().info(f"valid key: {key}")
                    self.tf_tracker[key] = t
                    ## change name
                    if t.header.frame_id == 'base_link':
                        t.header.frame_id += '_debug'
                    if t.child_frame_id == 'base_link':
                        t.child_frame_id += '_debug'
                    filtered_transforms.append(t)
                    self.tf_cnt += 1

        if filtered_transforms:
            new_msg = TFMessage(transforms=filtered_transforms)
            # self.get_logger().info(f"pub tf: {new_msg}")
            self.tf_pub.publish(new_msg)

            # Mark these as republished to avoid loop
            # for t in filtered_transforms:
            #     key = (t.header.frame_id, t.child_frame_id)
            #     self.already_published.add(key)
        if time.time() - self.last_print_tf_cnt >= 1:
            self.last_print_tf_cnt = time.time()
            self.get_logger().info(f"tf re-publish/sec: {self.tf_cnt}")
            self.tf_cnt = 0


def main(args=None):
    rclpy.init(args=args)
    node = TfRepublisher()
    node.create_rate(100)
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
