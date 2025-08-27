from movement_platform_if.srv import Command
from rclpy.node import Node
import rclpy
import json


class CommandNode(Node):
    def __init__(self):
        super().__init__('CommandNode')
        self.cmd_cli = self.create_client(Command, '/platform/command')
        while not self.cmd_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info("node init")

    def send_request(self, src, dst):
        req = Command.Request()
        req_dict = {
            "src": src,
            "dst": dst
        }
        req.cmd = json.dumps(req_dict)
        self.get_logger().info(req.cmd)
        future = self.cmd_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = CommandNode()
    node.create_rate(100)

    tasks = [
        ['init', 'init'],
        ['home', 'left'],
        ['left', 'top'],
        ['top', 'unload'],
        ['unload', 'right'],
        ['right', 'unload'],
        ['unload', 'home']
    ]

    # tasks = [
    #     ['init', 'init'],
    #     ['init', 'test'],
    # ]
    
    # tasks = [
    #     ['init', 'init'],
    #     ['home', 'left_c'],
    #     ['left_c', 'top_c'],
    #     ['top_c', 'right_c'],
    #     ['right_c', 'home']
    # ]

    for task in tasks:
        node.send_request(task[0], task[1])


if __name__ == '__main__':
    main()