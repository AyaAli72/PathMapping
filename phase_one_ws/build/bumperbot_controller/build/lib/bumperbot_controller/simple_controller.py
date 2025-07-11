import rclpy
from rclpy.node import Node

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.get_logger().info('Simple Controller Node has started!')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

