import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random

class PointsGenerator(Node):
    def __init__(self):  # Corrected method name
        super().__init__('points_generator')  # Corrected super call
        self.send_current_position = self.create_publisher(Point, 'position', 10)

        self.timer_period = 7  # seconds
        self.timer = self.create_timer(self.timer_period, self.point_send)

    def point_send(self):
        point_msg = Point()
        point_msg.x = random.uniform(-10, 10)
        point_msg.y = random.uniform(-10, 10)
        point_msg.z = 0.0  # For an angle
        self.send_current_position.publish(point_msg)
        self.get_logger().info(f"Publishing Point: x={point_msg.x}, y={point_msg.y}, z={point_msg.z}")

def main(args=None):
    rclpy.init(args=args)
    point_publisher = PointsGenerator()
    rclpy.spin(point_publisher)

    point_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  # Fixed main entry condition
    main()
