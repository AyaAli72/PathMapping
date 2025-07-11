import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"  # Adjust to the desired frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "red_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
      
        marker.pose.position.x = 10.0  # x-coordinate set to 0
        marker.pose.position.y = 0.0  # y-coordinate set to 0
        marker.pose.position.z = 0.2  # z-coordinate set to 0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0

        marker.scale.x = 1.0  # Sphere diameter
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.0  # Fully opaque
        self.publisher.publish(marker)
        self.get_logger().info('Published a red marker')

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
