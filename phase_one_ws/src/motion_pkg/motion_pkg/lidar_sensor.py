import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class ObstacleDistanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_distance_node')

        # Subscribe to LiDAR scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher for the minimum distance in meters
        self.publisher = self.create_publisher(Float32, '/obstacle_distance', 10)

        self.get_logger().info('ObstacleDistanceNode started: publishing closest obstacle distance')

    def scan_callback(self, msg):
        # Filter out 'inf' and 'nan' values
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]

        if valid_ranges:
            min_distance = min(valid_ranges)
            self.get_logger().info(f'Closest obstacle: {min_distance:.2f} meters')

            distance_msg = Float32()
            distance_msg.data = min_distance
            self.publisher.publish(distance_msg)
        else:
            self.get_logger().warn('No valid LiDAR readings to compute distance.')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDistanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
