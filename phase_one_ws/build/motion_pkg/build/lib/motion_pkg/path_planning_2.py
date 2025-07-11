import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np
import random
import math
import time

class NodeStruct:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

class DummyPose:
    def __init__(self, x, y, yaw_deg):
        class Loc:
            pass
        class Rot:
            pass
        self.location = Loc()
        self.rotation = Rot()
        self.location.x = x
        self.location.y = y
        self.rotation.yaw = yaw_deg

class RRTStar(Node):
    def __init__(self):
        super().__init__('rrt_star_node')

        self.map_size = [100, 100]
        self.step_size = 1.0
        self.max_iter = 1000
        self.goal_region_radius = 4
        self.search_radius = 5
        self.path = None
        self.goal_reached = False
        self.vehicle_length = 0.5
        self.speed_conversion = np.array([[1, -self.vehicle_length / 2], [1, self.vehicle_length / 2]])

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.lidar_data = []

        self.start = NodeStruct(0.0, 0.0)
        self.goal = NodeStruct(10.0, 10.0)
        self.node_list = [self.start]

        self.create_subscription(Pose2D, '/robot_pose', self.gps_callback, 10)
        self.create_subscription(LaserScan, '/obstacle_distance', self.process_lidar_data, 10)

        self.robot_desired_speed = self.create_publisher(TwistStamped, '/bumperbot_controller/cmd_vel', 10)
        self.wheel_control_speed = self.create_publisher(Float64MultiArray, 'simple_velocity_controller/commands', 10)

        self.create_timer(0.1, self.control_loop)

    def get_steering_angle(self, vehicle_pos, target_location):
        dx = target_location[0] - vehicle_pos.location.x
        dy = target_location[1] - vehicle_pos.location.y
        target_angle = math.atan2(dy, dx)
        current_angle = math.radians(vehicle_pos.rotation.yaw)
        steering_angle = target_angle - current_angle
        steering_angle = max(-1.0, min(1.0, steering_angle / (math.pi/4)))
        self.current_angular_velocity = steering_angle
        return steering_angle

    def calculate_throttle(self, vehicle_pos, target_location):
        distance = np.linalg.norm([
            target_location[0] - vehicle_pos.location.x,
            target_location[1] - vehicle_pos.location.y])
        throttle = min(1.0, distance / (2*self.step_size))
        self.current_linear_velocity = throttle
        return throttle

    def gps_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
        self.get_logger().info(f"Robot pose: ({self.current_x:.2f}, {self.current_y:.2f})")




    def process_lidar_data(self, msg):
        # Placeholder - no obstacle processing for now
        pass




    def is_collision_free(self, node):
        return True  # No obstacle check yet

    def get_random_node(self):
        if random.random() > 0.2:
            return NodeStruct(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))
        else:
            return NodeStruct(self.goal.x, self.goal.y)

    def get_nearest_node(self, node_list, rand_node):
        distances = [np.linalg.norm([node.x - rand_node.x, node.y - rand_node.y]) for node in node_list]
        nearest_node_idx = np.argmin(distances)
        return node_list[nearest_node_idx]

    def steer(self, from_node, to_node):
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node = NodeStruct(from_node.x + self.step_size * math.cos(theta),
                              from_node.y + self.step_size * math.sin(theta))
        if not self.is_collision_free(new_node):
            return None
        new_node.cost = from_node.cost + self.step_size
        new_node.parent = from_node
        return new_node

    def reached_goal(self, node):
        return np.linalg.norm([node.x - self.goal.x, node.y - self.goal.y]) < self.goal_region_radius

    def generate_final_path(self, goal_node):
        path = []
        node = goal_node
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        path = path[::-1]
        return path

    def move_vehicle_along_path(self):
        if not self.path:
            return
        i = 0
        while i < len(self.path):
            target_location = self.path[i]
            vehicle_pos = DummyPose(self.current_x, self.current_y, math.degrees(self.current_theta))
            steering_angle = self.get_steering_angle(vehicle_pos, target_location)
            throttle = self.calculate_throttle(vehicle_pos, target_location)

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = "base_link"
            twist_msg.twist.linear.x = throttle
            twist_msg.twist.angular.z = steering_angle
            self.robot_desired_speed.publish(twist_msg)

            robot_speed = np.array([[throttle], [steering_angle]])
            wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed)

            wheel_speed_msg = Float64MultiArray()
            wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
            self.wheel_control_speed.publish(wheel_speed_msg)

            distance = np.linalg.norm([self.current_x - target_location[0], self.current_y - target_location[1]])
            if distance < self.step_size:
                i += 1

    def plan(self):
        self.get_logger().info("Planning...")
        self.node_list = [self.start]
        self.path = None
        self.goal_reached = False

        for _ in range(self.max_iter):
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, rand_node)
            new_node = self.steer(nearest_node, rand_node)
            if new_node is not None:
                self.node_list.append(new_node)
                if self.reached_goal(new_node):
                    self.path = self.generate_final_path(new_node)
                    if self.path:
                        self.goal_reached = True
                        self.move_vehicle_along_path()
                        return

    def control_loop(self):
        if not self.goal_reached:
            self.plan()


def main(args=None):
    rclpy.init(args=args)
    node = RRTStar()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
