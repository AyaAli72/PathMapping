import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import TwistStamped, Pose2D
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import csv
from collections import deque
from action_file.action import Target

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller_node")
        
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_separation = self.get_parameter("wheel_separation").value

        self.get_logger().info(f"Wheel radius: {self.wheel_radius} m, Wheel separation: {self.wheel_separation} m")

        self.vel_publisher = self.create_publisher(TwistStamped, "/bumperbot_controller/cmd_vel", 10)
        self.pose_subscriber = self.create_subscription(Pose2D, "/robot_pose", self.current_position_callback, 10)
        self.action_server = ActionServer(self, Target, "reach_target", self.target_position_callback)

        self.goal_queue = deque()
        self.current_x, self.current_y, self.current_theta = 0.0, 0.0, 0.0
        self.prev_distance_error = 0.0
        self.prev_angle_error = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.log_file = open("pd_tuning_log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["time", "distance_error", "angle_error"])

    def set_robot_velocity(self, linear_velocity: float, angular_velocity: float):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = linear_velocity
        twist_msg.twist.angular.z = angular_velocity
        self.vel_publisher.publish(twist_msg)

    def current_position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def get_current_position(self):
        return self.current_x, self.current_y, self.current_theta

    def target_position_callback(self, goal_handle):
        self.get_logger().info("Starting goal processing")
        goal = goal_handle.request.target_point
        self.target_x = goal.x
        self.target_y = goal.y
        self.target_theta = goal.z

        result_msg = Target.Result()
        result_msg.success = True
        goal_handle.succeed()
        return result_msg

    def get_target_position(self):
        return self.target_x, self.target_y, self.target_theta

    def control_loop(self):
        if not self.goal_queue:
            return

        target_x, target_y, target_theta = self.goal_queue[0]
        dx, dy = target_x - self.current_x, target_y - self.current_y
        distance_error = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = (target_angle - self.current_theta + math.pi) % (2 * math.pi) - math.pi

        if distance_error < 0.05:
            self.goal_queue.popleft()
            self.set_robot_velocity(0.0, 0.0)
            self.get_logger().info("Target reached. Moving to next goal.")
            return

        Kp_linear, Kd_linear = 1.2, 0.5
        Kp_angular, Kd_angular = 1.0, 0.3

        linear_velocity = Kp_linear * distance_error + Kd_linear * (distance_error - self.prev_distance_error)
        angular_velocity = Kp_angular * angle_error + Kd_angular * (angle_error - self.prev_angle_error)

        linear_velocity = max(min(linear_velocity, 1.0), -1.0)
        angular_velocity = max(min(angular_velocity, 1.0), -1.0)

        self.set_robot_velocity(linear_velocity, angular_velocity)

        self.prev_distance_error = distance_error
        self.prev_angle_error = angle_error

        current_time = self.get_clock().now().nanoseconds * 1e-9
        self.csv_writer.writerow([current_time, distance_error, angle_error])


def main(args=None):
    rclpy.init(args=args)
    robot_controller_node = RobotControllerNode()

    while rclpy.ok():
        rclpy.spin_once(robot_controller_node)

        x_recent, y_recent, theta_recent = robot_controller_node.get_current_position()
        try:
            x_target, y_target, theta_target = robot_controller_node.get_target_position()
        except AttributeError:
            continue

        robot_controller_node.get_logger().info(
            f"Current: ({x_recent:.2f}, {y_recent:.2f}, {math.degrees(theta_recent):.2f}), "
            f"Target: ({x_target}, {y_target}, {theta_target})"
        )

        dx = x_target - x_recent
        dy = y_target - y_recent
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - theta_recent

        if distance < 0.000001:
            robot_controller_node.set_robot_velocity(0.0, 0.0)
            robot_controller_node.get_logger().info("Target reached! Robot stopped.")
            continue

        k_linear = 0.5
        k_angular = 1.0
        linear_velocity = min(k_linear * distance, 1.0)
        angular_velocity = min(k_angular * angle_error, 1.0)

        robot_controller_node.set_robot_velocity(linear_velocity, angular_velocity)

    robot_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
