#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import TwistStamped, Pose2D
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
from action_file.action import Target


global goal_received
goal_received = False


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller_node")

        # Parameters for wheel radius and separation
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius} meters")
        self.get_logger().info(f"Using wheel separation: {self.wheel_separation} meters")

        # Publishers
        self.wheel_control_speed = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.robot_desired_speed = self.create_publisher(TwistStamped, "/bumperbot_controller/cmd_vel", 10)

        # Subscriber to current position
        self.current_position_sub = self.create_subscription(Pose2D, "/robot_pose", self.current_position_callback, 10)

        # Action server
        self.action_server = ActionServer(self, Target, "reach_target", self.target_position_callback)

        # Initialize position variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0

        # Speed conversion matrix for differential kinematics
        self.speed_conversion = np.array(
            [
                [self.wheel_radius / 2.0, self.wheel_radius / 2.0],
                [
                    self.wheel_radius / self.wheel_separation,
                    -self.wheel_radius / self.wheel_separation,
                ],
            ]
        )

    def set_robot_velocity(self, linear_velocity: float, angular_velocity: float):
        # Publish to /bumperbot_controller/cmd_vel
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = linear_velocity
        twist_msg.twist.angular.z = angular_velocity
        self.robot_desired_speed.publish(twist_msg)

        # Compute wheel speeds using differential kinematics
        robot_speed = np.array([[linear_velocity], [angular_velocity]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed)

        # Publish to simple_velocity_controller/commands
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]  # [right_wheel, left_wheel]
        self.wheel_control_speed.publish(wheel_speed_msg)

    def current_position_callback(self, msg):
        # Update current position
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def get_current_position(self):
        # Return the current position as a tuple
        return self.current_x, self.current_y, self.current_theta

    def target_position_callback(self, goal_handle):
        # Store target position
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
        # Return the target position as a tuple
        return self.target_x, self.target_y, self.target_theta


def main(args=None):
    rclpy.init(args=args)

    # Create the robot controller node
    robot_controller_node = RobotControllerNode()

    while rclpy.ok():
        rclpy.spin_once(robot_controller_node)

        # Get current and target positions
        x_recent, y_recent, theta_recent = robot_controller_node.get_current_position()
        x_target, y_target, theta_target = robot_controller_node.get_target_position()

        robot_controller_node.get_logger().info(f"Current position: ({x_recent:.2f}, {y_recent:.2f}, {math.degrees(theta_recent):.2f})")
        robot_controller_node.get_logger().info(f"Target position: ({x_target}, {y_target}, {theta_target})")

        # Calculate errors
        dx = x_target - x_recent
        dy = y_target - y_recent
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - theta_recent

        

        if distance < 0.000001:  # Target reached
            robot_controller_node.set_robot_velocity(0.0, 0.0)
            robot_controller_node.get_logger().info("Target reached! Robot stopped.")
            continue

        # Control logic
        k_linear = 0.5
        k_angular = 1.0
        linear_velocity = min(k_linear * distance, 1.0)  # Cap linear velocity at 1 m/s
        angular_velocity = min(k_angular * angle_error, 1.0)  # Cap angular velocity at 1 rad/s

        # Set robot velocity
        robot_controller_node.set_robot_velocity(linear_velocity, angular_velocity)

    robot_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
