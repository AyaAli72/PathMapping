import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, Pose2D
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import random
import time
import threading
import matplotlib.pyplot as plt
import matplotlib
from collections import deque

# Use TkAgg backend for better compatibility
matplotlib.use('TkAgg')

class EnhancedPIDController:
    def __init__(self, kp, ki, kd, min_output, max_output, windup_limit=1.0, derivative_filter=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.windup_limit = windup_limit
        self.derivative_filter = derivative_filter
        self.reset()
        
    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_derivative = 0.0
        self.last_time = time.time()
        
    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            return 0.0
            
        # Apply derivative filter
        derivative = (error - self.previous_error) / dt
        filtered_derivative = (1 - self.derivative_filter) * self.previous_derivative + self.derivative_filter * derivative
        
        # Anti-windup for integral term
        if abs(self.integral) > self.windup_limit:
            self.integral *= 0.95  # Leaky integrator to prevent windup
            
        # PID terms
        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * filtered_derivative
        
        # Total output
        output = p + i + d
        
        # Clamp output to limits
        output = max(self.min_output, min(output, self.max_output))
        
        # Update state only if not saturated
        if abs(output) < 0.95 * self.max_output:
            self.integral += error * dt
        self.previous_error = error
        self.previous_derivative = filtered_derivative
        self.last_time = current_time
        
        return output

class RRTNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class OptimizedPathPlotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        
        # Initialize empty plots
        self.robot_path = self.ax.plot([], [], 'b-', linewidth=1.5, label='Robot Path')[0]
        self.planned_path = self.ax.plot([], [], 'g--', linewidth=1.0, label='Planned Path')[0]
        self.robot_position = self.ax.plot([], [], 'ro', markersize=8, label='Robot Position')[0]
        self.goal_position = self.ax.plot([], [], 'kx', markersize=10, label='Goal Position')[0]
        
        # Set plot parameters
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Robot Navigation Path')
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_aspect('equal', adjustable='datalim')
        
        # Initialize data with fixed-length buffers
        self.max_points = 500  # Limit number of points to reduce lag
        self.robot_x = deque(maxlen=self.max_points)
        self.robot_y = deque(maxlen=self.max_points)
        self.planned_x = deque(maxlen=self.max_points)
        self.planned_y = deque(maxlen=self.max_points)
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        
        # Track if we need to update view limits
        self.needs_limits_update = True
        
    def update_plot(self):
        # Update data efficiently
        self.robot_path.set_data(self.robot_x, self.robot_y)
        self.planned_path.set_data(self.planned_x, self.planned_y)
        self.robot_position.set_data([self.current_robot_x], [self.current_robot_y])
        self.goal_position.set_data([self.goal_x], [self.goal_y])
        
        # Update view limits only when necessary
        if self.needs_limits_update and self.robot_x:
            margin = 0.5
            x_min = min(min(self.robot_x), self.goal_x) - margin
            x_max = max(max(self.robot_x), self.goal_x) + margin
            y_min = min(min(self.robot_y), self.goal_y) - margin
            y_max = max(max(self.robot_y), self.goal_y) + margin
            
            self.ax.set_xlim(x_min, x_max)
            self.ax.set_ylim(y_min, y_max)
            self.needs_limits_update = False
        
        # Efficient canvas update
        self.fig.canvas.draw_idle()
        return True
    
    def add_robot_point(self, x, y):
        self.robot_x.append(x)
        self.robot_y.append(y)
        self.current_robot_x = x
        self.current_robot_y = y
        self.needs_limits_update = True
    
    def set_planned_path(self, path):
        self.planned_x.clear()
        self.planned_y.clear()
        for point in path:
            self.planned_x.append(point[0])
            self.planned_y.append(point[1])
        self.needs_limits_update = True
    
    def set_goal(self, x, y):
        self.goal_x = x
        self.goal_y = y
        self.needs_limits_update = True

class EnhancedRRTNavigator(Node):
    def __init__(self, vehicle_length, map_size):
        super().__init__('enhanced_rrt_navigator')

        self.goal = None
        self.map_size = map_size
        self.step_size = 0.4
        self.max_iter = 4000
        self.goal_region_radius = 0.15
        self.waypoint_tolerance = 0.15

        self.safety_margin = 0.6
        self.backup_distance = 1.2
        self.lateral_escape_distance = 1.5
        self.max_backup_attempts = 2
        self.evasion_retry_limit = 3
        self.turn_around_attempts_limit = 2
        self.turn_around_trigger_limit = 5
        self.obstacle_cycles = 0

        self.vehicle_length = vehicle_length
        self.current_pose = (0.0, 0.0, 0.0)
        self.path = []
        self.path_index = 0
        self.goal_reached = False
        self.obstacle_history = deque(maxlen=10)  # Fixed-size buffer
        self.current_clearance = {'front': float('inf'), 'left': float('inf'), 'right': float('inf'), 'rear': float('inf')}
        self.start = None
        self.evasion_attempts = 0
        self.turn_around_attempts = 0
        self.new_goal_requested = False

        # Enhanced PID controllers with anti-windup and filtering
        self.angular_pid = EnhancedPIDController(kp=1.5, ki=0.02, kd=0.3, 
                                               min_output=-1.5, max_output=1.5,
                                               windup_limit=0.5, derivative_filter=0.2)
        self.linear_pid = EnhancedPIDController(kp=1.0, ki=0.01, kd=0.15, 
                                              min_output=-0.5, max_output=0.5,
                                              windup_limit=0.3, derivative_filter=0.15)
        self.precision_angular_pid = EnhancedPIDController(kp=2.0, ki=0.02, kd=0.4, 
                                                         min_output=-0.8, max_output=0.8,
                                                         windup_limit=0.3, derivative_filter=0.1)
        self.precision_linear_pid = EnhancedPIDController(kp=1.2, ki=0.015, kd=0.2, 
                                                        min_output=-0.2, max_output=0.2,
                                                        windup_limit=0.2, derivative_filter=0.1)

        self.normal_speed = 0.4
        self.precision_speed = 0.15
        self.rotation_gain = 0.8
        self.backup_speed = -0.25

        self.wheel_control_pub = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/bumperbot_controller/cmd_vel", 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Pose2D, "/robot_pose", self.pose_callback, 10)
        self.nav_timer = self.create_timer(0.1, self.navigation_cycle)

        self.wheel_radius = 0.033
        self.wheel_separation = 0.16
        self.speed_matrix = np.array([
            [self.wheel_radius/2, self.wheel_radius/2],
            [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]
        ])
        
        # Create optimized plotter with efficient updates
        plt.ion()
        self.plotter = OptimizedPathPlotter()
        self.plot_update_timer = self.create_timer(0.5, self.plot_update_callback)
        
        self.get_logger().info("Enhanced RRT Navigator Ready. Waiting for initial pose...")

    def plot_update_callback(self):
        try:
            # Update plot at reduced frequency
            self.plotter.update_plot()
            plt.pause(0.001)
        except Exception as e:
            self.get_logger().error(f"Plot update failed: {str(e)}")

    def pose_callback(self, msg):
        self.current_pose = (msg.x, msg.y, msg.theta)
        # Add point to plotter
        self.plotter.add_robot_point(msg.x, msg.y)
        
        if self.start is None:
            self.start = RRTNode(msg.x, msg.y)
            self.get_logger().info("Initial position set. Waiting for goal...")

    def lidar_callback(self, msg):
        self.environment_analysis(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max)

    def set_goal(self, goal):
        self.goal = RRTNode(goal[0], goal[1])
        self.goal_reached = False
        self.path = []
        self.path_index = 0
        self.obstacle_cycles = 0
        self.evasion_attempts = 0
        self.turn_around_attempts = 0
        self.get_logger().info(f"New goal set: ({goal[0]:.2f}, {goal[1]:.2f})")
        self.plotter.set_goal(goal[0], goal[1])
        
        # Reset PID controllers
        self.angular_pid.reset()
        self.linear_pid.reset()
        self.precision_angular_pid.reset()
        self.precision_linear_pid.reset()
        
        if self.start is None:
            self.get_logger().warn("Waiting for current position before planning...")
        else:
            self.plan_path()

    def environment_analysis(self, ranges, angle_min, angle_inc, max_range):
        sectors = {
            'front': (-math.pi/6, math.pi/6), 
            'left': (math.pi/6, math.pi/2), 
            'right': (-math.pi/2, -math.pi/6), 
            'rear': (math.pi/2, 3*math.pi/4)
        }
        clearance = {}
        for sector, (start, end) in sectors.items():
            # Use median instead of mean to reduce noise impact
            sector_ranges = [r for i, r in enumerate(ranges) 
                            if start <= (angle_min + i*angle_inc) % (2*math.pi) <= end 
                            and not math.isinf(r)]
            clearance[sector] = np.median(sector_ranges) if sector_ranges else max_range
        self.current_clearance = clearance
        self.obstacle_history.append(clearance)

    def navigation_cycle(self):
        if self.goal is None:
            return
            
        if self.goal_reached:
            self.stop_motion()
            self.prompt_user_options()
            return
            
        if self.near_goal():
            self.precision_approach()
            return
            
        if self.obstacle_detected():
            self.obstacle_cycles += 1
            self.obstacle_response()
        else:
            self.reset_obstacle_state()
            self.path_following()

    def prompt_user_options(self):
        if not self.new_goal_requested:
            self.new_goal_requested = True
            self.get_logger().info("\n\nGoal reached! Enter new goal coordinates:")

    def reset_obstacle_state(self):
        self.evasion_attempts = 0
        self.turn_around_attempts = 0
        self.obstacle_cycles = 0
        if hasattr(self, 'tried_quarter_turn'):
            del self.tried_quarter_turn

    def obstacle_response(self):
        if len(self.obstacle_history) < 3:
            self.backup_maneuver()
        elif not hasattr(self, 'tried_quarter_turn'):
            turn_dir = 'left' if self.current_clearance['left'] > self.current_clearance['right'] else 'right'
            self.get_logger().info(f"Trying 90-degree {turn_dir} turn before considering full turn...")
            angle_offset = math.pi / 2 if turn_dir == 'left' else -math.pi / 2
            self.tried_quarter_turn = True
            self.rotate_to_angle((self.current_pose[2] + angle_offset) % (2 * math.pi))
        elif (self.turn_around_attempts < self.turn_around_attempts_limit and 
              self.obstacle_cycles >= self.turn_around_trigger_limit):
            self.get_logger().info("Obstacle persists. Trying 180-degree turn...")
            self.rotate_to_angle((self.current_pose[2] + math.pi) % (2 * math.pi))
            self.turn_around_attempts += 1
            self.obstacle_cycles = 0
            self.plan_path()
        else:
            self.lateral_evasion()

    def near_goal(self):
        return math.hypot(self.current_pose[0]-self.goal.x, self.current_pose[1]-self.goal.y) < 1.0

    def precision_approach(self):
        dx = self.goal.x - self.current_pose[0]
        dy = self.goal.y - self.current_pose[1]
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.angle_difference(target_angle)
        
        if distance < self.goal_region_radius:
            self.goal_reached = True
            self.stop_motion()
            return
            
        # Use PID controllers for smooth approach
        angular_velocity = self.precision_angular_pid.compute(angle_error)
        linear_velocity = self.precision_linear_pid.compute(distance)
        
        # Apply dynamic speed limits based on distance
        max_linear = min(self.precision_speed, distance * 0.5)
        linear_velocity = max(min(linear_velocity, max_linear), -max_linear)
        
        self.set_velocity(linear_velocity, angular_velocity)

    def obstacle_detected(self):
        # Add hysteresis to prevent oscillation
        return self.current_clearance['front'] < (self.vehicle_length + self.safety_margin * 1.2)

    def backup_maneuver(self):
        if not hasattr(self, 'backup_start'):
            self.backup_start = time.time()
            self.get_logger().warn("Initiating backup maneuver...")
        elapsed = time.time() - self.backup_start
        backup_duration = self.backup_distance / abs(self.backup_speed)
        if elapsed < backup_duration:
            self.set_velocity(self.backup_speed, 0)
        else:
            del self.backup_start
            self.plan_path()

    def lateral_evasion(self):
        if not hasattr(self, 'escape_direction'):
            self.escape_direction = self.choose_evasion_direction()
            self.escape_target = self.calculate_escape_vector()
            self.get_logger().info(f"Executing lateral evasion to {self.escape_direction}")
            self.evasion_attempts = 0
        self.evasion_attempts += 1
        if self.evasion_attempts > self.evasion_retry_limit:
            self.get_logger().warn("Evasion attempts exceeded. Switching to backup maneuver.")
            del self.escape_direction
            self.backup_maneuver()
            return
        dx = self.escape_target[0] - self.current_pose[0]
        dy = self.escape_target[1] - self.current_pose[1]
        distance = math.hypot(dx, dy)
        if distance < 0.3:
            del self.escape_direction
            self.plan_path()
            return
        target_angle = math.atan2(dy, dx)
        angle_error = self.angle_difference(target_angle)
        
        # Use PID controllers for evasion with reduced gain
        angular_velocity = self.angular_pid.compute(angle_error) * 0.7
        linear_velocity = self.linear_pid.compute(distance) * 0.5
        
        self.set_velocity(linear_velocity, angular_velocity)

    def choose_evasion_direction(self):
        # Use weighted average of recent clearances
        weights = [0.5, 0.3, 0.2]  # More weight to recent measurements
        left_clear = 0
        right_clear = 0
        for i, clearance in enumerate(reversed(self.obstacle_history)):
            if i >= len(weights):
                break
            left_clear += clearance['left'] * weights[i]
            right_clear += clearance['right'] * weights[i]
        return 'left' if left_clear >= right_clear else 'right'

    def calculate_escape_vector(self):
        angle = math.pi/3 if self.escape_direction == 'left' else -math.pi/3
        return (
            self.current_pose[0] + self.lateral_escape_distance * math.cos(angle),
            self.current_pose[1] + self.lateral_escape_distance * math.sin(angle)
        )

    def path_following(self):
        if not self.path or self.path_index >= len(self.path):
            self.plan_path()
            return
            
        target = self.path[self.path_index]
        dx = target[0] - self.current_pose[0]
        dy = target[1] - self.current_pose[1]
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.angle_difference(target_angle)
        
        if distance < self.waypoint_tolerance:
            self.path_index += 1
            # Reset PID controllers when moving to new waypoint
            self.angular_pid.reset()
            self.linear_pid.reset()
            return
            
        # Dynamic speed adjustment based on angle error
        speed_factor = max(0.3, 1.0 - min(1.0, abs(angle_error) / (math.pi/4)))
        
        # Use PID controllers for smooth movement
        angular_velocity = self.angular_pid.compute(angle_error)
        linear_velocity = self.linear_pid.compute(distance) * speed_factor
        
        # Limit velocity to normal speed
        linear_velocity = min(linear_velocity, self.normal_speed * speed_factor)
        linear_velocity = max(linear_velocity, -self.normal_speed * speed_factor)
        
        self.set_velocity(linear_velocity, angular_velocity)

    def plan_path(self):
        if self.start is None or self.goal is None:
            return
            
        self.node_list = [self.start]
        path_found = False
        for _ in range(self.max_iter):
            rand = self.sample_point()
            nearest = self.find_nearest(rand)
            new_node = self.extend_tree(nearest, rand)
            if self.node_valid(new_node):
                new_node.parent = nearest
                self.node_list.append(new_node)
                if self.reached_target(new_node):
                    self.path = self.build_path(new_node)
                    self.path_index = 0
                    path_found = True
                    self.get_logger().info("Path found with {} waypoints".format(len(self.path)))
                    # Update plot with planned path
                    self.plotter.set_planned_path(self.path)
                    break
        if not path_found:
            self.get_logger().warn("Path planning unsuccessful, retrying...")
            time.sleep(0.5)
            self.plan_path()

    def find_nearest(self, target_node):
        return min(self.node_list, key=lambda n: math.hypot(n.x - target_node.x, n.y - target_node.y))

    def sample_point(self):
        if random.random() < 0.4:
            return self.goal
        return RRTNode(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))

    def extend_tree(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.hypot(dx, dy)
        step = min(distance, self.step_size)
        theta = math.atan2(dy, dx)
        return RRTNode(from_node.x + step * math.cos(theta), from_node.y + step * math.sin(theta))

    def node_valid(self, node):
        if not node:
            return False
        front_clear = self.current_clearance.get('front', 0) > self.vehicle_length + 0.5
        lateral_clear = True
        if hasattr(self, 'escape_direction'):
            lateral_clear = self.current_clearance.get(self.escape_direction, 0) > 0.7
        return front_clear and lateral_clear

    def reached_target(self, node):
        return math.hypot(node.x - self.goal.x, node.y - self.goal.y) <= self.goal_region_radius

    def build_path(self, node):
        path = []
        while node:
            path.append([node.x, node.y])
            node = node.parent
        return path[::-1]

    def angle_difference(self, target):
        # Normalize angle difference to [-π, π]
        diff = (target - self.current_pose[2] + math.pi) % (2 * math.pi) - math.pi
        # Handle wrap-around cases
        if diff < -math.pi:
            diff += 2 * math.pi
        elif diff > math.pi:
            diff -= 2 * math.pi
        return diff

    def set_velocity(self, linear, angular):
        # Apply low-pass filter to smooth commands
        if not hasattr(self, 'prev_linear'):
            self.prev_linear = linear
            self.prev_angular = angular
            
        # Smoothing factor (0.2 = 80% of previous value, 20% of new value)
        linear = 0.8 * self.prev_linear + 0.2 * linear
        angular = 0.8 * self.prev_angular + 0.2 * angular
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(linear)
        cmd.twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)
        
        # Convert to wheel velocities
        wheel_speed = np.linalg.inv(self.speed_matrix) @ np.array([[linear], [angular]])
        msg = Float64MultiArray()
        msg.data = [float(wheel_speed[1, 0]), float(wheel_speed[0, 0])]
        self.wheel_control_pub.publish(msg)
        
        # Store for next iteration
        self.prev_linear = linear
        self.prev_angular = angular

    def stop_motion(self):
        self.set_velocity(0.0, 0.0)

    def rotate_to_angle(self, target_angle):
        angle_error = self.angle_difference(target_angle)
        angular_velocity = self.angular_pid.compute(angle_error)
        self.set_velocity(0.0, angular_velocity)
        
    def destroy_node(self):
        plt.close(self.plotter.fig)
        super().destroy_node()

def user_interface(node):
    while rclpy.ok():
        try:
            # Wait for initial pose
            while rclpy.ok() and node.start is None:
                time.sleep(0.1)
                
            # Get initial goal
            if node.goal is None:
                print("\nEnter initial goal coordinates:")
                x = float(input("X: "))
                y = float(input("Y: "))
                node.set_goal([x, y])
            
            # Handle goal reached options
            if node.new_goal_requested:
                print("\nEnter new goal coordinates:")
                x = float(input("X: "))
                y = float(input("Y: "))
                node.set_goal([x, y])
                node.new_goal_requested = False
                    
            time.sleep(0.1)
        except Exception as e:
            node.get_logger().error(f"User interface error: {str(e)}")
            time.sleep(0.5)

def main():
    rclpy.init()
    navigator = EnhancedRRTNavigator(vehicle_length=0.235, map_size=[10, 10])
    
    # Start user interface in a separate thread
    ui_thread = threading.Thread(target=user_interface, args=(navigator,))
    ui_thread.daemon = True
    ui_thread.start()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()
        ui_thread.join()

if __name__ == '__main__':
    main()
