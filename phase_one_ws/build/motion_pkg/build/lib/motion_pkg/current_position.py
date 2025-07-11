#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
import tf_transformations
import math
import tkinter as tk
from threading import Thread

class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_pose_publisher')
        
        # Subscribe to BOTH controller odometry AND Gazebo ground truth
        self.odom_sub = self.create_subscription(
            Odometry,
            '/bumperbot_controller/odom',
            self.odom_callback,
            10
        )
        
        self.ground_truth_sub = self.create_subscription(
            Odometry,
            '/ground_truth_odom',  # Gazebo's ground truth topic
            self.ground_truth_callback,
            10
        )
        
        # Publishers
        self.pose_publisher = self.create_publisher(Pose2D, '/robot_pose', 10)
        self.marker_publisher = self.create_publisher(Marker, '/robot_position_marker', 10)
        
        # Initialize variables
        self.current_pose = Pose2D()
        self.marker_id = 0
        self.last_update_time = self.get_clock().now()
        
        # Start GUI
        self.gui_thread = Thread(target=self.start_gui, daemon=True)
        self.gui_thread.start()
        
        self.get_logger().info("Robot Pose Publisher initialized with ground truth tracking")

    def odom_callback(self, msg):
        self.process_pose(msg.pose.pose, "ODOMETRY")

    def ground_truth_callback(self, msg):
        self.process_pose(msg.pose.pose, "GROUND TRUTH")

    def process_pose(self, pose, source):
        # Extract position
        x = pose.position.x
        y = pose.position.y
        
        # Extract orientation
        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]
        euler = tf_transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        
        # Update current pose
        self.current_pose.x = x
        self.current_pose.y = y
        self.current_pose.theta = yaw
        
        # Publish updates
        self.pose_publisher.publish(self.current_pose)
        self.publish_marker(x, y, yaw, source)
        
        # Throttle console output to 2Hz
        now = self.get_clock().now()
        if (now - self.last_update_time).nanoseconds > 500000000:
            self.print_console_output(x, y, yaw, source)
            self.last_update_time = now

    def print_console_output(self, x, y, yaw, source):
        angle = int((math.degrees(yaw) + 45) // 90) % 4
        directions = ['↑ North', '→ East', '↓ South', '← West']
        
        output = f"""
        ┌───────────────────────────────────────┐
        │  {source:^35} │
        │  X: {x:8.3f} m                           │
        │  Y: {y:8.3f} m                           │
        │  θ: {math.degrees(yaw):8.1f}° {directions[angle]:<9}│
        └───────────────────────────────────────┘
        """
        self.get_logger().info(output)

    def publish_marker(self, x, y, yaw, source):
        marker = Marker()
        marker.header.frame_id = "odom" if source == "ODOMETRY" else "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = source.lower().replace(" ", "_")
        marker.id = self.marker_id
        self.marker_id = (self.marker_id + 1) % 1000
        
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position and orientation
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        
        # Scale and color
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        if source == "ODOMETRY":
            marker.color.r = 1.0  # Red for odometry
        else:
            marker.color.g = 1.0  # Green for ground truth
            
        marker.color.a = 1.0
        marker.lifetime.sec = 1
        
        self.marker_publisher.publish(marker)

    def start_gui(self):
        root = tk.Tk()
        root.title("Robot Pose Display")
        root.geometry("350x250")
        
        # Create and pack widgets
        tk.Label(root, text="Robot Pose Feedback", font=('Arial', 16)).pack(pady=10)
        
        source_frame = tk.Frame(root)
        source_frame.pack()
        self.source_label = tk.Label(source_frame, text="Source: N/A", font=('Arial', 12))
        self.source_label.pack()
        
        pose_frame = tk.Frame(root)
        pose_frame.pack(pady=10)
        
        self.x_label = tk.Label(pose_frame, text="X: 0.000 m", font=('Arial', 14))
        self.x_label.pack()
        self.y_label = tk.Label(pose_frame, text="Y: 0.000 m", font=('Arial', 14))
        self.y_label.pack()
        self.theta_label = tk.Label(pose_frame, text="θ: 0.0° North", font=('Arial', 14))
        self.theta_label.pack()
        
        def update_gui():
            directions = ['North', 'East', 'South', 'West']
            angle = int((math.degrees(self.current_pose.theta) + 45) // 90) % 4
            
            self.x_label.config(text=f"X: {self.current_pose.x:.3f} m")
            self.y_label.config(text=f"Y: {self.current_pose.y:.3f} m")
            self.theta_label.config(
                text=f"θ: {math.degrees(self.current_pose.theta):.1f}° {directions[angle]}"
            )
            root.after(100, update_gui)
        
        update_gui()
        root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()