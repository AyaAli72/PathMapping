import numpy as np
import random
import math
import time
import matplotlib.pyplot as plt
from collections import deque
import csv

# ROS Libraries 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, NavSatFix
from geometry_msgs.msg import TwistStamped, Pose2D
from std_msgs.msg import Float64MultiArray
from action_file.action import Target
from geometry_msgs.msg import Pose, Twist
import sensor_msgs_py.point_cloud2 as pc2
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor


###################################################### Initial functions ############################################################################
def get_steering_angle(self, vehicle_pose: Pose2D, target_location):
    dx = target_location[0] - vehicle_pose.x
    dy = target_location[1] - vehicle_pose.y
    target_angle = math.atan2(dy, dx)
    current_angle = vehicle_pose.theta
    steering_angle = target_angle - current_angle

    # Normalize steering angle to [-pi, pi]
    while steering_angle > math.pi:
        steering_angle -= 2 * math.pi
    while steering_angle < -math.pi:
        steering_angle += 2 * math.pi

    # Normalize to [-1.0, 1.0] for typical actuator input scaling
    max_steering = math.pi / 4  # 45 degrees max steering
    normalized_steering = max(-1.0, min(1.0, steering_angle / max_steering))
    return normalized_steering


def calculate_throttle(self, vehicle_pose: Pose2D, target_location):
    distance = np.linalg.norm([
        target_location[0] - vehicle_pose.x,
        target_location[1] - vehicle_pose.y])

    # Smooth throttle control logic
    if distance < 1.0:
        return distance / 2.0  # Slow down near goal
    else:
        return min(1.0, 0.5 + (distance / (2 * self.step_size)))
#######################################################################################################################################################


class RRTNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


class RRTStar(Node):
    def __init__(self, vehicle_length, start, goal, map_size, step_size=1.0, max_iter=1000):
        self.start = RRTNode(start[0], start[1])
        self.goal = RRTNode(goal[0], goal[1])
        self.map_size = map_size
        self.step_size = step_size
        self.max_iter = max_iter
        self.node_list = [self.start]
        self.goal_region_radius = 4
        self.search_radius = 5
        self.path = None
        self.goal_reached = False
        self.lidar_data = []
        self.vehicle_length = vehicle_length

        self.current_x, self.current_y, self.current_theta = 0.0, 0.0, 0.0
        self.lidar_data = deque(maxlen=1000)
        self.lidar_data = []

        self.vel_publisher = self.create_publisher(Twist, "/bumperbot_controller/cmd_vel", 10)
        self.lidar_subscriber = self.create_subscription(PointCloud2, '/lidar_points', self.process_lidar_data, 10)
        self.gps_subscriber = self.create_subscription(Pose2D, "/robot_pose", self.gps_callback, 10)

############################################################## Sensors Readings CallBacks#################################################################

#LIDAR Callback
#-----------------
    def process_lidar_data(self, msg):
        point_cloud_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.lidar_data = list(point_cloud_data)
        print(f"LIDAR data points: {len(self.lidar_data)}")  # Print the number of points
        return self.lidar_data

#GPS Callback
#----------------- 
    def gps_callback(self, msg):
        self.start_location = [msg.x, msg.y]
        self.current_latitude = msg.x
        self.current_longitude = msg.y
        self.current_theta = msg.theta
#######################################################################################################################################################




############################################################## Path Planning Logic #################################################################  
    def get_nearest_node(self, node_list, rand_node):
        distances = [np.linalg.norm([node.x - rand_node.x, node.y - rand_node.y]) for node in node_list]
        nearest_node_idx = np.argmin(distances)
        return node_list[nearest_node_idx]

    def is_collision_free(self, node):
        if len(self.lidar_data) == 0:
            return True  # Assume no obstacles if no LIDAR data is available

        safety_margin = 2.0
        for point in self.lidar_data:
            obstacle_distance = math.sqrt((point[0] - node.x) ** 2 + (point[1] - node.y) ** 2)
            if obstacle_distance < self.vehicle_length + safety_margin:
                print(f"Collision detected at ({node.x}, {node.y}) with obstacle at ({point[0]}, {point[1]})")
                return False
        return True

    def get_random_node(self):
        if random.random() > 0.2:
            return RRTNode(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))
        else:
            return RRTNode(self.goal.x, self.goal.y)

    def steer(self, from_node, to_node):
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node = RRTNode(from_node.x + self.step_size * math.cos(theta), from_node.y + self.step_size * math.sin(theta))
        if new_node.x < 0 or new_node.x > self.map_size[0] or new_node.y < 0 or new_node.y > self.map_size[1]:
            return None
        if not self.is_collision_free(new_node):
            return None
        new_node.cost = from_node.cost + self.step_size
        new_node.parent = from_node
        return new_node

    def generate_final_path(self, goal_node):
        path = []
        node = goal_node
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        return path[::-1]

    def plan(self):
        self.node_list = [self.start]
        self.path = None
        self.goal_reached = False
        for i in range(self.max_iter):
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, rand_node)
            new_node = self.steer(nearest_node, rand_node)
            if new_node is not None:
                self.node_list.append(new_node)
                if np.linalg.norm([new_node.x - self.goal.x, new_node.y - self.goal.y]) < self.goal_region_radius:
                    self.path = self.generate_final_path(new_node)
                    if self.path:
                        self.goal_reached = True
                    return


################################################################ Gazebo Sensor Node ###################################################################

'''
class GazeboSensorNode(Node):
    def __init__(self):
        super().__init__('gazebo_sensor_node')

        self.broadcaster = TransformBroadcaster(self)
        self.lidar_pub = self.create_publisher(LaserScan, 'scan', QoSProfile(depth=10))
        self.gps_pub = self.create_publisher(NavSatFix, 'gps', QoSProfile(depth=10))
        self.spawn_service_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.model_state_pub = self.create_publisher(ModelState, '/gazebo/set_model_state', QoSProfile(depth=10))

        self.spawn_vehicle()

    def spawn_vehicle(self):
        urdf_file = '/home/mido/Music/phase_one_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro'  # <--- Updated this path
        entity_name = "bumperbot"
        self.get_logger().info(f"Spawning bumperbot from {urdf_file} in Gazebo.")
        
        spawn_request = SpawnEntity.Request()
        spawn_request.name = entity_name
        spawn_request.xml = urdf_file

        while not self.spawn_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn_entity service...")

        future = self.spawn_service_client.call_async(spawn_request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Vehicle spawned: {response.success}")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn vehicle: {e}")
'''


################################################################ Main ################################################################################

def main():
    rclpy.init()

    goal_location = [10.0, 10.0]  # Example goal location
    vehicle_length = 0.0951
    map_size = [100, 100]  # Example map size

    # Create the RRT* node
    rrt_star_node = RRTStar(
        vehicle_length=vehicle_length,
        goal=goal_location,
        map_size=map_size
    )

    executor = MultiThreadedExecutor()
    executor.add_node(rrt_star_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down both nodes.")
    finally:
        rrt_star_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
