import math
import random
import time
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import PointCloud2,LaserScan
from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import struct
import threading

def get_steering_angle(vehicle_location, vechile_orientation, target_location):
    dx = target_location[0] - vehicle_location.x
    dy = target_location[1] - vehicle_location.y
    angle_to_target = math.atan2(dy, dx)  # Angle to target in radians
    # Current vehicle yaw in radians
 
    quaternion = (vechile_orientation.x, vechile_orientation.y, vechile_orientation.z, vechile_orientation.w)
        
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    _, _, yaw = euler_from_quaternion(quaternion)
    current_yaw = math.radians(yaw)
    # Steering angle is the difference between target angle and current yaw
    steering_angle = angle_to_target - current_yaw
    # Normalize steering angle to [-π, π]
    steering_angle = (steering_angle + math.pi) % (2 * math.pi) - math.pi
    # Scale the steering angle for smoother turns
    steering = steering_angle * 0.1  # Reduce the steering sensitivity

    return steering

def calculate_throttle(vehicle_location, target_location):
    dx = target_location[0] - vehicle_location.x
    dy = target_location[1] - vehicle_location.y
    distance_to_target = math.sqrt(dx**2 + dy**2)  # Euclidean distance
    # Always apply some throttle if the vehicle is far from the goal
    if distance_to_target > 0.05:
        return 0.5  # Fixed throttle value for testing
    else:
        return 0.0  # Stop if close to the goal
   
# Node class representing a state in the space
class myNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

class PathPlanning(Node):
    def __init__(self):
        super().__init__('path_planning')

        self.vel_publisher = self.create_publisher(TwistStamped, "/bumperbot_controller/cmd_vel", 10)
        rrt_star = RRTStar(
            vehicle_length=0.5,   #1 till we know length
            map_size=[100, 100],
            vel_publisher=self.vel_publisher,
        )  
        self.goal_point_subscriber = self.create_subscription(Point,'/goalpoint',rrt_star.set_goal_point,10)
        # self.lidar_subscriber = self.create_subscription(PointCloud2,'/lidar/points',rrt_star.process_lidar_data,10)
        self.lidar_subscriber = self.create_subscription(LaserScan,'scan',rrt_star.process_lidar_data,10)
        # self.odom_subscriber = self.create_subscription(Odometry, "/odometry/filtered", rrt_star.set_current_pos, 10)
        self.odom_subscriber = self.create_subscription(Odometry, "/bumperbot_controller/odom", rrt_star.set_current_pos, 10)

        # Start the read loop in a separate thread
        self.read_thread = threading.Thread(target=rrt_star.do_plan)
        self.read_thread.start()

# RRT* algorithm
class RRTStar:
    def __init__(self, vehicle_length, map_size, vel_publisher, step_size=0.2, max_iter=900):
        self.start = myNode(0,0)
        self.goal = myNode(0,0)
        self.map_size = map_size
        self.step_size = step_size
        self.max_iter = max_iter
        self.node_list = [self.start]
        self.goal_region_radius = 0.2
        self.search_radius = 2
        self.path = None
        self.goal_reached = False
        self.lidar_data = []
        self.vehicle_length = vehicle_length
        self.vel_publisher = vel_publisher
        self.curr_pos = Point(x=0.0, y=0.0, z=0.0)
        self.curr_orientation = None
        self.condition = threading.Condition()
                                                                                                                                          
    def set_current_pos(self, msg:Odometry):
        self.curr_pos = msg.pose.pose.position
        self.curr_orientation = msg.pose.pose.orientation
        # print(f"vechile pos ya waddd: {self.curr_pos}")
        # print(f"Current Position -> x: {self.curr_pos.x}, y: {self.curr_pos.y}, z: {self.curr_pos.z}")

    def set_goal_point(self, goal):
        self.goal = myNode(goal.x, goal.y)
        with self.condition:
            print("I notified you")
            self.condition.notify_all()
        # self.plan()

    def do_plan(self):
        while True:
            with self.condition:
                self.condition.wait()
                print("now I can Plan")
                self.plan()

    def process_lidar_data(self, msg):
        angle_min = msg.angle_min  
        angle_increment = msg.angle_increment  
        
        self.lidar_data = []  # Reset data to avoid accumulation

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r <= 0 or r > msg.range_max:  
                continue  # Ignore invalid readings

            theta = angle_min + i * angle_increment  # Compute angle in radians
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            
            self.lidar_data.append((x, y))  # Store (x, y) coordinates

    
    # def gps_callback(self, data):
    #     latitude = data.latitude
    #     longitude = data.longitude
    #     altitude = data.altitude
    #     print("Car Location:", self.vehicle.get_transform().location)  # Debug: Print car location
    #     print(f"GPS Data: Lat={latitude}, Lon={longitude}, Alt={altitude}")
          
    def get_nearest_node(self, node_list, rand_node):
        distances = [np.linalg.norm([node.x - rand_node.x, node.y - rand_node.y]) for node in node_list]
        nearest_node_idx = np.argmin(distances)
        return node_list[nearest_node_idx]
    
    def is_collision_free(self, node):
        # return True
        if len(self.lidar_data) == 0:
            return True  # إذا لم تكن هناك بيانات ليدار، افترض أنه لا يوجد عائق
        
        safety_margin = 0.5  # مسافة أمان حول السيارة
        for point in self.lidar_data:
            obstacle_distance = math.sqrt((point[0] - node.x)**2 + (point[1] - node.y)**2)
            
            # إذا كانت المسافة إلى العائق أقل من المسافة الآمنة، فهناك عائق
            if obstacle_distance < self.vehicle_length + safety_margin:
                print(f"Obstacle detected at ({point[0]}, {point[1]}) with distance {obstacle_distance}")
                return False
            
        return True
    
    def get_random_node(self):
        if random.random() > 0.2:
            return myNode(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))
        else:
            return myNode(self.goal.x, self.goal.y)
    
    def choose_parent(self, neighbors, nearest_node, new_node):
        min_cost = nearest_node.cost + np.linalg.norm([new_node.x - nearest_node.x, new_node.y - nearest_node.y])
        best_node = nearest_node
        for neighbor in neighbors:
            cost = neighbor.cost + np.linalg.norm([new_node.x - neighbor.x, new_node.y - neighbor.y])
            if cost < min_cost and self.is_collision_free(neighbor):
                best_node = neighbor
                min_cost = cost
        new_node.cost = min_cost
        new_node.parent = best_node
        return new_node

    def find_neighbors(self, new_node):
        return [node for node in self.node_list
                if np.linalg.norm([node.x - new_node.x, node.y - new_node.y]) < self.search_radius]

    def rewire(self, new_node, neighbors):
        for neighbor in neighbors:
            cost = new_node.cost + np.linalg.norm([neighbor.x - new_node.x, neighbor.y - new_node.y])
            if cost < neighbor.cost and self.is_collision_free(neighbor):
                neighbor.parent = new_node
                neighbor.cost = cost

    def reached_goal(self, node):
        return np.linalg.norm([node.x - self.goal.x, node.y - self.goal.y]) < self.goal_region_radius

    def steer(self, from_node, to_node):
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node = myNode(from_node.x + self.step_size * math.cos(theta),
                        from_node.y + self.step_size * math.sin(theta))
        
        if not self.is_collision_free(new_node):
            return None  # Return None if collision is detected
        
        new_node.cost = from_node.cost + self.step_size
        new_node.parent = from_node
        return new_node
        
    def convert_to_twistStamped(self, throttle, brake, steering):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = Clock().now().to_msg()
        
        # Assuming max speed is 20 m/s for mapping throttle to velocity
        max_speed = 2.0  
        # Compute linear velocity (basic model: throttle increases speed, brake stops it)
        twist_msg.twist.linear.x = (throttle - brake) * max_speed

        # Compute angular velocity (assuming full steer = ±1 corresponds to max 30 degrees turn rate)
        max_turn_rate = 0.3  # rad/s (adjust based on your vehicle's real turn rate)
        twist_msg.twist.angular.z = steering * max_turn_rate

        return twist_msg

    def twistStamped_stop(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = Clock().now().to_msg()
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.angular.z = 0.0
        return twist_msg


    def move_vehicle_along_path(self):
        if self.path:
            target_location = self.path[0]
            curr_point_idx = 0
            while(True):
                distance_to_target_location = np.linalg.norm([self.curr_pos.x - target_location[0], self.curr_pos.y - target_location[1]])
                if distance_to_target_location < 0.1:
                    curr_point_idx += 1
                    if curr_point_idx < len(self.path):
                        target_location = self.path[curr_point_idx]
                # Calculate steering and throttle
                steering_angle = get_steering_angle(self.curr_pos, self.curr_orientation, target_location)
                steering = steering_angle * 0.1  # Smoother steering
                throttle = calculate_throttle(self.curr_pos, target_location)

                # Debug: Print control values
                print(f"Applying Control - Throttle: {throttle}, Steering: {steering}")

                # Check distance to goal
                distance_to_goal = np.linalg.norm([self.curr_pos.x - self.goal.x, self.curr_pos.y - self.goal.y])
                print(f"vechile position is: {self.curr_pos} and dis to goal is: {distance_to_goal}")
                if distance_to_goal < self.goal_region_radius:
                    print("Goal reached! Stopping the car.")
                    self.vel_publisher.publish(self.twistStamped_stop())
                    # self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0))
                    return

                # Check for collisions
                if not self.is_collision_free(myNode(target_location[0], target_location[1])):
                    print("Obstacle detected! Stopping and replanning...")
                    self.vel_publisher.publish(self.twistStamped_stop())
                    # self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0))  # Stop the vehicle
                    time.sleep(1)  # Add a small delay before replanning
                    self.plan()  # Replan the path
                    return  # Exit the current path-following loop

                # Apply control to the vehicle
                self.vel_publisher.publish(self.convert_to_twistStamped(throttle,0.0,steering))
                # self.vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=0.0, steer=steering))
                time.sleep(0.1)  # Add a small delay for smoother movement
        else:
            self.vel_publisher.publish(self.twistStamped_stop())
            # self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0))
            print("No path to follow.")

    def generate_final_path(self, goal_node):
        path = []
        node = goal_node
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        path = path[::-1]
        print("Generated Path:", path)  # Debug: Print the generated path
    
        # X = [point[0] for point in path]
        # Y = [point[1] for point in path]
        # plt.scatter(X, Y, color='red', label='path')
        # plt.xlabel("X")
        # plt.ylabel("Y")
        # plt.title("Path Visualization")
        # plt.legend()
        # plt.show()
        return path        

    def plan(self):
        for i in range(self.max_iter):
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, rand_node)
            new_node = self.steer(nearest_node, rand_node)

            if new_node is not None:
                self.node_list.append(new_node)
                print(new_node.x,new_node.y)
                if self.reached_goal(new_node):
                    self.path = self.generate_final_path(new_node)
                    self.goal_reached = True
                    print("Goal reached! Stopping the vehicle.")
                    self.move_vehicle_along_path()
                    print("Done !!")
                    return
                

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanning()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()




# def process_lidar_data(self, point_cloud_data):
#     point_step = point_cloud_data.point_step
#     row_step = point_cloud_data.row_step 
#     self.lidar_data = []  # Clear previous data

#     for row in range(point_cloud_data.height):  
#         for col in range(point_cloud_data.width):
#             offset = row * row_step + col * point_step
#             x, y, z = struct.unpack_from('fff', point_cloud_data.data, offset=offset)

#             # Ignore invalid points (NaN values)
#             if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
#                 continue

#             self.lidar_data.append((x, y, z))

#     # print(self.lidar_data)

# def process_lidar_data(self, point_cloud_data):
#     dtype = np.dtype([("x", np.float32), ("y", np.float32), ("z", np.float32)])

#     points = np.frombuffer(point_cloud_data.data, dtype=dtype)

#     # Convert to a list and remove NaN values
#     self.lidar_data = [
#         (p["x"], p["y"], p["z"])
#         for p in points
#         if np.isfinite(p["x"]) and np.isfinite(p["y"]) and np.isfinite(p["z"])
#     ]