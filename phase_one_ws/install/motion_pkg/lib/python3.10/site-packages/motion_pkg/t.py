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




############################################################## Topics Listeining ##################################################################   
        self.wheel_control_speed = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.robot_desired_speed = self.create_publisher(TwistStamped, "/bumperbot_controller/cmd_vel", 10)
        self.lidar_subscriber = self.create_subscription(PointCloud2, '/lidar_points', self.process_lidar_data, 10)
        self.gps_subscriber = self.create_subscription(Pose2D, "/robot_pose", self.gps_callback, 10)
##################################################################################################################################################   




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
        self.current_x = msg.x
        self.current_x = msg.y
        self.current_theta = msg.theta
    def get_current_position(self):
        # Return the current position as a tuple
        return self.current_x, self.current_y, self.current_theta
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
        new_node = Node(from_node.x + self.step_size * math.cos(theta),
                        from_node.y + self.step_size * math.sin(theta))

        # تأكد من أن العقدة الجديدة ضمن نطاق الخريطة
        if new_node.x < 0 or new_node.x > self.map_size[0] or new_node.y < 0 or new_node.y > self.map_size[1]:
            print("New node is out of map bounds.")
            return None

        if not self.is_collision_free(new_node):
            print("Collision detected in steer function.")
            return None  # Return None if collision is detected

        new_node.cost = from_node.cost + self.step_size
        new_node.parent = from_node
        print(f"New node created: ({new_node.x}, {new_node.y})")  # طباعة العقدة الجديدة
        return new_node

    def move_vehicle_along_path(self):
        if self.path:
            print("Moving along path...")
            i = 0  # Index for the current point in the path
            while i < len(self.path):
                target_location = self.path[i]
                vehicle_pos = [self.current_x,self.current_y]

                # Check for obstacles
                if not self.is_collision_free(Node(target_location[0], target_location[1])):
                    print("Obstacle detected! Replanning from current position to next point...")
                    # Current position of the vehicle
                    current_position = [vehicle_pos[0] ,vehicle_pos[1] ]   #Add Test Point
                    if i + 1 < len(self.path):
                        next_point = self.path[i + 1]
                        # Replan between current position and next point
                        new_segment = self.replan_between_points(current_position, next_point)
                        if new_segment:
                            # Replace the old segment with the new one
                            self.path = self.path[:i] + new_segment + self.path[i + 1:]
                            print("Path updated with new segment.")
                        else:
                            print("Failed to replan. Stopping the vehicle.")

                            #---------------------------------Stop the Robot-----------------------------------#
                            #self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0)) #
                            #----------------------------------------------------------------------------------#
                            return
                    else:
                        print("No next point to replan. Stopping the vehicle.")
                        #---------------------------------Stop the Robot-----------------------------------#
                        #self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0)) #
                        #----------------------------------------------------------------------------------#
                        return
                    
                    
                    
                    #########################################Move the Robot##########################################################################
                    # Calculate steering and throttle
                    steering_angle = self.get_steering_angle(vehicle_pos, target_location)  #Replace with theta from the Current position node
                    steering = steering_angle * 0.1  # Smoother steering
                    throttle = self.calculate_throttle(vehicle_pos, target_location)

                     # Apply control to the vehicle
                    self.vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=0.0, steer=steering))
                    print(f"Steering: {steering}, Throttle: {throttle}")  # طباعة زاوية التوجيه وقيمة الدفع
                    time.sleep(0.1)  # Small delay for smoother movement
                    ##################################################################################################################################


                    # Check distance to the current point
                    distance_to_point = np.linalg.norm([vehicle_pos[0] - target_location[0],vehicle_pos[1] - target_location[1]])

                    if distance_to_point < self.step_size:
                        i += 1  # Move to the next point in the path

                    distance_to_goal = np.linalg.norm([vehicle_pos[0] - self.goal.x,vehicle_pos[1] - self.goal.y])

                    if distance_to_goal < self.goal_region_radius:
                        print("Goal reached! Stopping the car.")
                        #---------------------------------Stop the Robot-----------------------------------#
                        #self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0)) #
                        #----------------------------------------------------------------------------------#
                    return
                else:
                    #---------------------------------Stop the Robot-----------------------------------#
                    #self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0)) #
                    #----------------------------------------------------------------------------------#
                    print("No path to follow.")

    def replan_between_points(self, start, goal):
        # Check if start and goal points are collision-free
        if not self.is_collision_free(Node(start[0], start[1])) or not self.is_collision_free(Node(goal[0], goal[1])):
            print("Start or goal point is in collision. Cannot replan.")
            return None

        # Reset node list
        self.node_list = [Node(start[0], start[1])]

        # Run RRT* between the points
        for _ in range(self.max_iter):
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, rand_node)
            new_node = self.steer(nearest_node, rand_node)

            if new_node is not None:
                self.node_list.append(new_node)

                # If the goal is reached
                if self.reached_goal(new_node):
                    return self.generate_final_path(new_node)

        return None  # If planning fails
    
############################################################### Gives the the whole path  ##################################################################

  
    def generate_final_path(self, goal_node):
        path = []
        node = goal_node
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        path = path[::-1]
        print("Generated Path:", path)  # طباعة المسار
        return path
    
################################################################## Gives the path point by point  ##################################################################
    def plan(self):
        print("Replanning...")
        self.node_list = [self.start]
        self.path = None
        self.goal_reached = False

        for i in range(self.max_iter):
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, rand_node)
            new_node = self.steer(nearest_node, rand_node)

            if new_node is not None:
                self.node_list.append(new_node)

                if self.reached_goal(new_node):
                    self.path = self.generate_final_path(new_node)
                    if self.path:
                        self.goal_reached = True
                        print("Goal reached! Stopping the vehicle.")
                        self.move_vehicle_along_path()
                    return  # Exit after generating the path
    

    def get_steering_angle(self, vehicle_pos, target_location):
        
        steering_angle =self.current_theta
        return steering_angle








################################################################ Main ################################################################################

def main():
    rclpy.init()

    start_location = [0.0, 0.0]
    goal_location = [start_location[0] + 5, start_location[1] + 20]
    vehicle_length = 0.0951
    map_size = [100, 100]  # Example map size

    # Create the RRT* node
    rrt_star_node = RRTStar(vehicle_length=vehicle_length,goal=goal_location,map_size=map_size)

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















'''
############################################################## Motion Algorithm #######################################################################

    def get_steering_angle(self, vehicle_pos, target_location):
        dx = target_location[0] - vehicle_pos.location.x
        dy = target_location[1] - vehicle_pos.location.y
        target_angle = math.atan2(dy, dx)
        current_angle = math.radians(vehicle_pos.rotation.yaw)
        steering_angle = target_angle - current_angle

        # Normalize steering angle to [-1, 1]
        steering_angle = max(-1.0, min(1.0, steering_angle / (math.pi/4)))

        # Use stored linear velocity (default to 0.0)
        linear_velocity = getattr(self, "current_linear_velocity", 0.0)

        # Publish Twist message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = linear_velocity
        twist_msg.twist.angular.z = steering_angle
        self.robot_desired_speed.publish(twist_msg)

        # Compute and publish wheel speeds
        robot_speed = np.array([[linear_velocity], [steering_angle]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_control_speed.publish(wheel_speed_msg)

        # Save angular velocity
        self.current_angular_velocity = steering_angle

        return steering_angle
    
    def calculate_throttle(self, vehicle_pos, target_location):
        distance = np.linalg.norm([
            target_location[0] - vehicle_pos.location.x,
            target_location[1] - vehicle_pos.location.y])

        # Smooth throttle control
        if distance < 1.0:
            throttle = distance / 2.0
        else:
            throttle = min(1.0, 0.5 + (distance / (2 * self.step_size)))

        # Use stored angular velocity (default to 0.0)
        angular_velocity = getattr(self, "current_angular_velocity", 0.0)

        # Publish Twist message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = throttle
        twist_msg.twist.angular.z = angular_velocity
        self.robot_desired_speed.publish(twist_msg)

        # Compute and publish wheel speeds
        robot_speed = np.array([[throttle], [angular_velocity]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_control_speed.publish(wheel_speed_msg)

        # Save linear velocity
        self.current_linear_velocity = throttle

        return throttle

###############################################################################################################################################

'''

'''
################################################################ Gazebo Sensor Node ###################################################################

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
###############################################################################################################################################
'''