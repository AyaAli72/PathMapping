import numpy as np
import random
import math
import time
import matplotlib.pyplot as plt
import carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.load_world('Town03')

blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter("vehicle.audi.tt")[0]
starting_point = world.get_map().get_spawn_points()[0]
vehicle = world.try_spawn_actor(vehicle_bp,starting_point)
vehicle_pos = vehicle.get_transform()
lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
lidar_blueprint = blueprint_library.find("sensor.lidar.ray_cast")
lidar_blueprint.set_attribute("channels", str(64))
lidar_blueprint.set_attribute("points_per_second", str(500000))
lidar_blueprint.set_attribute("rotation_frequency", str(10.0))
lidar_blueprint.set_attribute("upper_fov", str(30.0))
lidar_blueprint.set_attribute("lower_fov", str(-25.0))
lidar_blueprint.set_attribute("range", str(100.0))
spectator_rotation = vehicle_pos.rotation
lidar_transform = carla.Transform(starting_point.location, spectator_rotation)
lidar_sensor = world.spawn_actor(lidar_blueprint, lidar_transform, attach_to = vehicle)
gps_bp = blueprint_library.find('sensor.other.gnss')
gps_bp.set_attribute("sensor_tick", "1.0")  
gps_location = carla.Transform(carla.Location(x=0, y=0, z=2))  
gps_sensor = world.spawn_actor(gps_bp, gps_location, attach_to=vehicle)

bounding_box = vehicle.bounding_box
vehicle_length = bounding_box.extent.x

def get_steering_angle(self, vehicle_pos, target_location):
    dx = target_location[0] - vehicle_pos.location.x
    dy = target_location[1] - vehicle_pos.location.y
    target_angle = math.atan2(dy, dx)
    current_angle = math.radians(vehicle_pos.rotation.yaw)
    steering_angle = target_angle - current_angle

    # Normalize steering angle to [-1, 1]
    steering_angle = max(-1.0, min(1.0, steering_angle / (math.pi/4)))
    return steering_angle

def calculate_throttle(self, vehicle_pos, target_location):
    distance = np.linalg.norm([
        target_location[0] - vehicle_pos.location.x,
        target_location[1] - vehicle_pos.location.y])

    # Smooth throttle control
    if distance < 1.0:  # Slow down when close
        return distance / 2.0
    else:
        return min(1.0, 0.5 + (distance / (2*self.step_size)))

# Node class representing a state in the space
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

# RRT* algorithm
class RRTStar:
    def __init__(self, vehicle_length, start, goal, map_size, carla_world, vehicle, lidar_sensor, gps_sensor, step_size=1.0, max_iter=1000):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.map_size = map_size
        self.step_size = step_size
        self.max_iter = max_iter
        self.node_list = [self.start]
        self.goal_region_radius = 4
        self.search_radius = 5
        self.path = None
        self.goal_reached = False
        self.carla_world = carla_world
        self.vehicle = vehicle
        self.lidar_sensor = lidar_sensor
        self.gps_sensor = gps_sensor
        self.lidar_data = []
        self.vehicle_length = vehicle_length

    def process_lidar_data(self, point_cloud_data):
        points = np.frombuffer(point_cloud_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        self.lidar_data = points[:, :3]  # Extract x, y, z coordinates
        print(f"LIDAR data points: {len(self.lidar_data)}")  # طباعة عدد النقاط
        return self.lidar_data

    def gps_callback(self, data):
        latitude = data.latitude
        longitude = data.longitude
        altitude = data.altitude
        print("Car Location:", self.vehicle.get_transform().location)

    def get_nearest_node(self, node_list, rand_node):
        distances = [np.linalg.norm([node.x - rand_node.x, node.y - rand_node.y]) for node in node_list]
        nearest_node_idx = np.argmin(distances)
        return node_list[nearest_node_idx]

    def is_collision_free(self, node):
        if len(self.lidar_data) == 0:
            return True  # Assume no obstacles if no LIDAR data is available

        safety_margin = 2.0  # Safety margin around the vehicle
        for point in self.lidar_data:
            obstacle_distance = math.sqrt((point[0] - node.x) ** 2 + (point[1] - node.y) ** 2)

            # If the distance to the obstacle is less than the safety margin, there is a collision
            if obstacle_distance < self.vehicle_length + safety_margin:
                print(f"Collision detected at ({node.x}, {node.y}) with obstacle at ({point[0]}, {point[1]})")
                return False
        return True

    def get_random_node(self):
        if random.random() > 0.2:
            return Node(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))
        else:
            return Node(self.goal.x, self.goal.y)

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
                vehicle_pos = self.vehicle.get_transform()

                # Check for obstacles
                if not self.is_collision_free(Node(target_location[0], target_location[1])):
                    print("Obstacle detected! Replanning from current position to next point...")
                    # Current position of the vehicle
                    current_position = [vehicle_pos.location.x, vehicle_pos.location.y]
                    # Next point in the path (if available)
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
                            self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0))
                            return
                    else:
                        print("No next point to replan. Stopping the vehicle.")
                        self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0))
                        return

                # Calculate steering and throttle
                steering_angle = self.get_steering_angle(vehicle_pos, target_location)
                steering = steering_angle * 0.1  # Smoother steering
                throttle = self.calculate_throttle(vehicle_pos, target_location)

                # Apply control to the vehicle
                self.vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=0.0, steer=steering))
                print(f"Steering: {steering}, Throttle: {throttle}")  # طباعة زاوية التوجيه وقيمة الدفع
                time.sleep(0.1)  # Small delay for smoother movement

                # Check distance to the current point
                distance_to_point = np.linalg.norm([vehicle_pos.location.x - target_location[0],
                                                vehicle_pos.location.y - target_location[1]])
                if distance_to_point < self.step_size:
                    i += 1  # Move to the next point in the path

                # Check distance to the goal
                distance_to_goal = np.linalg.norm([vehicle_pos.location.x - self.goal.x,
                                                vehicle_pos.location.y - self.goal.y])
                if distance_to_goal < self.goal_region_radius:
                    print("Goal reached! Stopping the car.")
                    self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0))
                    return
        else:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0))
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

    def generate_final_path(self, goal_node):
        path = []
        node = goal_node
        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        path = path[::-1]
        print("Generated Path:", path)  # طباعة المسار
        return path

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
        # Calculate the angle between the vehicle's current orientation and the target location
        dx = target_location[0] - vehicle_pos.location.x
        dy = target_location[1] - vehicle_pos.location.y
        target_angle = math.atan2(dy, dx)
        current_angle = math.radians(vehicle_pos.rotation.yaw)
        steering_angle = target_angle - current_angle
        return steering_angle

    def calculate_throttle(self, vehicle_pos, target_location):
        # Calculate the distance to the target location
        distance = np.linalg.norm([target_location[0] - vehicle_pos.location.x,
                                   target_location[1] - vehicle_pos.location.y])
        # Adjust throttle based on distance
        return min(1.0, distance / self.step_size)

if __name__ == "__main__":
    try:
        start_location = [starting_point.location.x, starting_point.location.y]
        goal_location = [start_location[0] + 5, start_location[1] + 20]
        rrt_star = RRTStar(
            vehicle_length=vehicle_length,
            start=start_location,
            goal=goal_location,
            map_size=[100, 100],
            carla_world=world,
            vehicle=vehicle,
            lidar_sensor=lidar_sensor,
            gps_sensor=gps_sensor
        )
        rrt_star.gps_sensor.listen(lambda data: rrt_star.gps_callback(data))
        rrt_star.lidar_sensor.listen(lambda point_cloud: rrt_star.process_lidar_data(point_cloud))
        rrt_star.plan()
        # rrt_star.prints()
    finally:
        print("Simulation ended.")
        time.sleep(1)

vehicle.destroy()
lidar_sensor.destroy()
gps_sensor.destroy()
