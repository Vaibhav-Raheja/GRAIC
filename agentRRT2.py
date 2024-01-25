import carla
import time
import numpy as np
from matplotlib import pyplot as plt
import math
from scipy.interpolate import CubicSpline
import random
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent))

class Node:
    def __init__(self, location, cost=0, children=[], parent=None):
        self.location = location
        self.cost = cost
        self.children = children
        self.parent = parent

    def add_child(self, node):
        self.children.append(node)

    def set_parent(self, parent):
        self.parent = parent

    def update_cost(self, cost):
        self.cost = cost

class RRTStar:
    def __init__(self, start, goal, limits, obstacles, threshold=0.1):
        self.start = start
        self.goal = goal
        self.dims = len(self.start)
        self.limits = limits
        self.obstacles = obstacles
        self.distance_threshold = threshold

        self.root = Node(list(start), cost=0)
        self.last_node = None
        self.node_list = [self.root]

        self.goal_eps = 0.1
        self.maximum_iterations = 1000

    def generate_random_point(self):
        location = []
        for i in range(self.dims):
            rand = random.random()
            scaling = self.limits[i][1] - self.limits[i][0]
            location.append(rand*scaling + self.limits[i][0])
        return location

    def check_collision(self, location):
        for obs in self.obstacles:
            dist = self.distance(obs[:-1], location)
            if dist <= obs[-1]:
                return True
        return False

    def check_path_collision(self, location_1, location_2):
        P1 = np.array(location_1)
        P2 = np.array(location_2)
        for obs in self.obstacles:
            Q = np.array(obs[:-1])
            t = np.dot(Q - P1, P2 - P1)/np.dot(P2 - P1, P2 - P1)
            t = max(min(t, 1), 0)
            pt = P1 + t*(P2 - P1)
            if np.dot(pt - Q, pt - Q) <= obs[-1]**2:
                return True
        return False

    def steer(self, start_location, end_location):
        dist = self.distance(start_location, end_location)
        if dist < self.distance_threshold:
            return end_location
        else:
            start_loc = np.array(start_location)
            direction = np.array(end_location) - start_loc
            return tuple(start_loc + self.distance_threshold * direction / dist)
        pass

    @staticmethod
    def distance(a, b):
        diff = np.array(a) - np.array(b)
        return np.sqrt(np.dot(diff, diff))

    def find_nearest(self, x):
        min_distance = np.inf
        min_node = None
        for n in self.node_list:
            dist = self.distance(n.location, x)
            if  dist < min_distance:
                min_distance = dist
                min_node = n
        return min_node, min_distance

    def find_k_closest(self, x, k):
        distances = np.array([])
        for i in range(len(self.node_list)):
            distances = np.append(distances, self.distance(self.node_list[i].location, x))
        indices = distances.argsort()
        distances = distances[indices]
        sorted_node_list = [self.node_list[i] for i in indices[:k]]
        return sorted_node_list, distances[:k]

    def generate_path(self):
        iterations = 0
        found = False
        while iterations < self.maximum_iterations and not found:
            if random.random() < 0.1:
                x = self.goal
            else:
                while True:
                    x = self.generate_random_point()
                    if not self.check_collision(x):
                        break

            x_nearest, d = self.find_nearest(x)
            x_new = self.steer(x_nearest.location, x)
            if self.check_collision(x_new):
                continue
            if not self.check_path_collision(x_nearest.location, x_new):
                new_node = Node(x_new, parent=x_nearest, cost=np.inf)
                x_min = x_nearest

                k = min(100, len(self.node_list))
                X_near, distances = self.find_k_closest(x_new, k)
                for x_near, d in zip(X_near, distances):
                    if not self.check_path_collision(x_near.location, x_new):
                        new_cost = x_near.cost + d
                        if new_cost < new_node.cost:
                            new_node.update_cost(new_cost)
                            x_min = x_near
                new_node.set_parent(x_min)
                self.node_list.append(new_node)

                for x_near in X_near:
                    if x_near == x_min:
                        continue
                    d = self.distance(new_node.location, x_near.location)
                    if not self.check_path_collision(x_new, x_near.location) and \
                            x_near.cost > new_node.cost + d:
                        x_near.set_parent(new_node)
                        x_near.update_cost(new_node.cost + d)

                # plt.cla()
                # plt.plot(self.start[0], self.start[1], 'xr')
                # for n in self.node_list:
                #     c = n.location
                #     if n.parent is not None:
                #         p = n.parent.location
                #         plt.plot([c[0], p[0]], [c[1], p[1]], 'k')
                #         plt.plot(c[0], c[1], 'xc')
                # plt.pause(1)

                goal_dist = self.distance(x_new, self.goal)
                if goal_dist < self.goal_eps:
                    if self.check_path_collision(self.goal, x_new):
                        continue
                    else:
                        goal_node = Node(self.goal, parent=new_node, cost=new_node.cost + goal_dist)
                        self.node_list.append(goal_node)
                        self.last_node = goal_node
                        found = True
                iterations += 1

        return found

    def retrieve_path(self):
        if self.last_node is None:
            found = self.generate_path()
            if not found:
                print("Could not find path in this trial. Try again.")
                return None
        path = []
        current_node = self.last_node
        while current_node is not None:
            path.append(current_node.location)
            current_node = current_node.parent
        return path[::-1]

if __name__=="__main__":
    start = (1, 1)
    goal = (2, 2)
    limits = [(0, 3), (0, 3)]
    obstacles = [(1.5, 1.5, 0.3)]
    path_planner = RRTStar(start, goal, limits, [])
    path_planner.generate_path()
    print(path_planner.retrieve_path())


# def get_path_length(path):
#     le = 0
#     for i in range(len(path) - 1):
#         dx = path[i + 1][0] - path[i][0]
#         dy = path[i + 1][1] - path[i][1]
#         d = math.hypot(dx, dy)
#         le += d

#     return le


# def get_target_point(path, targetL):
#     le = 0
#     ti = 0
#     lastPairLen = 0
#     for i in range(len(path) - 1):
#         dx = path[i + 1][0] - path[i][0]
#         dy = path[i + 1][1] - path[i][1]
#         d = math.hypot(dx, dy)
#         le += d
#         if le >= targetL:
#             ti = i - 1
#             lastPairLen = d
#             break

#     partRatio = (le - targetL) / lastPairLen

#     x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
#     y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

#     return [x, y, ti]


# def line_collision_check(first, second, obstacleList):
#     # Line Equation

#     x1 = first[0]
#     y1 = first[1]
#     x2 = second[0]
#     y2 = second[1]

#     try:
#         a = y2 - y1
#         b = -(x2 - x1)
#         c = y2 * (x2 - x1) - x2 * (y2 - y1)
#     except ZeroDivisionError:
#         return False

#     for (ox, oy, size) in obstacleList:
#         d = abs(a * ox + b * oy + c) / (math.hypot(a, b))
#         if d <= size:
#             return False

#     return True  # OK


# def path_smoothing(path, max_iter, obstacle_list):
#     le = get_path_length(path)

#     for i in range(max_iter):
#         # Sample two points
#         pickPoints = [random.uniform(0, le), random.uniform(0, le)]
#         pickPoints.sort()
#         first = get_target_point(path, pickPoints[0])
#         second = get_target_point(path, pickPoints[1])

#         if first[2] <= 0 or second[2] <= 0:
#             continue

#         if (second[2] + 1) > len(path):
#             continue

#         if second[2] == first[2]:
#             continue

#         # collision check
#         if not line_collision_check(first, second, obstacle_list):
#             continue

#         # Create New path
#         newPath = []
#         newPath.extend(path[:first[2] + 1])
#         newPath.append([first[0], first[1]])
#         newPath.append([second[0], second[1]])
#         newPath.extend(path[second[2] + 1:])
#         path = newPath
#         le = get_path_length(path)

#     return path














def get_speed(velocity):
    velocity = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    # print(velocity)
    return velocity



class Agent():
    def __init__(self, vehicle=None, L=2.875):
        self.vehicle = vehicle
        self.L = L  # Wheelbase of the vehicle
        # all_filtered_obstacles = []


    # def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, waypoints):
    #     lookahead_distance = 5.0
    #     lookahead_point = None

    #     # Find the first waypoint that is at least lookahead_distance away
    #     for waypoint in waypoints:
    #         if math.dist((curr_x, curr_y), waypoint[:2]) > lookahead_distance:
    #             lookahead_point = waypoint
    #             break

    #     if lookahead_point is None:
    #         return 0.0  # No steering if no lookahead point is found

    #     # Calculate the angle to the lookahead point
    #     angle_to_waypoint = math.atan2(lookahead_point[1] - curr_y, lookahead_point[0] - curr_x)

    #     # Calculate the target steering angle using the pure pursuit formula
    #     target_steering = np.arctan(2 * self.L * np.sin(angle_to_waypoint - curr_yaw) / lookahead_distance)
    #     # print(target_steering)
    #     return target_steering
    

    def run_step(self, filtered_obstacles, waypoints, vel, transform, boundary):
        print("Reach Customized Agent")

        # Current vehicle state
        current_location = transform.location
        current_rotation = transform.rotation
        current_velocity = get_speed(vel)

        # Define the start and goal for the RRT* algorithm
        start = [waypoints[0][0], waypoints[0][1]]
        goal = [waypoints[7][0], waypoints[7][1]]  # Last waypoint as the goal

        # Convert filtered_obstacles to the format expected by RRT*
        obstacle_list = self.process_boundaries(boundary)

        # Initialize RRT* and generate path
        rrt = RRTStar(start=start, goal=goal, obstacles=obstacle_list, limits=[(current_location.x-100,current_location.x+100), (current_location.y-100,current_location.y+100)])
        path = rrt.generate_path()

        # If a path is found, smooth it and generate control commands
        if path:
            # Smooth the path
            smooth_path = self.smooth_path(path)

            # Generate control commands to follow the path
            control = self.follow_path(smooth_path, current_velocity, current_rotation)

        else:
            # If no path is found, stop the vehicle
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
        return control
    
    def process_boundaries(self, boundary):
        obstacles = []
        for boundary_side in boundary:  # boundary_side is either left_boundary or right_boundary
            for waypoint in boundary_side:
                # Access the x, y coordinates of the waypoint
                x = waypoint.transform.location.x
                y = waypoint.transform.location.y
                # Create an obstacle tuple with the x, y coordinates and a predefined radius
                obstacle = (x, y, 0.5)  # Example radius of 0.5 meters
                obstacles.append(obstacle)
        return obstacles


    def smooth_path(self, path):
        # Extract x and y coordinates from the path
        x = [point[0] for point in path]
        y = [point[1] for point in path]

        # Create a parameterization variable
        t = np.linspace(0, 1, num=len(path))

        # Create cubic spline representations for x(t) and y(t)
        cs_x = CubicSpline(t, x)
        cs_y = CubicSpline(t, y)

        # Sample the splines at finer intervals to create the smoothed path
        fine_t = np.linspace(0, 1, num=len(path) * 5)  # Increase the number of points
        smooth_x = cs_x(fine_t)
        smooth_y = cs_y(fine_t)

        # Reconstruct the smoothed path
        smoothed_path = list(zip(smooth_x, smooth_y))
        print(smoothed_path)
        return smoothed_path

    def follow_path(self, path, current_velocity, current_rotation):
        # Implement the logic to follow the path
        control = carla.VehicleControl()
        control.throttle = 0.5  # Example throttle value
        control.steer = 0.0     # Example steer value
        return control


























    # def run_step(self, filtered_obstacles, waypoints, vel, transform, boundary):
        
    #     # save_waypoints_to_csv(waypoints)
    #     # save_boundary_to_csv(boundary)

    #     # Get the current velocity in m/s
    #     # print(vel)
    #     current_velocity = get_speed(vel)
    #     # print(current_velocity)
    #     # Determine the target velocity
    #     # target_velocity = self.longitudinal_controller(current_velocity, target_velocity)


    #     # Get the current position and orientation
    #     curr_x = transform.location.x
    #     curr_y = transform.location.y
    #     curr_yaw = math.radians(transform.rotation.yaw)

    #     # Calculate the steering angle
    #     target_velocity = self.longitudinal_controller(curr_x, curr_y, current_velocity, curr_yaw, waypoints)
    #     steer = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, waypoints)

    #     # Create the control command
    #     control = carla.VehicleControl()
    #     control.throttle = target_velocity
    #     # control.throttle = 0.0
    #     control.steer = steer
    #     control.brake = 0  # Set the brake to 0 for now

    #     # save_to_csv(filtered_obstacles, 'filtered_obstacles.csv')
    #     # Return the control command
    #     return control
