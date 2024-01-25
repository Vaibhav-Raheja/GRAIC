import carla
import math
import numpy as np

import csv


def get_speed(velocity):
    velocity = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    # print(velocity)
    return velocity

class Agent():
    def __init__(self, vehicle=None, L=2.875):
        self.vehicle = vehicle
        self.L = L  # Wheelbase of the vehicle
        self.prev_velocity_error = 0.0  # Initialize previous velocity error for PD controller
        self.dt = 1/60
        self.integral_velocity_error = 0
        self.last_cte = 0.0


    def longitudinal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):
        straight_speed = 0.8
        turn_speed = 0.3
        brake = 0.0
        if len(future_unreached_waypoints) >= 2:
            first_waypoint = future_unreached_waypoints[0]
            second_waypoint = future_unreached_waypoints[1]
            angle = math.atan2(second_waypoint[1] - first_waypoint[1], second_waypoint[0] - first_waypoint[0])
            # Ensure the angle is within the range [0, 2*pi]
            angle = (angle + 2 * math.pi) % (2 * math.pi)
        else:
            angle = 0.0
        angle_error = abs(math.degrees(curr_yaw) - math.degrees(angle))
        angle_error = angle_error % 360
        if 340 > angle_error > 10: 
            target_velocity = turn_speed
            # brake = 0.5
        else:
            target_velocity = straight_speed
        # print(target_velocity,math.degrees(angle),math.degrees(curr_yaw),angle_error)
        return target_velocity, brake
    # def stanley_controller(self, curr_x, curr_y, curr_yaw, vel, waypoints):
    #     # Parameters for Stanley controller
    #     k = 0.11  # Control gain for cross-track error
    #     k_yaw = 1.2     # Control gain for heading error
    #     k_soft = 0.75  # Softening constant
        
    #     # Find the nearest point on the path
    #     min_dist = float('inf')
    #     nearest_point = None
    #     nearest_waypoint_index = None
    #     for i, waypoint in enumerate(waypoints):
    #         dist = math.sqrt((curr_x - waypoint[0])**2 + (curr_y - waypoint[1])**2)
    #         if dist < min_dist:
    #             min_dist = dist
    #             nearest_point = waypoint
    #             nearest_waypoint_index = i
        
    #     # Calculate cross-track error
    #     cross_track_error = min_dist
    #     if nearest_waypoint_index is not None:
    #         path_direction = math.atan2(waypoints[nearest_waypoint_index][1] - curr_y, waypoints[nearest_waypoint_index][0] - curr_x)
    #         cross_track_direction = math.atan2(nearest_point[1] - curr_y, nearest_point[0] - curr_x)
    #         cross_track_error *= math.copysign(1, math.sin(path_direction - cross_track_direction))
        
    #     # Calculate heading error
    #     path_direction = math.atan2(waypoints[min(len(waypoints)-1, nearest_waypoint_index + 1)][1] - waypoints[nearest_waypoint_index][1],
    #                                 waypoints[min(len(waypoints)-1, nearest_waypoint_index + 1)][0] - waypoints[nearest_waypoint_index][0])
    #     heading_error = path_direction - curr_yaw
        
    #     # Normalize heading error to be within [-pi, pi]
    #     heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
    #     # Steering control law (now including the k_yaw gain)
    #     steer_angle = k_yaw * heading_error + math.atan2(k * cross_track_error, k_soft + vel)
        
    #     # Normalize steering angle to be within [-1, 1] assuming that the steering angle is within [-pi, pi]
    #     steer_angle = max(-1.0, min(1.0, steer_angle / math.pi))
    #     print(heading_error,steer_angle)
    #     return steer_angle



    def run_step(self, filtered_obstacles, waypoints, vel, transform, boundary):
        
        # save_waypoints_to_csv(waypoints)
        # save_boundary_to_csv(boundary)

        # Get the current velocity in m/s
        # print(vel)
        current_velocity = get_speed(vel)
        # print(current_velocity)
        # Determine the target velocity
        # target_velocity = self.longitudinal_controller(current_velocity, target_velocity)


        # Get the current position and orientation
        curr_x = transform.location.x
        curr_y = transform.location.y
        curr_yaw = math.radians(transform.rotation.yaw)

        # Calculate the steering angle
        target_velocity , brake = self.longitudinal_controller(curr_x, curr_y, current_velocity, curr_yaw, waypoints)

        steer = self.stanley_controller(curr_x, curr_y, curr_yaw, current_velocity, waypoints)
        # steer = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, waypoints)
        # print(steer)
        # steer = np.clip(steer,-1.0,1.0)
        
        # Create the control command
        control = carla.VehicleControl()
        control.throttle = target_velocity
        # control.throttle = 0.0
        control.steer = steer
        control.brake = brake  # Set the brake to 0 for now

        # save_to_csv(filtered_obstacles, 'filtered_obstacles.csv')
        # Return the control command
        return control
