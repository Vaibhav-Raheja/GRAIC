"""
GRAIC 2023 Autonomous Racing Agent - Final Production Implementation

This module implements a complete agent controller using Pure Pursuit with PD control
for lateral control and adaptive speed control for longitudinal control.
"""
import carla
import math
import numpy as np


def get_speed(velocity):
    """
    Calculate speed magnitude from CARLA velocity vector.

    Args:
        velocity: carla.Vector3D velocity vector

    Returns:
        float: Speed magnitude in m/s
    """
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)


class Agent():
    """
    Production-ready autonomous racing agent with Pure Pursuit and PD control.

    This agent uses:
    - Pure Pursuit algorithm for path following
    - PD (Proportional-Derivative) controller for steering
    - Adaptive speed control based on heading error and turn sharpness

    Attributes:
        vehicle: CARLA vehicle actor (optional)
        L: Wheelbase of the vehicle in meters (default: 2.875)
    """
    def __init__(self, vehicle=None, L=2.875):
        self.vehicle = vehicle
        self.L = L  # Wheelbase of the vehicle

    def control(self, curr_x, curr_y, curr_vel, curr_yaw, waypoints):
        """
        Calculate control outputs using Pure Pursuit with PD steering and adaptive speed.

        Args:
            curr_x: Current x position in meters
            curr_y: Current y position in meters
            curr_vel: Current velocity in m/s
            curr_yaw: Current yaw angle in radians
            waypoints: List of waypoints to follow [[x, y, z], ...]

        Returns:
            tuple: (target_velocity, target_steering, brake) control values
        """
        prev_error = 0.0
        lookahead_distance = 15.0
        kp = 1.3
        kd = 0.5
        max_speed = 0.8
        min_speed = 0.3
        max_angle_error = 65  # degrees
        sharp_turn_threshold = 20  # degrees
        high_speed_threshold = 0.7
        
        max_possible_speed = 30
        brake = 0
        
        
        curr_vel = min(curr_vel / max_possible_speed, 1.0)

        # Find lookahead point
        lookahead_point = None
        for waypoint in waypoints:
            if len(waypoint) >= 2 and math.dist((curr_x, curr_y), waypoint[:2]) > lookahead_distance:
                lookahead_point = waypoint
                break

        if lookahead_point is None:
            # If no suitable lookahead point, use the last waypoint
            if len(waypoints) > 0 and len(waypoints[-1]) >= 2:
                lookahead_point = waypoints[-1]
            else:
                return 0.0, 0.0, 1.0  # Stop if no valid waypoints

        # Calculate heading error
        angle_to_waypoint = math.atan2(lookahead_point[1] - curr_y, lookahead_point[0] - curr_x)
        heading_error = math.atan2(math.sin(angle_to_waypoint - curr_yaw), math.cos(angle_to_waypoint - curr_yaw))
        
        # Sharp turn and high-speed logic
        if abs(math.degrees(heading_error)) > sharp_turn_threshold or curr_vel>high_speed_threshold:
            target_velocity = max(min_speed, curr_vel - min_speed) # Slow down if it's a sharp turn and at high speed
            brake = 0.65*curr_vel
            # brake = min(brake / 1, 1.0)
            # print(brake)
        else:
            target_velocity = min(max_speed, curr_vel + 0.05)  
        
        # PD control for steering
        pd_steering = kp * heading_error + kd * (heading_error - prev_error)
        target_steering = np.arctan(2 * self.L * np.sin(pd_steering) / lookahead_distance)

        # Adjust target velocity based on heading error
        angle_error = abs(math.degrees(heading_error))  # Use heading error for velocity adjustment
        angle_error = angle_error % 360
        # print(angle_error)
        # print(angle_error / max_angle_error)
        normalized_angle_error = min(angle_error / max_angle_error, 1.0)
        target_velocity = max_speed - ((max_speed - min_speed) * normalized_angle_error)

        return target_velocity, target_steering, brake

    def run_step(self, filtered_obstacles, waypoints, vel, transform, boundary):
        """Execute one step of navigation."""
        try:
            # Validate inputs
            if not waypoints or len(waypoints) == 0:
                # No waypoints available, stop the vehicle
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 1.0
                return control

            current_velocity = get_speed(vel)

            # Get the current position and orientation
            curr_x = transform.location.x
            curr_y = transform.location.y
            curr_yaw = math.radians(transform.rotation.yaw)

            # Calculate control outputs
            target_velocity, steer, brake = self.control(curr_x, curr_y, current_velocity, curr_yaw, waypoints)

            # Clamp control values to valid ranges
            target_velocity = np.clip(target_velocity, 0.0, 1.0)
            steer = np.clip(steer, -1.0, 1.0)
            brake = np.clip(brake, 0.0, 1.0)

            # Create the control command
            control = carla.VehicleControl()
            control.throttle = target_velocity
            control.steer = steer
            control.brake = brake

            return control

        except Exception as e:
            # Log error and return safe control (stop the vehicle)
            print(f"Error in agent control: {e}")
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            return control
