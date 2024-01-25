import carla
import math
import numpy as np
import cvxpy as cp

class MPCController:
    def __init__(self, horizon=10, dt=0.1):
        self.horizon = horizon
        self.dt = dt
        # Define vehicle parameters
        self.L = 2.875  # Wheelbase

    def bicycle_model(self, x, y, psi, v, delta, a):
        x_next = x + v * cp.cos(psi) * self.dt
        y_next = y + v * cp.sin(psi) * self.dt
        psi_next = psi + v / self.L * cp.tan(delta) * self.dt
        v_next = v + a * self.dt
        return x_next, y_next, psi_next, v_next

    def optimize(self, x, y, psi, v, waypoints):
        # Define optimization variables
        delta = cp.Variable(self.horizon)  # Steering angle
        a = cp.Variable(self.horizon)      # Acceleration

        cost = 0
        states = []
        for t in range(self.horizon):
            x, y, psi, v = self.bicycle_model(x, y, psi, v, delta[t], a[t])
            states.append([x, y, psi, v])
            waypoint_idx = min(t, len(waypoints) - 1)
            cost += cp.sum_squares([x - waypoints[waypoint_idx][0], y - waypoints[waypoint_idx][1]])

        constraints = [-np.pi/4 <= delta, delta <= np.pi/4, -3 <= a, a <= 3]

        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        if delta.value is not None and a.value is not None:
            return delta.value[0], a.value[0]
        else:
            return 0, 0



def run_mpc(controller, initial_state, waypoints):
    x, y, psi, v = initial_state
    for waypoint in waypoints:
        delta, a = controller.optimize(x, y, psi, v, waypoint)
        x, y, psi, v = controller.bicycle_model(x, y, psi, v, delta, a)
        # Apply delta and a to the vehicle (pseudo-code)
        # vehicle.apply_control(delta, a)
        # Update the state for the next iteration























def get_speed(velocity):
    velocity = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    # print(velocity)
    return velocity

class Agent:
    def __init__(self, vehicle=None, L=2.875):
        self.vehicle = vehicle
        self.L = L  # Wheelbase
        self.mpc_controller = MPCController(horizon=10, dt=1/60)  # Initialize MPC with desired horizon and timestep

    def run_step(self, filtered_obstacles, waypoints, vel, transform, boundary):
        current_velocity = get_speed(vel)
        curr_x = transform.location.x
        curr_y = transform.location.y
        curr_yaw = math.radians(transform.rotation.yaw)

        # Assuming waypoints is a list of [x, y] points
        # Pass the current state and waypoints to MPC optimize method
        delta, a = self.mpc_controller.optimize(curr_x, curr_y, curr_yaw, current_velocity, waypoints)

        # Create and return the vehicle control command
        control = carla.VehicleControl()
        control.throttle = max(0.0, a) if a > 0 else 0.0
        control.brake = abs(min(0.0, a)) if a < 0 else 0.0
        control.steer = delta
        return control
