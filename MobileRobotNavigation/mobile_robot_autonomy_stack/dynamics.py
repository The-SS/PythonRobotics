"""
Robot dynamics model.

Author: Sleiman Safaoui (@The-SS)

"""
import numpy as np
from PathTracking.stanley_controller.stanley_controller import normalize_angle


class StanleyState(object):
    """
    Class representing the state of a vehicle.
    Edited copy of the PathTracking.stanley_controller.stanley_controller State

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, wb=1.0, max_steer=np.radians(30.0), dt=0.1):
        """Instantiate the object."""
        super(StanleyState, self).__init__()
        # state
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        # parameters
        self.wb = wb
        self.max_steer = max_steer
        # time step
        self.dt = dt

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -self.max_steer, self.max_steer)

        self.x += self.v * np.cos(self.yaw) * self.dt
        self.y += self.v * np.sin(self.yaw) * self.dt
        self.yaw += self.v / self.wb * np.tan(delta) * self.dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * self.dt

    def reset_state(self, x, y, yaw, v):
        """
        Forcefully change the state.

        :param x: (float) x-position
        :param y: (float) y-position
        :param yaw: (float) steering angle
        :param v: (float) forward velocity
        """
        self.x, self.y, self.yaw, self.v = x, y, yaw, v


class StanleyCtrl:
    """
    Stanley Controller class. Combines the functions from
    PathTracking.stanley_controller.stanley_controller
    into one class
    """
    def __init__(self, state: StanleyState, Kp: float, k: float):
        self.state = state
        self.Kp = Kp
        self.k = k

    def calc_target_index(self, cx, cy):
        """
        Compute index in the trajectory list of the target.

        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = self.state.x + self.state.wb * np.cos(self.state.yaw)
        fy = self.state.y + self.state.wb * np.sin(self.state.yaw)

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(self.state.yaw + np.pi / 2),
                          -np.sin(self.state.yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle

    def pid_control(self, target, current):
        """
        Proportional control for the speed.

        :param target: (float)
        :param current: (float)
        :return: (float)
        """
        return self.Kp * (target - current)

    def stanley_control(self, cx, cy, cyaw, last_target_idx):
        """
        Stanley steering control.

        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = self.calc_target_index(cx, cy)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = normalize_angle(cyaw[current_target_idx] - self.state.yaw)
        print(theta_e)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.k * error_front_axle, self.state.v)
        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx
