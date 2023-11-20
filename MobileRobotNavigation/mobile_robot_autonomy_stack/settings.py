"""
Defines the settings for the autonomy application.
This includes defining:
 - the robot parameters
 - the simulation parameters including the robot start/goal positions
 - the environment in which the mobile robot must operate

All units are in meters.

Author: Sleiman Safaoui (@The-SS)
"""
import numpy as np
from plotting import plot_env


def sim_params():
    # plotting
    show_animation = False

    # autonomy stack update rates
    planning_rate = 5  # Hz
    control_rate = 50  # Hz

    # control parameters
    Kp = 1.0  # speed proportional gain
    k = 0.5  # control gain

    # robot params
    robot_radius = 1.0  # m
    wheelbase = 2 * robot_radius  # m
    max_speed = 3.0  # m/s
    max_steer = np.radians(60.0)  # radian

    # initializations
    sx, sy = 1.5, 1.5  # m
    gx, gy = 13., 13.  # m

    # environment
    grid_size = 0.5  # m
    ox, oy = def_env(ds=grid_size)
    oxd, oyd = def_env(ds=grid_size, dense=True)

    return {"animate": show_animation,
            "planning_rate": planning_rate, "control_rate": control_rate,
            "ctrl_speed_gain": Kp, "ctrl_heading_gain": k,
            "rob_rad": robot_radius, "wheelbase": wheelbase, "max_speed": max_speed, "max_steer": max_steer,
            "start": [sx, sy], "goal": [gx, gy],
            "grid_size": grid_size, "env": [ox, oy], "env_dense": [oxd, oyd],
            }


def def_env(ds=0.1, dense=False):
    """
    Defines the environment bounds and the obstacles.
    params:
        ds: step size
        dense: if true, ds is divided by 10 (used for improved plotting)
    returns:
        ox, oy: lists of x-y pairs indicating that the position is occupied by an obstacle
    """
    if dense:
        ds /= 10
    # define env bounds
    xmin, xmax = 0, 15
    ymin, ymax = 0, 15
    ox, oy = [], []
    for i in np.linspace(xmin, xmax, round((xmax - xmin) / ds), endpoint=True):  # bottom
        ox.append(i)
        oy.append(ymin)
    for i in np.linspace(ymin, ymax, round((ymax - ymin) / ds), endpoint=True):  # right
        ox.append(xmax)
        oy.append(i)
    for i in np.linspace(xmin, xmax, round((xmax - xmin) / ds), endpoint=True):  # top
        ox.append(i)
        oy.append(ymax)
    for i in np.linspace(ymin, ymax, round((ymax - ymin) / ds), endpoint=True):  # left
        ox.append(xmin)
        oy.append(i)

    def populate_obst(xmin, xmax, ymin, ymax):
        """ defines a rectangular obstacle between the given bounds"""
        for cy in np.linspace(ymin, ymax, round((ymax - ymin) / ds), endpoint=True):
            for cx in np.linspace(xmin, xmax, round((xmax - xmin) / ds), endpoint=True):
                ox.append(cx)
                oy.append(cy)

    # define obstacles: list of [xmin, xmax, ymin, ymax]
    ob_list = [#[3, 4, 0, 10],
               [3, 4, 0, 4],
               [7, 8, 5, 15],
               [8, 12, 5, 6],
               [7, 9, 1, 2],
               [11, 12, 3, 5],
               [11, 15, 9, 10]]
    for ob in ob_list:
        populate_obst(*ob)

    return ox, oy


if __name__ == "__main__":
    params = sim_params()
    plot_env(params["start"], params["goal"], params["env"])
    plot_env(params["start"], params["goal"], params["env_dense"])

