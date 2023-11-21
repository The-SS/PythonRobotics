"""
Helper functions for the mobile autonomy stack

Author: Sleiman Safaoui (@The-SS)

"""
import numpy as np


def at_goal(gx, gy, x, y, thresh):
    """
    checks if the robot is close enough to the goal
    :param gx: (float) goal x-values
    :param gy: (float) goal y-values
    :param x: (float) robot x-position
    :param y: (float) robot y-position
    :param thresh: (float) threshold for robot being close to the goal
    """
    return np.linalg.norm([x-gx, y-gy]) < thresh


def in_collision(ox, oy, x, y, rad):
    """
    checks if the robot is in collision with and of the obstacles.
    assumes robot is a circle and that the obstacles list is dense enough
    :param ox: ([float]) obstacle x-values
    :param oy: ([float]) obstacle y-values
    :param x: (float) robot x-position
    :param y: (float) robot y-position
    :param rad: (float) robot radius
    """
    for px, py in zip(ox, oy):
        if np.linalg.norm([x-px, y-py]) < rad:
            return True
    return False
