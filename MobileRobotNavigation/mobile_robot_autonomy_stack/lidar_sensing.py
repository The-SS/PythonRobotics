"""
Defines function used to simulate a lidar.
Lidar is assumed to always be aligned with the world frame for simplicity.

All units are in meters.

Author: Sleiman Safaoui (@The-SS)
"""
import numpy as np
eps = 1e-6


def _ray_with_horizontal_line(ray_theta, ys, y):
    if abs(ray_theta) < eps or abs(ray_theta - np.pi) < eps:
        return np.inf
    if np.pi < ray_theta < 2*np.pi and ys < y:
        return np.inf
    if 0 < ray_theta < np.pi and ys > y:
        return np.inf
    return (y-ys) / np.sin(ray_theta)


def _ray_with_vertical_line(ray_theta, xs, x):
    if abs(ray_theta - np.pi/2) < eps or abs(ray_theta - 3*np.pi/2) < eps:
        return np.inf
    if np.pi/2 < ray_theta < 3*np.pi/2 and xs < x:
        return np.inf
    if (3*np.pi/2 < ray_theta or ray_theta < np.pi/2) and xs > x:
        return np.inf
    return (x-xs) / np.cos(ray_theta)


def _polar_to_cart(xs, ys, r, theta, env_bounds):
    xmin, xmax, ymin, ymax = env_bounds
    if r == np.inf:
        if abs(theta) < eps:
            return xmax, ys
        if abs(theta - np.pi/2) < eps:
            return xs, ymax
        if abs(theta - np.pi) < eps:
            return xmin, ys
        if abs(theta - 3*np.pi/2) < eps:
            return xs, ymin
    return r * np.cos(theta) + xs, r * np.sin(theta) + ys


def _ray_to_obstacle(xs, ys, ray_theta, env_bounds, ob_list):
    r_list = [np.inf]
    xmin, xmax, ymin, ymax = env_bounds
    r_list.append(_ray_with_horizontal_line(ray_theta, ys, ymin))
    r_list.append(_ray_with_horizontal_line(ray_theta, ys, ymax))
    r_list.append(_ray_with_vertical_line(ray_theta, xs, xmin))
    r_list.append(_ray_with_vertical_line(ray_theta, xs, xmax))
    for [xmin, xmax, ymin, ymax] in ob_list:
        r = _ray_with_horizontal_line(ray_theta, ys, ymin)
        x, y = _polar_to_cart(xs, ys, r, ray_theta, env_bounds)
        if x < xmin or x > xmax:
            r = np.inf
        r_list.append(r)
        r = _ray_with_horizontal_line(ray_theta, ys, ymax)
        x, y = _polar_to_cart(xs, ys, r, ray_theta, env_bounds)
        if x < xmin or x > xmax:
            r = np.inf
        r_list.append(r)
        r = _ray_with_vertical_line(ray_theta, xs, xmin)
        x, y = _polar_to_cart(xs, ys, r, ray_theta, env_bounds)
        if y < ymin or y > ymax:
            r = np.inf
        r_list.append(r)
        r = _ray_with_vertical_line(ray_theta, xs, xmax)
        x, y = _polar_to_cart(xs, ys, r, ray_theta, env_bounds)
        if y < ymin or y > ymax:
            r = np.inf
        r_list.append(r)
    r = min(r_list)
    x, y = _polar_to_cart(xs, ys, r, ray_theta, env_bounds)
    return r, x, y


def lidar_sensing(xs, ys, env_bounds, ob_list, count):
    ray_theta_list = np.linspace(np.radians(0), 2*np.pi, count, endpoint=False)
    r_list = [0 for _ in ray_theta_list]
    x_list = [0 for _ in ray_theta_list]
    y_list = [0 for _ in ray_theta_list]
    for i in range(len(r_list)):
        r_list[i], x_list[i], y_list[i] = _ray_to_obstacle(xs, ys, ray_theta_list[i], env_bounds, ob_list)
    return r_list, x_list, y_list


