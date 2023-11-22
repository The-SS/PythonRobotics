"""
Planning and control stack for a mobile robot using existing algorithms in PythonRobotics.
Planning with A*.

Author: Sleiman Safaoui (@The-SS)

"""
import numpy as np

from settings import sim_params
from plotting import plot_env
from PathPlanning.AStar.a_star import AStarPlanner
import time
from dynamics import StanleyState, StanleyCtrl
import matplotlib.pyplot as plt
from PathPlanning.CubicSpline import cubic_spline_planner
# from PathTracking.stanley_controller.stanley_controller import pid_control, stanley_control
from helpers import in_collision, at_goal
from lidar_sensing import lidar_sensing


t = 0  # simulation time
t_sim = 20  # [sec] simulation time
at_goal_thresh = 0.3

# load simulation parameters
params = sim_params()
x0, y0 = params['start']
yaw0 = np.radians(90)
v0 = 0
wb, max_steer = params['wheelbase'], params['max_steer']
plan_rate, ctrl_rate = params['planning_rate'], params['control_rate']
target_speed = params['max_speed']
rob_rad = params['rob_rad']
plan_dt, ctrl_dt = 1./plan_rate, 1./ctrl_rate
sim_steps = t_sim * ctrl_rate  # total number of simulation steps
Kp, k = params['ctrl_speed_gain'], params['ctrl_heading_gain']

# initialize dynamics and controller
dyn = StanleyState(x=x0, y=y0, yaw=yaw0, v=v0, wb=1, max_steer=np.radians(30.0), dt=ctrl_dt)
ctrlr = StanleyCtrl(dyn, Kp, k)

# initialize autonomy stack variables
plan_x, plan_y = [], []  # motion plan
cx, cy, cyaw = [], [], []  # cubic spline fit of motion plan
target_idx = 0

# history
plan_x_original, plan_y_original = [], []
hist_t = [t]
hist_x = [ctrlr.state.x]
hist_y = [ctrlr.state.y]

for sim_step in range(sim_steps):
    print('----------------------')
    print('t = ', t)

    # perception
    r, ox, oy = lidar_sensing(ctrlr.state.x, ctrlr.state.y, env_bounds=params['env_bounds_list'],
                            ob_list=params['ob_list'], count=360)

    # update planning at
    if sim_step % plan_rate == 0:
        t_plan_start = time.time()
        ox_env, oy_env = params['env_bounds']
        a_star = AStarPlanner(ox + ox_env, oy + oy_env, params['grid_size'], params['rob_rad'], params['animate'])
        plan_x, plan_y = a_star.planning(ctrlr.state.x, ctrlr.state.y, *params['goal'])
        t_plan_end = time.time()
        print('A* time: ', t_plan_end - t_plan_start)

        plot_env(params['start'], params['goal'], params['env_dense'], show=False)
        plt.scatter(ox, oy, color='hotpink', marker='x')
        plt.plot(plan_x, plan_y, "-r")
        plt.show(block=False)

        # find control and move vehicle
        plan_x = plan_x[::-1]
        plan_y = plan_y[::-1]
        if len(plan_x) == 1 or at_goal(*params['goal'], ctrlr.state.x, ctrlr.state.y, at_goal_thresh):
            break
        cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(plan_x, plan_y, ds=ctrl_dt)
        target_idx, _ = ctrlr.calc_target_index(cx, cy)

    # apply control
    a_i = ctrlr.pid_control(target_speed, dyn.v)
    d_i, target_idx = ctrlr.stanley_control(cx, cy, cyaw, target_idx)
    ctrlr.state.update(a_i, d_i)

    # increase sim time
    t += ctrl_dt

    # save data
    hist_t.append(t)
    hist_x.append(ctrlr.state.x)
    hist_y.append(ctrlr.state.y)

    # plot
    plt.cla()
    ax = plt.gca()
    plot_env(params['start'], params['goal'], params['env_dense'], show=False)
    if len(hist_t) == 2:
        plan_x_original, plan_y_original = plan_x, plan_y
    plt.scatter(ox, oy, color='hotpink', marker='x')
    plt.plot(plan_x, plan_y, "-r")
    plt.plot(hist_x, hist_y, "-", color='blue')
    if in_collision(*params['env'], ctrlr.state.x, ctrlr.state.y, rob_rad):
        circ = plt.Circle((ctrlr.state.x, ctrlr.state.y), rob_rad, color='tab:blue', ec='tab:red', lw=2)
    else:
        circ = plt.Circle((ctrlr.state.x, ctrlr.state.y), rob_rad, color='tab:blue', ec='k', lw=2)
    ax.add_patch(circ)
    plt.arrow(ctrlr.state.x, dyn.y, 0.9*rob_rad*np.cos(ctrlr.state.yaw), 0.9*rob_rad*np.sin(ctrlr.state.yaw),
              width=0.3, head_width=0, color='yellow')
    plt.pause(ctrl_dt)


plt.pause(3.0)
plt.close()



