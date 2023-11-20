"""
Defines the settings for the plotting functions

Author: Sleiman Safaoui (@The-SS)
"""

import matplotlib.pyplot as plt


def plot_env(start, goal, env, show=True):
    plt.plot(*env, ".k")
    plt.plot(*start, "og")
    plt.plot(*goal, "xb")
    plt.grid(True)
    plt.axis("equal")
    if show:
        plt.show()
