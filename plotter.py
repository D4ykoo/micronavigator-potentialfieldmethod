import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Plotter:
    """
    Class for plotting the 3D potential field
    """
    def __init__(self, planner):
        self.fig = plt.figure()
        self.planner = planner
        self.ax = self.fig.add_subplot(111, projection='3d')

    def plot_potential_field(self, ax, current_position, goal_position, config):
        """
        Plots the potential field as a surface plot. It takes the current position,
        goal position, and a configuration object as input.
        """
        x_range = np.linspace(0, config.grid_size[0], 100)
        y_range = np.linspace(0, config.grid_size[1], 100)
        X, Y = np.meshgrid(x_range, y_range)
        Z = np.zeros_like(X)

        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                position = np.array([X[i, j], Y[i, j]])
                Z[i, j] = self.planner.calculate_total_potential(
                    position, goal_position, config.att_strength, config.rep_strength, config.safe_radius)

        ax.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)
        ax.scatter(current_position[0], current_position[1], self.planner.calculate_total_potential(
            current_position, goal_position, config.att_strength, config.rep_strength, config.safe_radius),
                   color='blue', marker='o', label='Current Position')
        ax.scatter(goal_position[0], goal_position[1],
                   self.planner.calculate_total_potential(goal_position, goal_position, config.att_strength,
                                                          config.rep_strength, config.safe_radius),
                   color='green', marker='o', label='Goal Position')
        self.ax.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)