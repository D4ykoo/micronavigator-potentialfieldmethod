import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from planner import Planner


class Plotter:
    def __init__(self, att_strength, rep_strength, radius, obstacles, grid_size, goal_position):
        self.att_strength = att_strength
        self.rep_strength = rep_strength
        self.radius = radius
        self.obstacles = obstacles
        self.grid_size = grid_size
        self.goal_position = goal_position
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def plot_potential_field(self):
        x_range = np.linspace(0, self.grid_size[0], 100)
        y_range = np.linspace(0, self.grid_size[1], 100)
        X, Y = np.meshgrid(x_range, y_range)
        Z = np.zeros_like(X)

        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                current_position = np.array([X[i, j], Y[i, j]])
                Z[i, j] = Planner.calculate_total_potential(
                    current_position=current_position,
                    goal_position=self.goal_position,
                    obstacles=self.obstacles,
                    att_strength=self.att_strength,
                    rep_strength=self.rep_strength,
                    safe_radius=self.radius)

        self.ax.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)

    def show_plot(self):
        self.ax.set_xlabel('X-axis')
        self.ax.set_ylabel('Y-axis')
        self.ax.set_zlabel('Potential Field')
        self.ax.set_title('3D Potential Field Visualization')
        self.ax.legend()
        plt.show()
