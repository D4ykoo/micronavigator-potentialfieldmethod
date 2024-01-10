import numpy as np
import matplotlib.pyplot as plt

from occupancy_grid import OccupancyGrid
from scenarios import ScenarioGenerator, Scenarios


def repulsive_potential(current_position, obstacle_position, strength, safe_radius):
    distance = np.linalg.norm(current_position - obstacle_position)
    if distance < safe_radius:
        return 0.5 * strength * (1 / (distance + 1.0) - 1 / safe_radius) ** 2
    else:
        return 0


def attractive_potential(current_position, goal_position, strength):
    return 0.5 * strength * np.linalg.norm(current_position - goal_position) ** 2


class Planner:
    def __init__(self, grid):
        self.occupancy_grid = grid

    def mark_grid_with_obstacles(self, obstacles):
        for obstacle in obstacles:
            self.occupancy_grid.mark_cell(*obstacle)

    def sum_occupancies(self, current_position, rep_strength, safe_radius):
        occupancy_grid = self.occupancy_grid
        total_repulsive_potential = 0
        for i in range(occupancy_grid.grid.shape[0]):
            for j in range(occupancy_grid.grid.shape[1]):
                if occupancy_grid.is_occupied(i, j):
                    total_repulsive_potential += repulsive_potential(current_position, np.array([i, j]), rep_strength,
                                                                     safe_radius)

        return total_repulsive_potential

    def calculate_total_potential(self, current_position, goal_position, att_strength, rep_strength,
                                  safe_radius):
        att_potential = attractive_potential(current_position, goal_position, att_strength)
        rep_potential = self.sum_occupancies(current_position, rep_strength, safe_radius)
        total_potential = att_potential + rep_potential
        return total_potential

    def calculate_gradient(self, current_position, goal_position, att_strength, rep_strength, safe_radius):
        delta = 1e-6
        gradient = np.zeros_like(current_position, dtype=float)

        for i in range(len(current_position)):
            temp_pos = current_position.copy()
            temp_pos[i] += delta
            potential_plus = self.calculate_total_potential(temp_pos, goal_position, att_strength,
                                                            rep_strength,
                                                            safe_radius)

            temp_pos[i] -= 2 * delta
            potential_minus = self.calculate_total_potential(temp_pos, goal_position, att_strength,
                                                             rep_strength,
                                                             safe_radius)

            gradient[i] = (potential_plus - potential_minus) / (2 * delta)

        return gradient

    def move_towards_goal(self, current_position, goal_position, att_strength, rep_strength, safe_radius,
                          step_size):
        gradient = self.calculate_gradient(current_position, goal_position, att_strength, rep_strength,
                                           safe_radius)
        new_position = current_position - step_size * gradient
        return new_position

    def navigate(self, current_position, goal_position, config):
        gradient = self.calculate_gradient(current_position, goal_position, config.att_strength, config.rep_strength,
                                           config.safe_radius)
        new_position = current_position - config.step_size * gradient

        # Ensure the new position stays within the grid boundaries
        new_position = np.clip(new_position, [0, 0], [config.grid_size[0], config.grid_size[1]])

        return new_position
