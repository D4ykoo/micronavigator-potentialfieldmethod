import numpy as np
import matplotlib.pyplot as plt

from occupancy_grid import OccupancyGrid
from scenarios import ScenarioGenerator, Scenarios


def repulsive_potential(current_position, obstacle_position, strength, safe_radius, robot_size):
    distance = np.linalg.norm(current_position - obstacle_position)
    if distance < (safe_radius + 0.5 * (robot_size[0] + robot_size[1])):
        return 0.5 * strength * (1 / (distance + 1.0) - 1 / (safe_radius + 0.5 * (robot_size[0] + robot_size[1]))) ** 2
    else:
        return 0


def attractive_potential(current_position, goal_position, strength):
    return 0.5 * strength * np.linalg.norm(current_position - goal_position) ** 2


class Planner:
    def __init__(self, grid, occupancy_grid, roboter_size):
        self.occupancy_grid = grid
        self.og_occupancy_grid = occupancy_grid
        self.roboter_size = roboter_size

    def is_valid_move(self, new_position):
        """
        TODO
        Check if the new position is a valid move, i.e., not occupied by an obstacle
        and within the grid boundaries.
        """
        x, y = round(new_position[0]), round(new_position[1])

        #return not self.og_occupancy_grid.is_occupied(x, y) and self.og_occupancy_grid.valid_cell(x, y)
        return True

    def mark_grid_with_obstacles(self, obstacles):
        for obstacle in obstacles:
            self.occupancy_grid.mark_cell(*obstacle)
            self.og_occupancy_grid.mark_cell(*obstacle)

    def sum_occupancies(self, current_position, rep_strength, safe_radius):
        occupancy_grid = self.occupancy_grid
        total_repulsive_potential = 0
        for i in range(occupancy_grid.grid.shape[0]):
            for j in range(occupancy_grid.grid.shape[1]):
                if occupancy_grid.is_occupied(i, j):
                    total_repulsive_potential += repulsive_potential(current_position,
                                                                     np.array([i, j]),
                                                                     rep_strength,
                                                                     safe_radius,
                                                                     self.roboter_size)

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
        # Normalize the gradient to ensure it has a consistent scale
        normalized_gradient = gradient / np.linalg.norm(gradient)

        # Use the normalized gradient to determine the change in position
        delta_x = normalized_gradient[0] * step_size
        delta_y = normalized_gradient[1] * step_size

        new_position = current_position - np.array([delta_x, delta_y])

        return new_position

    def navigate(self, current_position, goal_position, config):
        gradient = self.calculate_gradient(current_position, goal_position, config.att_strength, config.rep_strength,
                                           config.safe_radius)
        new_position = current_position - config.step_size * gradient

        # Ensure the new position stays within the grid boundaries
        new_position = np.clip(new_position, [0, 0], [config.grid_size[0], config.grid_size[1]])

        if not self.is_valid_move(new_position):
            # If the new position is invalid, find a valid move
            for i in range(1, int(config.grid_size[0] / 2)):
                offset = i * np.sign(gradient)
                candidate_position = new_position + offset
                if self.is_valid_move(candidate_position):
                    new_position = candidate_position
                    break

        return new_position