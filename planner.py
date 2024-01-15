import numpy as np


def repulsive_potential(current_position, obstacle_position, strength, safe_radius, robot_size):
    distance = np.linalg.norm(current_position - obstacle_position)

    """
    Calculates the repulsive potential between the current position and an obstacle.
    :return: The repulsive potential which is either calculated or 0 when the distance
             is less than the safe radius + robot size
    """

    if distance < (safe_radius + 0.5 * (robot_size[0] + robot_size[1])):
        epsilon = 0.5
        return 0.5 * strength * ((1 / (distance + epsilon)) - (1 / (safe_radius + epsilon))) ** 2
    else:
        return 0


def attractive_potential(current_position, goal_position, strength):
    """
     Calculates the attractive potential between the current position and the goal position.
    """
    return 0.5 * strength * np.linalg.norm(current_position - goal_position) ** 2


class Planner:
    def __init__(self, grid, occupancy_grid, roboter_size):
        self.occupancy_grid = grid
        self.og_occupancy_grid = occupancy_grid
        self.roboter_size = roboter_size

    def is_valid_move(self, new_position):
        """
        Checks if the current position is valid for.
        Not needed since the safe radius provides the roboter from collision with an obstacle.

        :return:
        """
        return True

    def mark_grid_with_obstacles(self, obstacles):
        """
        Marks the occupancy grid with obstacles at specified positions.
        :param obstacles: array of obstacle position
        :return: self.occupancy_grid with "1" as obstacle entry
        """
        for obstacle in obstacles:
            self.occupancy_grid.mark_cell(*obstacle)
            self.og_occupancy_grid.mark_cell(*obstacle)

    def sum_occupancies(self, current_position, rep_strength, safe_radius):
        """
        Calculates the total repulsive potential based on the occupied cells in the occupancy grid.
        :return: total repulsive potential of the obstacles
        """
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
        """
        Calculates the total potential at the current position by summing the attractive and repulsive potentials
        :return: total_potential = attractive_potential + repulsive_potential
        """
        att_potential = attractive_potential(current_position, goal_position, att_strength)
        rep_potential = self.sum_occupancies(current_position, rep_strength, safe_radius)
        total_potential = att_potential + rep_potential
        return total_potential

    def calculate_gradient(self, current_position, goal_position, att_strength, rep_strength, safe_radius):
        """
        Calculates the gradient of the total potential at the current position.
        :return gradient
        """
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

    def navigate(self, current_position, goal_position, config):
        """
        Performs the navigation algorithm by calculating the gradient, updating the position, and handling invalid moves

        :return: the new position of the robot
        """
        gradient = self.calculate_gradient(current_position, goal_position, config.att_strength, config.rep_strength,
                                           config.safe_radius)
        new_position = current_position - config.step_size * gradient

        # Ensure the new position stays within the grid boundaries
        new_position = np.clip(new_position, [0, 0], [config.grid_size[0], config.grid_size[1]])

        if not self.is_valid_move(new_position):
            # If the new position is invalid, find a valid move
            for i in range(1, int(config.grid_size[0] / 2)):
                offset = i * np.sign(new_position - current_position)
                candidate_position = current_position + offset
                if self.is_valid_move(candidate_position):
                    new_position = candidate_position
                    break

        return new_position

    def visualize_gradient(self, goal_position, att_strength, rep_strength, safe_radius):
        """
         Generates a 2D gradient map showing the magnitude of the gradient at each cell in the occupancy grid.

        :return: np array of the gradient
        """
        gradient_map = np.zeros_like(self.occupancy_grid.grid, dtype=float)

        for i in range(self.occupancy_grid.grid.shape[0]):
            for j in range(self.occupancy_grid.grid.shape[1]):
                current_position = np.array([i, j])
                gradient = self.calculate_gradient(current_position, goal_position, att_strength, rep_strength,
                                                   safe_radius)
                gradient_magnitude = np.linalg.norm(gradient)
                gradient_map[i, j] = gradient_magnitude

        return gradient_map
