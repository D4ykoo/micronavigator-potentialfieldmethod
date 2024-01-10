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

    def navigate(self, current_position, goal_position, att_strength, rep_strength,
                 safe_radius, step_size, grid_size):
        gradient = self.calculate_gradient(current_position, goal_position, att_strength, rep_strength,
                                           safe_radius)
        new_position = current_position - step_size * gradient

        # Ensure the new position stays within the grid boundaries
        new_position = np.clip(new_position, [0, 0], [grid_size[0], grid_size[1]])

        return new_position

    def plot_potential_field(self, ax, att_strength, rep_strength, safe_radius, obstacles, grid_size, goal_position):
        x_range = np.linspace(0, grid_size[0], 100)
        y_range = np.linspace(0, grid_size[1], 100)
        X, Y = np.meshgrid(x_range, y_range)
        Z = np.zeros_like(X)

        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                current_position = np.array([X[i, j], Y[i, j]])
                Z[i, j] = self.calculate_total_potential(
                    current_position, goal_position, obstacles, att_strength, rep_strength, safe_radius)

        ax.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)


def main():
    att_strength = 1.0
    rep_strength = 10.0
    safe_radius = 2.0
    step_size = 0.1
    max_iterations = 100

    grid_size = [6, 6]
    occupancy_grid = OccupancyGrid(size=grid_size[0])
    planner = Planner(occupancy_grid)

    scenarios = Scenarios()
    for j, scenario in enumerate(scenarios.scenarios):
        print("Scenario: ", j)
        scenario_gen = ScenarioGenerator(scenario)

        goal_position = scenario_gen.goal_position
        obstacle_positions = scenario_gen.obstacle_positions
        start_position = scenario_gen.start_position
        current_position = start_position

        planner.mark_grid_with_obstacles(obstacle_positions)

        print(planner.occupancy_grid.grid)
        print(planner.occupancy_grid.grid.shape)
        print(planner.occupancy_grid.grid.shape[0])
        print(planner.occupancy_grid.grid.shape[1])

        for i in range(max_iterations):
            plt.scatter(current_position[0], current_position[1], color='blue')
            plt.scatter(goal_position[0], goal_position[1], color='green')
            for obstacle in obstacle_positions:
                plt.scatter(obstacle[0], obstacle[1], color='red')

            if np.linalg.norm(current_position - goal_position) < 0.1:
                print("Reached the goal!")
                break

            current_position = planner.navigate(current_position,
                                                goal_position,
                                                att_strength,
                                                rep_strength,
                                                safe_radius,
                                                step_size,
                                                grid_size)

        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Potential Field Method')
        plt.show()


if __name__ == "__main__":
    main()
