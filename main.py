import numpy as np
from matplotlib import pyplot as plt

from config import Configurator
from occupancy_grid import OccupancyGrid
from planner import Planner
from scenarios import Scenarios, ScenarioGenerator


def main():
    config = Configurator()
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
                                                config)

        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Potential Field Method')
        plt.show()


if __name__ == "__main__":
    main()