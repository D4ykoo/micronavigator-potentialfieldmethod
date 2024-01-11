import numpy as np
from matplotlib import pyplot as plt

from config import Configurator
from evaluator import PathEvaluator
from occupancy_grid import OccupancyGrid
from planner import Planner
from plotter import Plotter
from robot import Robot
from scenarios import Scenarios, ScenarioGenerator


def main():
    """
    Main function of the micro navigator using the potential field method.
    Takes care of the navigation, evaluation and plotting, by containing instances
    of the Planner, Plotter, OccupancyGrid and ScenarioGenerator.

    :return: Plots and prints results of the evaluation
    """
    config = Configurator()
    grid_size = [6, 6]

    occupancy_grid = OccupancyGrid(size=grid_size[0])
    og_occupancy_grid = OccupancyGrid(size=grid_size[0])

    planner = Planner(occupancy_grid, og_occupancy_grid, [0.1, 0.1])
    plotter = Plotter(planner)
    scenarios = Scenarios()
    for j, scenario in enumerate(scenarios.scenarios):
        print("Scenario: ", j)
        scenario_gen = ScenarioGenerator(scenario)

        goal_position = scenario_gen.goal_position
        obstacle_positions = scenario_gen.obstacle_positions
        start_position = scenario_gen.start_position
        current_position = start_position
        roboter = Robot(0.1, 0.1, start_position)

        planner.mark_grid_with_obstacles(obstacle_positions)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Potential Field')
        ax.set_title('3D Potential Field Visualization')

        path_fig = plt.figure()
        path_ax = path_fig.add_subplot(111)
        path_ax.set_xlabel('X-axis')
        path_ax.set_ylabel('Y-axis')
        path_ax.set_title('2D Scatter Plot')

        legend_entries = {}

        evaluator = PathEvaluator(planner.occupancy_grid, config.step_size)
        planned_path = [current_position]

        for i in range(config.max_iterations):
            # Plotting in the 2D scatter plot
            path_ax.scatter(current_position[0], current_position[1], color='blue', label='Current Position')
            path_ax.scatter(goal_position[0], goal_position[1], color='green', label='Goal Position')
            for obstacle in obstacle_positions:
                path_ax.scatter(obstacle[0], obstacle[1], color='red', label='Obstacle')

            if np.linalg.norm(current_position - goal_position) < 0.1:
                print("Reached the goal!")
                break

            current_position = planner.navigate(roboter.position,
                                                goal_position,
                                                config)
            roboter_position = current_position
            roboter.set_position(roboter_position)

            planned_path.append(current_position)

        plotter.plot_potential_field(ax, current_position, goal_position, config)  # Show the 2D scatter plot

        # Evaluate the planned path
        evaluation_result = evaluator.evaluate_path(planned_path)

        # gradient_map = planner.visualize_gradient(goal_position, config.att_strength, config.rep_strength,
        # config.safe_radius) Print the results
        print("Occupancy grid: \n", planner.occupancy_grid.grid)
        print("Path Length:", evaluation_result['path_length'])
        print("Travel Time:", evaluation_result['travel_time'])
        print("Number of Obstacles on Path:", evaluation_result['num_obstacles'])

        for color, label in [('blue', 'Current Position'), ('green', 'Goal Position'), ('red', 'Obstacle')]:
            if label not in legend_entries:
                legend_entries[label] = plt.Line2D([0], [0], marker='o', color=color, label=label, linestyle='None')

        legend = list(legend_entries.values())
        path_ax.legend(handles=legend)
        ax.legend()

        # plt.imshow(gradient_map, cmap='viridis', origin='lower')
        # plt.colorbar()
        plt.show()


if __name__ == "__main__":
    main()
