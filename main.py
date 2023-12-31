import numpy as np

from occupancy_grid import OccupancyGrid
from planner import Planner
from robot import Robot

_GRID_SIZE = 5

if __name__ == '__main__':
    grid = OccupancyGrid(_GRID_SIZE)
    robot = Robot(1, 1, (0, 0))

    planner = Planner(robot, grid)

    scenario1 = {
        'start': (0, 0),
        'end': (4, 4),
        'obstacles': []
    }

    scenario2 = {
        'start': (0, 0),
        'end': (4, 4),
        'obstacles': [(0, 1), (1, 1), (2, 2), (3, 3)]
    }

    scenario3 = {
        'start': (0, 0),
        'end': (2, 2),
        'obstacles': []
    }
    scenarios = [scenario2]

    for index, scenario in enumerate(scenarios):
        start, end, obstacles = scenario['start'], scenario['end'], scenario['obstacles']

        for obstacle in obstacles:
            planner.grid.mark_cell(*obstacle)

        # print(planner.grid.grid)
        path = planner.navigate(start, end, obstacles)

        evaluation = planner.evaluate([scenario])

        for scenario_eval in evaluation:
            print(f"Scenario ID: {scenario_eval['Scenario_id']}")
            print(f"Path length: {scenario_eval['path_length']}")
            print(f"Travel time: {scenario_eval['travel_time']}")
            print(f"Obstacles encountered: {scenario_eval['obstacles']}")

        planner.visualize(grid, path)
        planner.grid = OccupancyGrid(_GRID_SIZE)
        print(planner.potential_field)