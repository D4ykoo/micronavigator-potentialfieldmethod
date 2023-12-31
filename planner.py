from builtins import range

import numpy as np
import matplotlib.pyplot as plt

from occupancy_grid import OccupancyGrid


class Planner:
    def __init__(self, robot, grid):
        self.robot = robot
        self.grid = grid
        self.potential_field = np.zeros_like(self.grid.grid)

    """ 
    If a cell is marked, so it is a obstacle, the potential field is set to a high number 
    """

    def potential_field_method(self, start, goal, obstacles, alpha=0.1, iterations=1000):
        start_x, start_y = start
        goal_x, goal_y = goal

        # create 2D matrix for the arbitrary potential field
        # self.potential_field = np.zeros_like(self.grid.grid.shape)

        # set the start potential to a high value
        self.potential_field[start_x][start_y] = 100

        # gradient descent method
        for i in range(iterations):
            # get gradient in x and y direction
            gradient_x = -2.0 * (start_x - goal_x)
            gradient_y = -2.0 * (start_y - goal_y)

            # update coordinate
            start_x -= alpha * gradient_x
            start_y -= alpha * gradient_y

            # check boundaries for coordinates
            start_x = np.clip(start_x, 0, self.grid.grid.shape[1] - 1)
            start_y = np.clip(start_y, 0, self.grid.grid.shape[0] - 1)

        # set the potential field by normalizing it
        self.potential_field = self.potential_field / np.max(self.potential_field)

        # add repulsions for the navigation -> optimizes the pathfinding
        for obstacle in obstacles:
            self._add_repulsion(obstacle)

    def _add_repulsion(self, obstacle):
        for x in range(self.grid.grid.shape[0]):
            for y in range(self.grid.grid.shape[1]):
                dist = np.sqrt((x - obstacle[0]) ** 2 + (y - obstacle[1]) ** 2)
                if dist <= 1.0:  # may need to change
                    self.potential_field[x][y] = 1e10

    """
    Navigates the roboter through the map, only uses the neighbour cells which are no obstacles
    If all cells are marked as obstacles, the roboter stays on position.
    
    This means the roboter navigates around the obstacles and has a far more efficient or better say optimized
    navigation than just having a look if the cell is blocked (in fact marked with a 1).
    """

    def navigate(self, start, goal, obstacles):
        self.potential_field_method(start, goal, obstacles)
        path = [start]
        current_pos = np.array(start)

        while not np.array_equal(current_pos, goal):
            next_step = self._get_next_step(current_pos)
            if self._calculate_potential(next_step, current_pos) > 0:
                break
            else:
                path.append(tuple(next_step))
        return path

    def _get_next_step(self, current_pos):
        all_neighbours = self._get_neighbours(current_pos)

        free_neighbours = [x for x in all_neighbours if not self.grid.is_occupied(x[0], x[1])]
        if not free_neighbours:
            return current_pos

        next_step_potentials = [self._calculate_potential(x, current_pos) for x in free_neighbours]

        return free_neighbours[np.argmin(next_step_potentials)]

    def _get_neighbours(self, current_pos):
        neighbours = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx != 0 or dy != 0:  # Exclude current_pos itself
                    new_pos = current_pos + np.array([dx, dy])
                    if self._valid_coordinates(new_pos):
                        neighbours.append(new_pos)
        return neighbours

    @staticmethod
    def _calculate_potential(current, goal):
        return np.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)

    def _valid_coordinates(self, pos):
        return 0 <= pos[0] < self.grid.grid.shape[1] and 0 <= pos[1] < self.grid.grid.shape[0]

    def visualize(self, grid, path):
        obstacles_x = []
        obstacles_y = []
        print(grid.grid)
        for i in range(grid.grid.shape[0]):
            for j in range(grid.grid.shape[1]):
                if grid.grid[i][j] == 1:
                    obstacles_x.append(i)
                    obstacles_y.append(j)

        plt.figure()
        print(path)
        print(obstacles_x)
        plt.scatter(obstacles_x, obstacles_y, color="red", label="Obstacle")
       # plt.plot(path[:, 1], path[:, 0], color="blue", label="Path")
      #  plt.scatter(path[0, 1], path[0, 0], color="yellow", label="Startpoint")
       # plt.scatter(path[-1, 1], path[-1, 0], color="green", label="Endpoint")

        plt.imshow(self.grid.grid)
        plt.legend()
        plt.gca().invert_yaxis()
        plt.show()

    """
    evaluates:
    path length
    travel time 
    obstacle count
    """

    def plan_stats(self, path):
        stats = {}
        stats['path_length'] = len(path)

        # calculate travel time by simply multiplying the path length by the time one-step takes
        # for simplicity, let's assume one-step takes 1 unit of time
        stats['travel_time'] = stats['path_length']

        # count the number of obstacles on the path
        stats['obstacles'] = sum([self.grid.is_occupied(x, y) for (x, y) in path])

        return stats

    def evaluate(self, scenarios):
        evaluations = []
        for scenario_id, scenario in enumerate(scenarios):
            start, end, obstacles = scenario['start'], scenario['end'], scenario['obstacles']
            self.grid = OccupancyGrid(self.grid.grid.shape[0])
            for obstacle in obstacles:
                self.grid.mark_cell(*obstacle)

            path = self.navigate(start, end, obstacles)
            stats = self.plan_stats(path)
            stats['Scenario_id'] = scenario_id
            evaluations.append(stats)
            print(scenario)
        return evaluations
