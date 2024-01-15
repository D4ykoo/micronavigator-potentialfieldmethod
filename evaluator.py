import numpy as np


class PathEvaluator:
    """
    This class is designed to evaluate paths based on various criteria such as:
    path length, travel time, and the number of obstacles encountered.
    """
    def __init__(self, occupancy_grid, step_size):
        self.occupancy_grid = occupancy_grid
        self.step_size = step_size

    def evaluate_path(self, path):
        """
        Evaluates the path given and is kind of a wrapper for the evaluation functions
        :param path:
        :return:
        """
        path_length = self.calculate_path_length(path)
        travel_time = self.calculate_travel_time(path)
        num_obstacles = self.count_obstacles_on_path(path)

        return {
            'path_length': path_length,
            'travel_time': travel_time,
            'num_obstacles': num_obstacles
        }

    def calculate_path_length(self, path):
        """
        Calculates the total length of the given path by summing up
        the Euclidean distances between consecutive points in the path.
        :param path: array of the steps the roboter took
        :return: path length
        """
        path_length = 0
        for i in range(1, len(path)):
            path_length += np.linalg.norm(path[i] - path[i - 1])
        return path_length

    def calculate_travel_time(self, path):
        """
        This method estimates the travel time for the given path
        based on the length of the path and the provided step size.
        This estimation is fine since each step takes a specific
        amount of time, it does not recognize dynamic travel time.
        :param path:
        :return:
        """
        print(len(path))
        print(self.step_size)
        return len(path) * self.step_size

    def count_obstacles_on_path(self, path):
        """
        This method counts the number of obstacles encountered along
        the given path by checking the occupancy grid.
        :param path:
        :return:
        """
        num_obstacles = 0
        for position in path:
            if self.occupancy_grid.is_occupied(int(position[0]), int(position[1])):
                num_obstacles += 1
                print(int(position[0]), int(position[1]))
        return num_obstacles
