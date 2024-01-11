import numpy as np


class OccupancyGrid:
    """
    OccupancyGrid creates on init an empty numpy matrix e.g. size = 5 it will create a 5x5 matrix.
    """

    def __init__(self, size):
        self.grid = np.full((size, size), 0, dtype=np.int64)

    def mark_cell(self, x, y):
        """
        Marks the cell as occupied
        """
        if not self.valid_cell(x, y):
            return
        self.grid[x][y] = 1
        return

    def unmark_cell(self, x, y):
        """
        Marks the cell as unoccupied
        :param x: x position of the cell
        :param y: y position of the cell
        """
        if not self.valid_cell(x, y):
            return
        return self.grid[x][y] == 0

    def is_occupied(self, x, y):
        if not self.valid_cell(x, y):
            return False
        a = self.grid[x][y] == 1
        return a

    """
    valid_cell: checks if the given coordinate is in the grid
    """

    def valid_cell(self, x, y):
        """
        Checks if the given coordinate is in the grid
        :param x: x position of the cell
        :param y: y position of the cell
        :return: True or False
        """
        return 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]
