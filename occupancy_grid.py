import numpy as np


class OccupancyGrid:
    """
    OccupancyGrid creates on init an empty numpy matrix e.g. size = 5 it will create a 5x5 matrix.
    """

    def __init__(self, size):
        self.grid = np.full((size, size), 0, dtype=np.int64)

    """
    mark_cell: marks the cell as occupied
    """

    def mark_cell(self, x, y):
        if not self._valid_cell(x, y):
            return
        self.grid[x][y] = 1
        return

    """
    unmark_cell: marks the cell as unoccupied
    """

    def unmark_cell(self, x, y):
        if not self._valid_cell(x, y):
            return
        return self.grid[x][y] == 0

    def is_occupied(self, x, y):
        if not self._valid_cell(x, y):
            return False
        a = self.grid[x][y] == 1
        return a

    """
    valid_cell: checks if the given coordinate is in the grid
    """

    def _valid_cell(self, x, y):
        return 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]
