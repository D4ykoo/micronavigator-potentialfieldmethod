import numpy as np

class Robot:
    """
    Robot class, the robot can be moved around.
    """
    def __init__(self, length, width, start_position):
        self.length = length
        self.width = width
        self.position = np.array(start_position, dtype=float)

    def move(self, delta):
        self.position = delta

    def set_position(self, new_position):
        self.position = np.array(new_position, dtype=float)