import numpy as np

class Robot:
    def __init__(self, length, width, start_position):
        self.length = length
        self.width = width
        self.position = np.array(start_position, dtype=float)

    def move(self, delta):
        # Update the position by adding the delta array
        self.position = delta

    def set_position(self, new_position):
        # Set the new position using the provided array
        self.position = np.array(new_position, dtype=float)