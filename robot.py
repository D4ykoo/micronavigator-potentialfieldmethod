class Robot:
    def __init__(self, length, width, start_position):
        self.length = length
        self.width = width
        self.position = start_position

    def move(self, x, y):
        self.position += (x, y)

    def set_position(self, x, y):
        self.position = (x, y)
