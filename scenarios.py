import numpy as np

scenario1 = {
    'start': (0, 0),
    'goal': (5, 5),
    'obstacles': [(0, 1), (3, 3), (4, 2), (5, 2)]
}

scenario2 = {
    'start': (0, 0),
    'goal': (5, 5),
    'obstacles': [(0, 1), (1, 1), (2, 2), (3, 3)]
}

scenarios = [scenario1, scenario2]


class Scenarios():
    def __init__(self):
        self.scenarios = scenarios


class ScenarioGenerator:
    def __init__(self, scenario):
        self.scenario = scenario

    @property
    def obstacle_positions(self):
        return [np.array(obstacle) for obstacle in self.scenario.get('obstacles', [])]

    @property
    def start_position(self):
        return np.array(self.scenario.get('start', [0, 0]))

    @property
    def goal_position(self):
        return np.array(self.scenario.get('goal', [0, 0]))
