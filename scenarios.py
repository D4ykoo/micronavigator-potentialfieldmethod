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

scenario3 = {
    'start': (0, 4),
    'goal': (4, 3),
    'obstacles': [(0, 1), (1, 1), (2, 2), (3, 3), (3, 4)]
}

scenarios = [scenario1, scenario2, scenario3]


class Scenarios:
    """
    Scenarios class to store a list of scenarios that can be accessed from another class.
    """
    def __init__(self):
        self.scenarios = scenarios


class ScenarioGenerator:
    """
    Scenario generator to generate scenario properties based on a scenario.
    """
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
