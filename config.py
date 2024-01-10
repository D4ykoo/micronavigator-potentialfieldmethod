class Configurator:
    """
    Configurator class
    Holds a default config for the potential field navigation
    The properties can be updated.

    Example usage:
    config = Configurator()
    config.update_config("step_size=0.5")
    """
    def __init__(self):
        self.att_strength = 1.0
        self.rep_strength = 10.0
        self.safe_radius = 2.0
        self.step_size = 0.1
        self.max_iterations = 100
        self.grid_size = [6, 6]

    def update_config(self, att_strength=None, rep_strength=None, safe_radius=None, step_size=None, max_iterations=None, grid_size=None):
        # Update the configuration with new values if provided
        if att_strength is not None:
            self.att_strength = att_strength
        if rep_strength is not None:
            self.rep_strength = rep_strength
        if safe_radius is not None:
            self.safe_radius = safe_radius
        if step_size is not None:
            self.step_size = step_size
        if max_iterations is not None:
            self.max_iterations = max_iterations
        if grid_size is not None:
            self.grid_size = grid_size