"""
This class create some potential path, called TargetSet
to imitate future motion, and determine the best path 
with some scoring function.
"""
class MotionPlanner():
    """
    This class require `WaypointPlanner` object to obtain
    waypoint information.
    """
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.target_set = None
        return

    """
    Create potential path.
    Output:
        path: 2d Array in local coordinate.
    """
    def generate_path(self):
        return

    """
    Make score to a specific path.
    Output: 
        score, number
    """
    def make_score(self, path, goal_state):
        return

    """
    Choose best path according to `make_score` function.
    Output: path, 2d Array
    """
    def choose_best_path(self):
        return

    def generate_emergency_path(self):
        return

    def generate_velocity_profile(self):
        return 

