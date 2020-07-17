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
    Input:
        current_goal_waypoint: A state for the vehicle to reach in global coordinate.
        current_state: A 1d array which contains the current information of the vehicle, like x, y, yaw, velocity.
            format:[x_point, y_point, yaw, velocity]
    Output:
        path_set
        path: 2d Array in local coordinate.
    """
    def generate_path(self):
        return

    """
    Get the obstacle information and check if the path's validity.
    Input:
        paths, obstacles
    Ouput:
        path_validity: 1d array of boolen value to classify whether a path is collision-free(True), or not(False). 
    """
    def collision_checker(self, paths, obstacles):
        return
    
    """
    Make score to a specific path.
    Input:
        path_set: A list of path in the local coordinate.
        path: A 2d nparray in local coordinate, Content is a list of points.
            format: [[x_points, y_point, t, velocity]
                     [                              ]
                     ................................]
        goal_state: current goal waypoint for the vehicle to reach in local coordinate.
            format: [x_goal, y_goal, t, v_goal]
        
    Output:
        score: A 1d array to indicate the score of the specific path. 
    """
    def make_score(self, path_set, current_goal_waypoint):
        
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

