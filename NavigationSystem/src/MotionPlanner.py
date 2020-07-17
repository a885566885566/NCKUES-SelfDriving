import numpy as np
import WaypointsPlanner as wp 
import Perception
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
        current_state: 4d nparray, in global coordinate 
        goal_waypoint: 4d nparray, in global coordinate
        num_of_points: odd number, number of point in target_set
        offset: number, distance between `target_set` points 
    Output:
        path: 2d Array in local coordinate.
    """
    def generate_path(self, current_state, goal_waypoint, num_of_points, offset):
        vec_goal = goal_waypoint - current_state 
        vec_del = np.array([vec_goal[0]+offset*np.cos(vec_goal[2]+np.pi/2),
                            vec_goal[1]+offset*np.sin(vec_goal[2]+np.pi/2),
                            0,
                            goal_waypoint[3]])
        self.target_set = np.zeros(num_of_points, 4)
        mid_idx = num_of_points // 2
        for i in range(mid_idx):
            if i == 0:
                self.target_set[mid_idx] = vec_goal
            else:
                self.target_set[mid_idx+i] = vec_goal + 
                
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

