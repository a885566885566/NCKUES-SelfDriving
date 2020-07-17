import numpy as np
import Utils
import WaypointsPlanner as wp 
import Perception
"""
This class create some potential path, called TargetSet to imitate 
future motion, and determine the best path with some scoring function.
"""
class MotionPlanner():
    """
    This class require `WaypointPlanner` object to obtain
    waypoint information.
    """
    def __init__(self):
        #self.waypoints = waypoints
        self.target_set = None
        return


    """
    Create potential path.
    Input: 
        current_state: 1d nparray, which contains the current information 
            of the vehicle in global coordinate, 
            format:[x_point, y_point, yaw, velocity]

        goal_waypoint: 1d nparray, in global coordinate
            A state for the vehicle to reach in global coordinate.

        num_of_points: odd number, number of point in target_set

        offset: number, distance between `target_set` points 

    Output:
        target_set
        path: 2d Array in local coordinate.
    """
    def generate_path(self, current_state, goal_waypoint, num_of_points, offset):
        target_set = self.generate_target_set(current_state, num_of_points, offset)
        pass

    """
    Generate `target_set`
    """
    def generate_target_set(self, current_state, goal_waypoint, num_of_points, offset):
        self.target_set = np.zeros((num_of_points, 4))

        goal_state = np.zeros(4)
        goal_state[0:2] = Utils.trans_global_to_local(current_state, goal_waypoint[0:2])
        goal_state[2] = goal_waypoint[2] - current_state[2]
        goal_state[3] = goal_waypoint[3]

        mid_idx = num_of_points // 2
        vec_delta = offset * np.array((goal_state[1], goal_state[0])) / np.linalg.norm(goal_state[0:2])

        self.target_set[mid_idx] = goal_state 
        for i in range(1, mid_idx+1):
            self.target_set[mid_idx+i] = goal_state 
            self.target_set[mid_idx-i] = goal_state 
            self.target_set[mid_idx+i][0:2] += i*vec_delta 
            self.target_set[mid_idx-i][0:2] -= i*vec_delta 
                
        return self.target_set 
    
    """
    Interpolate path with target set in local coordinate.
    """
    def interpolate_path(self, target_set):
        return

    """
    Get the obstacle information and check if the path's validity.
    Input:
        paths, obstacles
    Ouput:
        path_validity: 1d array of boolen value to classify whether a path 
        is collision-free(True), or not(False). 
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

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import WaypointsPlanner as wp
    import Simulation as si 

    way = wp.WaypointsPlanner()
    mp = MotionPlanner()

    sim = si.Simulation(way, 10)

    current_state = np.array([0, 100, 1, 10])
    while way.available():
        plt.cla()
        sim.plot_ways()
        current_goal_waypoint = way.load_waypoint(current_state) 
        sim.plot_vehicle(current_state)

        target_set = mp.generate_target_set(current_state, current_goal_waypoint, 11, 10)

        sim.scatter_with_local(current_state, target_set[:,0:2], 'o', 'red')

        current_state = current_goal_waypoint 
        plt.pause(0.1)

        
