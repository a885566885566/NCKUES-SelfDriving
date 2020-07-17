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
    def __init__(self):
        self.target_set = None
        return

    """
    Create potential path.
    Input: 
        current_state: 1d nparray, in global coordinate 
            A 1d array which contains the current information 
            of the vehicle, like x, y, yaw, velocity.
            format:[x_point, y_point, yaw, velocity]
        goal_waypoint: 1d nparray, in global coordinate
            A state for the vehicle to reach in global coordinate.
        num_of_points: odd number, number of point in target_set
        offset: number, distance between `target_set` points 

    Output:
        path_set
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
                self.target_set[mid_idx+i] = vec_goal
                
        return

    """
    Get the obstacle information and check if the path's validity.
    Input:
        path_set, obstacles
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
        
        path_validity: 1d array of boolen value to classify whether a path is collision-free(True), or not(False). 
    Output:
        score: A 1d array to indicate the score of the specific path,
               a low score implies the suitable path, 
               -1 represents the colliding path's score.
    """
    def make_score(self, path_set, goal_state, path_validity):
        
        score = []

        for i in range(len(path_set)):
            # Handle the case of collision-free path.

            if path_validity[i]:
                # Compute the collision-free path goal point's distance to centerline goal point.
                # A low score implies the suitable path.

                score.append( np.sqrt( (path_set[i][-1][0] - goal_state[0])**2 + (path_set[i][-1][1] - goal_state[1])**2 ) )

                for j in range(len(path_set)):
                    # Compute the collision-free path goal point's distance to colliding path's goal point
                    # then substract it from the score, farther distance to the colliding path implies the suitable path. 
                    if j == i:
                        continue
                    if not path_validity[j]:
                        score[i] -= np.sqrt( (path_set[i][-1][0] - path_set[j][-1][0])**2 + (path_set[i][-1][1] - path_set[j][-1][1])**2 )

            else:
                score.append(-1) 
        return score

    """
    Choose best path according to `make_score` function.
    Output: path, 2d Array
    """
    def choose_best_path(self):
    # TODO: need to find a way to choose a path among the same score path
        return

    def generate_emergency_path(self):
        return

    def generate_velocity_profile(self):
        return 

if __name__ == "__main__":

    mp = MotionPlanner()
    path_set = [ [[20,60]], [[20,55]], [[20,50]], [[20,45]], [[20,40]] ]
    goal_state = [20,50]
    path_validity = [True, False, True, True, True]
    score = mp.make_score(path_set, goal_state, path_validity)

    for i in range(len(score)):
        print( "the", (i+1), "path's score is", score[i])

