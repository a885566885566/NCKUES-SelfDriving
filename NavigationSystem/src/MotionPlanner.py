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
    def generate_path(self, current_state, goal_waypoint, obstacles, num_of_points, offset):
        local_goal = Utils.trans_global_to_local(current_state, goal_waypoint[0:2])
        #local_obs = Utils.trans_global_to_local(current_state, obstacles[0:2])

        # Generate target set
        target_set = self.generate_target_set(current_state, goal_waypoint, num_of_points, offset)

        # Interpolate the path
        path_pts = self.interpolate_path(target_set)

        # Filter out obstacles that are not possible to collide
        #danger_obs = self.obstacles_filter(goal_waypoint, obstacles)

        # Collision check and make score1(Distance from )

        return target_set, path_pts

    """
    Generate `target_set`
    Input:
        current_state:  1d nparray
        goal_waypoint:  1d nparray
        num_of_points:  odd integer, number of target set
        offset:         number, distance between target set
    """
    def generate_target_set(self, current_state, goal_waypoint, num_of_points, offset):
        self.target_set = np.zeros((num_of_points, 4))

        goal_state = np.zeros(4)
        goal_state[0:2] = Utils.trans_global_to_local(current_state, goal_waypoint[0:2])
        goal_state[2] = goal_waypoint[2] - current_state[2]
        goal_state[3] = goal_waypoint[3]

        mid_idx = num_of_points // 2
        vec_delta = offset * np.array((-np.sin(goal_state[2]), np.cos(goal_state[2])))

        target_set = np.zeros((num_of_points, 4))
        target_set[mid_idx] = goal_state 
        for i in range(1, mid_idx+1):
            target_set[mid_idx+i] = goal_state 
            target_set[mid_idx-i] = goal_state 
            target_set[mid_idx+i][0:2] += i*vec_delta 
            target_set[mid_idx-i][0:2] -= i*vec_delta 
               
        return target_set 
    
    """
    Interpolate path with cubic spline in local coordinate.
    Start from (0, 0) end to target_set, second and third 
    derivative were set to zero. Other boundary conditions:
    (0, 0):   f(0) =0,  f'(0) =0
    (x1, y1): f(x1)=y1, f'(x1)=dy/dx=tan(theta)
    S(x) = a*x^3 + b*x^2
        3*y1   tan(t)        tan(t) - 2*b*x1 
    a = ---- - ------   b =  --------------- 
        x1^2     x1               3*x1^2

    Input:
        target_set: 2d nparray
        num_of_interpolate: integer, number of point in each spline
    Return:
        path_points: each point of the path, shape in 
            [numb_of_points, num_of_interpolate, 2]
        a:  2d nparray, [number_of_points]
        b:  2d nparray, [number_of_points]
    """
    def interpolate_path(self, target_set, num_of_interpolate=10):
        #a = np.tan(target_set[:, 2]) / (3 * np.square(target_set[:,0]))
        #b = target_set[:,1] / np.square(target_set[:,0]) - np.tan(target_set[:, 2]) / (3 * target_set[:,0])
        tan_t = np.tan(target_set[:,2])
        x1 = target_set[:,0]
        y1 = target_set[:,1]
        x1_2 = np.square(x1)    # square of x1

        b = 3*y1/x1_2 - tan_t/x1
        a = (tan_t-2*b*x1)/(3*x1_2)
        x = np.linspace(0, target_set[:,0], num_of_interpolate)
        y = a*np.power(x, 3) + b*np.square(x)
        return np.stack((x.T,y.T), axis=-1)

    """
    This function filter out some obstacles that is not posible
    to collide the path zone.
    Input:
        goal: goal_waypoint, in local coord, 1d nparray
        obs: obstacles, in local coord, 2d nparray
    Return:
        obstacles that have possibility to collide.
    """
    def obstacles_filter(self, goal, obs):
        vec = obs - goal 
        # The discriminant
        if goal[1] == 0:
            D = obs[:, 0] - goal[0]
        else:
            # The line intersect to goal and perpendicular to its 
            # vector to origin.
            D = vec[1] + vec[0] * goal[0] / goal[1]
            D *= (1 if goal[1]>0 else -1)
        return obs[D<0]

    """
    Get the information of obstacles and check if the path's validity.
    Input:
        path_set, obstacles
    Ouput:
        path_validity: 1d array of boolen value to classify whether a path 
        is collision-free(True), or not(False). 
    """
    def collision_checker(self, paths, obstacles):
        suite = goal[goal[0]>0]     # the x axis must larger than 0
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

        target_set, path_pts = mp.generate_path(current_state, current_goal_waypoint, None, 11, 2)

        sim.scatter_with_local(current_state, target_set[:,0:2], '.', 'red')
        for path in path_pts:
            sim.plot_with_local(current_state, path, 'g-')

        current_state = current_goal_waypoint 
        plt.pause(0.01)

    """        

    mp = MotionPlanner()
    path_set = [ [[20,60]], [[20,55]], [[20,50]], [[20,45]], [[20,40]] ]
    goal_state = [20,50]
    path_validity = [True, False, True, True, True]
    score = mp.make_score(path_set, goal_state, path_validity)

    for i in range(len(score)):
        print( "the", (i+1), "path's score is", score[i])

    """
