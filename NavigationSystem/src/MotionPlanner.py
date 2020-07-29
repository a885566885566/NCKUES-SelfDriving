import numpy as np
import Utils
import WaypointsPlanner as wp 
#import Perception
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

        goal_waypoint: 1d nparray, a state which choose from waypoints for the 
                       vehicle to reach in global coordinate.
            format:[x_point, y_point, yaw, velocity]

        num_of_points: odd number, number of point in target_set

        offset: number, distance between `target_set` points
        
        obstacles: 2d nparray, obstacles' x and y value in local coordinate.
            format: [[x, y]
                     [x, y]...]

    Output:
        target_set: 2d Array in local coordinate.
            format: [[x, y, radian, velocity]
                     [x, y, radian, velocity]......]
            
        path_pts: 3d Array in local coordinate.
    """
    def generate_path(self, current_state, goal_waypoint, obstacles, num_of_points, offset):
        local_goal = Utils.trans_global_to_local(current_state, goal_waypoint[0:2])
        
        # TODO: decide delete this one or not, because the obstacles nparray from zed2 is actual in local coordinate. 
        local_obs = Utils.trans_global_to_local(current_state, obstacles[:, 0:2])

        # Generate target set
        target_set, slope = self.generate_target_set(current_state, goal_waypoint, num_of_points, offset)

        # TODO: the path_pts only have x and y value now, should give radian and velocity in each point.
        # Interpolate the path
        path_pts = self.interpolate_path(target_set)

        # TODO: change the imput parameter "local_obs" to "obstacles[:, 0:2]".
        # Filter out obstacles that are not possible to collide
        danger_obs = self.obstacles_filter(local_goal, local_obs, slope)
        #print(danger_obs)

        # Collision check and make score1( Distance from )

        return target_set, path_pts, danger_obs

    """
    Generate `target_set`
    Input:
        current_state:  1d nparray
        goal_waypoint:  1d nparray
        num_of_points:  odd integer, number of target set
        offset:         number, distance between target set
    Output:
        target_set: 2d nparray
        slope: number, would be used in obstacles filter 
    """
    def generate_target_set(self, current_state, goal_waypoint, num_of_points, offset):
        self.target_set = np.zeros((num_of_points, 4))

        goal_state = np.zeros(4)
        goal_state[0:2] = Utils.trans_global_to_local(current_state, goal_waypoint[0:2])
        goal_state[2] = goal_waypoint[2] - current_state[2]
        goal_state[3] = goal_waypoint[3]

        mid_idx = num_of_points // 2
        # perpendicular line of target set
        vec_delta = offset * np.array((-np.sin(goal_state[2]), np.cos(goal_state[2])))
        target_set = np.zeros((num_of_points, 4))
        target_set[mid_idx] = goal_state 
        for i in range(1, mid_idx+1):
            # generate target set besides the goal waypoint
            target_set[mid_idx+i] = goal_state 
            target_set[mid_idx-i] = goal_state 
            target_set[mid_idx+i][0:2] += i*vec_delta 
            target_set[mid_idx-i][0:2] -= i*vec_delta 
        
        if goal_state[2] <= 0.001:
            slope = 99999
        else:
            slope = np.cos(goal_state[2])/np.sin(goal_state[2])
        return target_set, slope
    
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
    Output:
        path_pts: each point of the path, shape in 
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
        goal(goal_state): goal_waypoint, in local coord, 1d nparray
        obs: obstacles, in local coord, 2d nparray
        slope: target_set line slope, return from 
            generate_target_set function
    Output:
        obstacles that have possibility to collide.
    """
    def obstacles_filter(self, goal, obs, slope):
        # Filter out those behind the car
        obs = obs[obs[:,0]>0]

        # Filter out those in front of car
        # The discriminant
        if slope >= 99999:  # Almost verticle
            D = obs[:, 0] - goal[0]
        else:
            # The line intersect to goal and perpendicular to its 
            # vector to origin. D = (y-y_g) + m*(x-x_g)
            D = obs[:,1] + slope * obs[:,0] 
            D *= (1 if goal[1]>0 else -1)
        return obs[D<0]

    """
    Get the information of obstacles and check if the path's validity.
    Input:
        path_pts: A list of path in the local coordinate.
            format: [path, path, path, path, path, ......]
        
        path: A 2d nparray in local coordinate, Content is a list of points.
            format: [[x_points, y_point, t, velocity]
                     [x_points, y_point, t, velocity]......] 
		
		obstacles: 2d nparray, stores the obstacle's position in local coordinate 
		           axis, its safe radius, and the current time.
			format: [[x, y, radius, time]
			         [x, y, radius, time]......]
   Ouput:
        path_validity: 1d array of boolen value to classify whether a path 
                       is collision-free(True), or not(False). 
    """
    def collision_checker(self, path_pts, obstacles):
        #TODO: 1.change the protect variable to the obstacle's radius value.
        path_validity = np.zeros(len(path_pts), dtype=bool)
        protect = 5
        obstacles = np.squeeze(obstacles)
        if len(obstacles.shape) < 2:
            dis = np.linalg.norm(obstacles[0:2] - path_pts[:,:,0:2], axis=2)
            path_validity |= np.any(dis<protect, axis=1)
        else:
            for obs in obstacles:
                dis = np.linalg.norm(obs[0:2] - path_pts[:,:,0:2], axis=2)
                path_validity |= np.any(dis<protect, axis=1)
        
        return np.logical_not(path_validity)
        
    """
    Make score to a specific path.
    Input:
        path_pts: A list of path in the local coordinate.
        
        path: A 2d nparray in local coordinate, Content is a list of points.
            format: [[x_points, y_point, t, velocity]
                     [x_points, y_point, t, velocity]......]
        
        goal_state: current goal waypoint for the vehicle to reach in local coordinate.
            format: [x_goal, y_goal, t, v_goal]
        
        path_validity: 1d array of boolen value to classify whether a path is 
                       collision-free(True), or not(False). 
    Output:
        score: A 1d array to indicate the score of the specific path,
               higher score implies the suitable path, 
               '-Inf' represents the colliding path's score.
    """
    def make_score(self, path_pts, goal_state, path_validity):
        score = []
        highest_centerline_distance = np.sqrt( (path_pts[0][-1][0] - goal_state[0])**2 + (path_pts[0][-1][1] - goal_state[1])**2 )
        """
        print("\npath_validity: ", path_validity)
        print("\ngoal_state: ", goal_state)
        for i in range(len(path_pts)):
            print("\npath ", i, " :", path_pts[i][-1])
        """
        for i in range(len(path_pts)):
            # Handle the case of collision-free path.
            
            if path_validity[i]:
                # Compute the collision-free path goal point's distance to centerline goal point.
                # A low distance implies the suitable path.
                # however, we think a higher score means better intuitively.
                # so I let score = ( 1/(distance to centerline goal point + 10) )*highest_centerline_distance, +10 to prevent divsion by zero
                # let a higher score implies the suitable path.
                distance_to_centerline = np.sqrt( (path_pts[i][-1][0] - goal_state[0])**2 + (path_pts[i][-1][1] - goal_state[1])**2)
                score.append( ( 1/( distance_to_centerline + 10 ) ) * highest_centerline_distance )
                #print("path ", i, " to centerline is: ", distance_to_centerline)

                for j in range(len(path_pts)):
                    # Compute the collision-free path goal point's distance to colliding path's goal point
                    # then add it to the score, farther distance to the colliding path implies the suitable path.
                    
                    if j == i:
                        continue
                    if not path_validity[j]:
                        distance_to_colliding_path = np.sqrt( (path_pts[i][-1][0] - path_pts[j][-1][0])**2 + (path_pts[i][-1][1] - path_pts[j][-1][1])**2 )
                        score[i] += distance_to_colliding_path
                        #print("path ", i, " to path ", j, " is: ", distance_to_colliding_path)
            else:
                score.append(float('-Inf'))
        #print("\nscore", score)
        return score

    """
    Choose best path according to `make_score` function.
    Input:
        score: 1d array
        path_pts: A list of path in the local coordinate.
    Output: path, 2d Array
    """
    def choose_best_path(self, score, path_pts):
        best_index = np.argmax(score)
        #print("\nbest_index: ", best_index)
        return path_pts[best_index], best_index

    def generate_emergency_path(self):
        return

    def generate_velocity_profile(self):
        return 

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import WaypointsPlanner as wp
    import Simulation as si 
    import VelocityProfile 

    vel_config = VelocityProfile.VelocityConfig(10, 0.1, 0.1, -0.5)
    way = wp.WaypointsPlanner(vel_config)
    mp = MotionPlanner()

    sim = si.Simulation(way, 10)
    
    current_state = np.array([0, 100, 1, 10], dtype=np.float64)

    # simulate some obstacles
    sim_obs = way.waypoints[:,0:2] + 200*(np.random.rand(way.waypoints.shape[0], 2) - 0.5)
    
    plt.figure(num="Global Coordinate Map")
    
    while way.available():
        # clear the window
        plt.cla()
        #sim.plot_ways()
        #sim.plot_obs(sim_obs)
        current_goal_waypoint = way.load_waypoint(current_state) 
        #sim.plot_vehicle(current_state)

        target_set, path_pts, danger = mp.generate_path(current_state, current_goal_waypoint, sim_obs, 11, 2)

        #sim.scatter_with_local(current_state, target_set[:,0:2], '.', 'red')
        
        goal_state = Utils.trans_global_to_local(current_state, current_goal_waypoint[0:2])
        path_validity = mp.collision_checker(path_pts, danger)
        score = mp.make_score(path_pts, goal_state, path_validity)
        [best_path, best_idx] = mp.choose_best_path(score, path_pts)
        danger = Utils.trans_local_to_global(current_state, danger)
        
        #sim.plot_obs(danger, "red")

        #for path in path_pts[path_validity]:
        #    sim.plot_with_local(current_state, path, 'g-')
            
        #sim.plot_with_local(current_state, best_path, 'y-')
        
        #TODO: implement the smith predictor in the below the code.
        current_state[0:2] = Utils.trans_local_to_global(current_state, target_set[best_idx][0:2])
        current_state[2] += target_set[best_idx][2]
        #current_state = current_goal_waypoint 
        
        # compare the path and the algorithm of car trying to move along the path
        for path in path_pts[path_validity]:
            plt.plot(path[:,0], path[:,1], 'r-')
            sim.plot_estimated_lane()
        
        plt.pause(1)
