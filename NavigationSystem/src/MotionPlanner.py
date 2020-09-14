import numpy as np
import Utils
import WaypointsPlanner as wp 
import Path 

class VelocityControl:
    def __init__(self, _kp, _ki, _dt):
        self.Kp = _kp   # speed proportional gain
        self.Ki = _ki 
        self.dt = _dt 
        self.integral = 0

    """
    Calculate the velocity command with the origin and target 
    point in a path. 
          current_x 
    p = ------------- in local 
          target_x 
    velocity = origin.v + p*(target.v - origin.v)
    """
    def get_vel_cmd(self, curretnt_state, path):
        if path is None:
            origin = current_state
        else:
            origin = path.origin 
        local = Utils.trans_global_to_local(
            origin, current_state[0:2])
        progress = local[0] / path.target[0]
        return origin[3] + progress * (path.target[3] - origin[3])

    """
    Velocity controller 
    """
    def pi_control(self, current_state, car):
        error = current_state[3] - car.vf;
        self.integral += self.Ki * error 
        return self.Ki * error + self.integral 
    
    def update(self, current_state, path, car):
        vel_cmd = self.get_vel_cmd(current_state, path)
        acc_cmd = self.pi_control(current_state, car)
        return acc_cmd 


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
        # Current path
        self.path = None 

    def check_path_validity(self, current_state, obs, regen_portion=0.7):
        """
        Check if the current path still valid to use.
        Input: 
            current_state
            obs: obstacles
        Output:
            Validity, boolean
        """
        #　Path not exist
        if self.path is None:
            return False

        # The progress of the path has exceeded the regenerate portion
        local = Utils.trans_global_to_local(
                self.path.origin, current_state[0:2])
        if local[0] / self.path.target[0] > regen_portion:
            return False 

        # New obstacles collide the original path
        if self.path.collision_check(obs):
            return False 
        return True 

    """
    Generate `target_set`
    Input:
        current_state:  1d nparray, in global 
        goal_state:     1d nparray, in local 
        num_of_points:  odd integer, number of target set
        offset:         number, distance between target set
    Output:
        target_set: 2d nparray, in local
        slope: number, would be used in obstacles filter 
    """
    def generate_target_set(self, current_state, goal_state, num_of_points, offset):
        self.target_set = np.zeros((num_of_points, 4))

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

    def generate_path(self, current_state, goal_waypoint, obstacles, 
            num_of_points, offset):
        """
        Create best path.
        Input: 
            current_state: 1d nparray, which contains the current 
                information  of the vehicle in global coordinate, 
                format:[x_point, y_point, yaw, velocity]
    
            goal_waypoint: 1d nparray, a state which choose from 
                waypoints for the vehicle to reach in global coordinate.
                format:[x_point, y_point, yaw, velocity]
    
            num_of_points: odd number, number of point in target_set
    
            offset: number, distance between `target_set` points
    
        Output:
            path: Object of Path 
        """

        # Transfer global target to local target
        origin = current_state
        pre_vel = 1 if self.path is None else self.path.target[3]
        origin[3] = pre_vel

        local_target = np.zeros(4)
        local_target[0:2] = Utils.trans_global_to_local(
                origin, goal_waypoint[0:2])
        local_target[2] = goal_waypoint[2] - origin[2]
        local_target[3] = goal_waypoint[3]

        # Generate target set
        [target_set, target_set_slope] = self.generate_target_set(
                current_state, local_target, num_of_points, offset)

        # Filter out those obstacles that is impossible to collide
        obs = self.obstacles_filter(
                current_state, goal_waypoint, obstacles, target_set_slope )

        # TODO: the path_pts only have x and y value now, should give 
        # radian and velocity in each point.

        path_list = []
        # Create potential path list
        for target in target_set:
            path = Path.Path(origin=current_state, target=target)
            if path.collision_check(obs) == 0:
                path_list.append(path)

        if len(path_list) == 0:
            # There does not exist sutible path
            # TODO 
            return (None, None)

        # Choose the best path
        scores = []
        for path in path_list:
            score = self.make_score(path, local_target)
            scores.append(score)
        best_idx = self.choose_best_path(
                np.array(scores), 
                weight=np.array([0.5, 0.5]))
        print(best_idx)
        self.path = path_list[best_idx]
        return self.path, path_list  

       
    def obstacles_filter(self, current_state, goal_waypoint, obstacles, slope):
        """
        This function filter out some obstacles that is not posible
        to collide the path zone.
        Input:
            current_state, in global 
            goal_waypoint, in global  
            obs: obstacles, in global coord, 2d nparray
            slope: target_set line slope, return from 
                generate_target_set function
        Output:
            obstacles that have possibility to collide.
        """
        # Translate to local coordinate
        local_goal = Utils.trans_global_to_local(
                current_state, goal_waypoint[0:2])
        obs = Utils.trans_global_to_local(current_state, obstacles[:, 0:2])
        # Filter out those behind the car
        #TODO: the filter seems have problem, so we return right here.
        obs = obs[obs[:,0]>0]
        return obs 

        # Filter out those in front of car
        # The discriminant
        if slope >= 99999:  # Almost verticle
            D = obs[:, 0] - local_goal[0]
        else:
            # The line intersect to goal and perpendicular to its 
            # vector to origin. D = (y-y_g) + m*(x-x_g)
            D = obs[:,1] + slope * obs[:,0] 
            D *= (1 if local_goal[1]>0 else -1)
        return obs[D<0]

    def make_score(self, path, goal_state):
        """
        Make score to a specific path.
        Input:
            path: Object of path 
            goal_state: current goal waypoint for the vehicle in local 
                coordinate.
                format: [x_goal, y_goal, t, v_goal]
            
        Output:
            score: A 1d array to indicate the score of the specific path in 
            different judgement scheme.
        """
        scores = []
        # Score 1: Goal distance to ideal goal_waypoint
        # Distance larger is bad
        goal_distance = np.linalg.norm(path.target[0:2] - goal_state[0:2])
        scores.append(goal_distance)

        # Score 2: Minimum value of the maxima of the curvature of a path
        # Curvature larger is bad
        sample_x = np.linspace(0, path.target[0], num=10)
        curvature = path.calc_curvature(sample_x)
        maxima = np.max(np.abs(curvature))
        scores.append(maxima)
        return np.array(scores)

    def choose_best_path(self, scores, weight=np.array((0.5, 0.5))):
        """
        Choose best path according to `make_score` function.
        Input:
            scores: 2d numpy array, indicate scores in different judgement 
                of each path, shape in 
                [numb_of_path, num_of_score_judgement]
        Output: 
            path, Object of Path 
        """
        # Normalization to each score
        minima = np.min(scores, axis=0)
        maxima = np.max(scores, axis=0)
        normalized = (scores - minima) / (maxima - minima)

        weighted = weight * normalized
        total_score = np.sum(weighted, axis=1)
        return np.argmin(total_score)
        # the rule is really sample:
        # 1. if the path to waypoint is collision free, then follow it.
        # 2. if the last path is collision free, then follow it.
        # 3. find the path which is collision free and closet to the last path and the path to waypoint.
        # TODO: 4. no path is collision free, should break.
        """
        centerline_idx = int((numb_of_path-1)/2)

        if path_validity[centerline_idx]:
            best_idx = centerline_idx
            last_path_idx = best_idx
            return best_idx, last_path_idx
        
        elif path_validity[last_path_idx]:
            best_idx = last_path_idx
            return best_idx, last_path_idx
        
        elif np.any(path_validity):
            right_idx = last_path_idx
            left_idx = last_path_idx
            while True:
                if right_idx + 1 <= numb_of_path-1:
                    right_idx = right_idx + 1
                if left_idx - 1 >= 0:
                    left_idx = left_idx - 1
                    
                if path_validity[right_idx] and path_validity[left_idx]:
                    if right_idx - centerline_idx >= centerline_idx - left_idx:
                        best_idx = right_idx
                        last_path_idx = best_idx
                        return best_idx, last_path_idx
                    else:
                        best_idx = left_idx
                        last_path_idx = best_idx
                        return best_idx, last_path_idx
                
                if path_validity[right_idx] and not(path_validity[left_idx]):
                    best_idx = right_idx
                    last_path_idx = best_idx
                    return best_idx, last_path_idx
                
                if not(path_validity[right_idx]) and path_validity[left_idx]:
                    best_idx = left_idx
                    last_path_idx = best_idx
                    return best_idx, last_path_idx
        else:
            # should output singal to break, because no path can follow
            best_idx = last_path_idx
            return best_idx, last_path_idx
        #print("\nbest_index: ", best_index)
        #return path_pts[best_index], best_index, last_path_idx
        """

    def generate_emergency_path(self, car, k):
        car.x -= k * car.v * np.cos(car.yaw)
        car.y -= k * car.v * np.sin(car.yaw)
        return car 

    def generate_velocity_profile(self):
        return 

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import WaypointsPlanner as wp
    import Simulation as si 
    import VelocityProfile
    from VehicleMotionTest import LookAheadStanley as Stanley 

    vel_config = VelocityProfile.VelocityConfig(10, 0.1, 0.1, -0.5)
    way = wp.WaypointsPlanner(vel_config)
    mp = MotionPlanner()
    sim = si.Simulation(way, 10)
    stanley = Stanley()
    car = si.Car()
    velocity_control = VelocityControl(0.1, 0, 0.1)
    
    current_state = np.array([0, 100, 1, 10], dtype=np.float64)
    # Use first way point as first state
    current_state = way.load_waypoint(current_state)

    car.set_state(current_state)
    car.v = 2
    view_range = 50

    # simulate some obstacles
    sim_obs = way.waypoints[:,0:2] + 200*(np.random.rand(way.waypoints.shape[0], 2) - 0.5)
    obs_vel = 2*(np.random.rand(way.waypoints.shape[0], 2)-0.5)
    plt.figure(num="Global Coordinate Map")
    
    while way.available():
        
        #%% clear the window and plot some object
        plt.cla()
        sim.plot_ways()

        #TODO: use Perception.py to get the obstacle information
        
        if mp.check_path_validity(current_state, sim_obs, regen_portion=0.5):
            # Path remain vaild
            pass 
        else:
            print("New path")
            # Require to generate new path
            current_goal_waypoint = way.load_waypoint(current_state) 
            [path, path_list] = mp.generate_path(current_state, current_goal_waypoint, sim_obs, 21, 2)
            if path is not None:
                # Update new path
                stanley.update_path(mp.path)
            else:    
                for i in range(10):
                    car = mp.generate_emergency_path(car, 0.1)
                    sim.plot_vehicle(car.get_current_state())
                    plt.xlim(current_state[0]-2*view_range, current_state[0]+2*view_range)
                    plt.ylim(current_state[1]-view_range, current_state[1]+view_range)
                    plt.pause(0.1)
                mp.path = None 
        
        lookahead = stanley.calc_lookahead(car, 0.1)
        steering = stanley.stanley(car, lookahead, 5)

        if mp.path is not None:
            acc_cmd = velocity_control.update(current_state, mp.path, car)
        else:
            acc_cmd = 0
        car.update(steering, acc_cmd, 0.1)
        current_state = car.get_current_state()

        try:
            # Plot paths
            # Plot target
            plt.scatter(current_goal_waypoint[0], current_goal_waypoint[1], marker='*')
            
            for p in path_list:
                sim.plot_path(p, 'y--')
            
            # Plot best path
            sim.plot_path(mp.path, 'g-')       

            sim.plot_obs(sim_obs)
            sim.plot_vehicle(current_state)

            plt.xlim(current_state[0]-2*view_range, current_state[0]+2*view_range)
            plt.ylim(current_state[1]-view_range, current_state[1]+view_range)
        except:
            print("Fail to plot")
        plt.pause(0.1)
        sim_obs += obs_vel

