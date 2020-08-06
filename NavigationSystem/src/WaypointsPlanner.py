import os
import json
import numpy as np 

"""
This class run A* algorithm when initializing, and obtain 
waypoints from JSON file.

# Check if any available waypoint, call available()
# Get next current goal waypoint, call load_waypoint()
"""
class WaypointsPlanner():
    def __init__(self, vel_config):
        self.ASTAR_EXCUTABLE_FILE = ''
        self.ASTAR_JSON_FOLDER = './astar_output'
        self.ASTAR_JSON_FILE_PATH = {
                'x_file':'longj_arrayx.json', 
                'y_file':'longj_arrayy.json'}
        self.waypoints = None 
        self.current_idx = 0
        self.__perform_astar(vel_config)
        return

    def __perform_astar(self, vel_config):
        # Run Astar algorithm
        #os.system()

        # Read from json output
        astar_x_file = open(os.path.join(self.ASTAR_JSON_FOLDER, self.ASTAR_JSON_FILE_PATH['x_file']))
        astar_x = np.asarray( json.load(astar_x_file) )
        astar_y_file = open(os.path.join(self.ASTAR_JSON_FOLDER, self.ASTAR_JSON_FILE_PATH['y_file']))
        astar_y = np.asarray( json.load(astar_y_file) )
        self.waypoints = np.zeros((astar_x.shape[0], 3))
        self.waypoints[:, 0:2] = np.vstack( (astar_x, astar_y) ).T

        # Calculate velocity profile
        self.waypoints[:,2] = vel_config.get_velocity_profile(self.waypoints)[0]
        return

    """
    Check if there is any available waypoints.
    Return true if there still have point 
    """
    def available(self):
        length = self.waypoints.shape[0]
        return not self.current_idx >= length -1 

    """
    This function return `current_goal_waypoint`.
    # TODO: Issue 
    Input:
        current_state: 1d nparray, [px, py, yaw, vel]
    Output:
        state: 1d nparray, containing position, yaw and
        velocity.
    """
    def load_waypoint(self, current_state):
        length = self.waypoints.shape[0]

        # get distance calculate from current index
        idx, dists = self.__get_closet_idx(current_state, self.current_idx)
        dist_sum = 0 

        # Rule for the choice for the next waypoint
        while idx < len(dists)-1 and dist_sum < self.__look_ahead_function(current_state):
            dist_sum += dists[idx]
            #dist_sum = dists[idx]
            idx += 1

        self.current_idx += idx  # `dists` start from current_idx

        # Calculate yaw
        if self.current_idx >= length-1:
            vec = self.waypoints[self.current_idx] - self.waypoints[self.current_idx-1]
        else:
            vec = self.waypoints[self.current_idx+1] - self.waypoints[self.current_idx]
        yaw = np.arctan(vec[1]/vec[0])

        return np.array((self.waypoints[self.current_idx][0], 
                         self.waypoints[self.current_idx][1], 
                         yaw, 
                         self.waypoints[self.current_idx][2]))

    """
    Calculate the closet index start from the given index
    Input:
        current_state: 1d nparray, [px, py, yaw, vel]
        start_idx: start from this index
    Output:
        idx, number
        dists, 1d nparray, from start_idx to end 
    """
    def __get_closet_idx(self, current_state, start_idx):
        # calculate the distance between each waypoint and current pos
        dists = np.linalg.norm(self.waypoints[start_idx:, 0:2] - current_state[0:2], axis=1)
        idx = np.argmin(dists)
        return idx, dists 

    """
    This function calculate `look_ahead_distance` with 
    velocity.
    """
    def __look_ahead_function(self, current_state):
        return (current_state[2]+8) * 5

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import Simulation as si 
    import VelocityProfile 

    vel_config = VelocityProfile.VelocityConfig(10, 0.1, 0.1, -0.5)
    way = WaypointsPlanner(vel_config)
    sim = si.Simulation(way, 10)

    plt.figure()
    current_goal_waypoint = way.waypoints[0]
    while way.available():
        plt.cla()
        sim.plot_ways()
        current_goal_waypoint = way.load_waypoint(current_goal_waypoint) 
        sim.plot_vehicle(current_goal_waypoint)

        print("(%d/%d)= "%(way.current_idx, way.waypoints.shape[0]), end=" ")
        print("(%d, %d), %.3f rad, %.1f m/s"%(current_goal_waypoint[0], current_goal_waypoint[1], current_goal_waypoint[2], current_goal_waypoint[3] ))

        # Plot current goal waypoint
        plt.scatter(current_goal_waypoint[0], current_goal_waypoint[1], marker="o", color='red', alpha=0.5)

        plt.pause(0.1)

    
