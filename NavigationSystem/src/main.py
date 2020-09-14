import matplotlib.pyplot as plt
import WaypointsPlanner as wp
import Simulation as si 
import VelocityProfile
import Perception_socket_version



def main():
    vel_config = VelocityProfile.VelocityConfig(10, 0.1, 0.1, -0.5)
    way = wp.WaypointsPlanner(vel_config)
    mp = MotionPlanner()
    sensor = Perception_socket_version.Listener()
    
    sim = si.Simulation(way, 10)
    
    #current_state = np.array([0, 100, 1, 10], dtype=np.float64)
    # Use first way point as first state
    #current_state = way.load_waypoint(current_state)

    # simulate some obstacles
    #sim_obs = way.waypoints[:,0:2] + 200*(np.random.rand(way.waypoints.shape[0], 2) - 0.5)
    position = sensor.get_current_pose()
    current_state = np.array([position[0], position[1], position[2], position[3]], dtype=np.float64)
    plt.figure(num="Global Coordinate Map")
    last_best_path_idx = 5
    
    while way.available():
        
        #%% clear the window and plot some object
        plt.cla()
        sim.plot_ways()
        #sim.plot_obs(sim_obs)
        sim.plot_vehicle(current_state)
        #print("sim obs : ", sim_obs)
        
        #%% finding the new best path
        # there are 2 conditions:
        #   1. if there are obstacles, then find the new best path and send the a, b coefficient to the controller.
        #   2. if there is no obstacle, just calculate the path to the waypoint amd send the path's a, b to the controller.
        current_goal_waypoint = way.load_waypoint(current_state) 
        #TODO: use Perception.py to get the obstacle information
        obstacles = sensor.get_obstacles()
        # if there is obstacle
        if obstacles.size > 0:
            target_set, path_pts, trajectory_coefficient, slope_for_obstacle_filter = mp.generate_path(current_state, current_goal_waypoint, 11, 2)
            sim.scatter_with_local(current_state, target_set[:,0:2], '.', 'red')
            #danger = mp.obstacles_filter(current_state, current_goal_waypoint, sim_obs, slope_for_obstacle_filter)
            path_validity = mp.collision_checker(path_pts, obstacles)
            [best_path_idx, last_best_path_idx] = mp.choose_best_path(len(path_pts), path_validity, last_best_path_idx)
            #danger = Utils.trans_local_to_global(current_state, danger)
            sim.plot_obs(obstacles, "red")
        # else if there is no obstacle
        else:
            target_set, path_pts, trajectory_coefficient, slope_for_obstacle_filter = mp.generate_path(current_state, current_goal_waypoint, 1, 2)
            sim.scatter_with_local(current_state, target_set[0, 0:2], '.', 'red')
            path_validity = [True]
            [best_path_idx, last_best_path_idx] = mp.choose_best_path(1, path_validity, last_best_path_idx)
        
        for path in path_pts[path_validity]:
                sim.plot_with_local(current_state, path[:,0:2], 'g-')
        sim.plot_with_local(current_state, path_pts[best_path_idx, :, 0:2], 'y-')
        
        #%% update current state     
        position = sensor.get_current_position()
        current_state[0] = position[0]
        current_state[1] = position[1]
        current_state[2] = position[2]
        current_state[3] = position[3]

        #current_state = current_goal_waypoint 
        
        plt.pause(0.1)
    return

if __name__ == "__main__":
    main()
