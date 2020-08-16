# -*- coding: utf-8 -*-
"""
Created on Thu Aug  6 21:21:36 2020

@author: johnny
"""

import numpy as np
import math
import matplotlib.pyplot as plt
import Simulation as si


class Car_State(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(Car_State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.car = si.Car()
        self.max_steer = np.radians(20)

    def update(self, acceleration, delta, dt=0.1):
        """
        Update the state of the vehicle.
        Stanley Control uses bicycle model.
        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -self.max_steer, self.max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / (self.car.car_length/2) * np.tan(delta) * dt
        self.yaw = self.normalize_angle(self.yaw)
        self.v += acceleration * dt
    
    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi
    
        while angle < -np.pi:
            angle += 2.0 * np.pi
    
        return angle


"""
    This controller class will calculate the steering angle with the given path points 
or path's function coefficient a, b, and output the current location to the motion plannar.
"""
 
class controller:
    
    def __init__(self):
        self.car_state = Car_State()
        self.k = 0.5  # control gain
        self.Kp = 1.0  # speed proportional gain
        self.dt = 0.001  # [s] time difference
        self.max_steer = np.radians(30.0)  # [rad] max steering angle
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        

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
        x1 = target_set[0,0]
        y1 = target_set[:,1]
        x1_2 = np.square(x1)    # square of x1
    
        b = 3*y1/x1_2 - tan_t/x1
        a = (tan_t-2*b*x1)/(3*x1_2)
        x = np.linspace(0, target_set[:,0], num_of_interpolate)
        y = a*np.power(x, 3) + b*np.square(x)
        return np.stack((x.T,y.T), axis=-1)[0], a, b
    
    
    """
        Given the path's function coefficient a, b, and x ,the function will calculate
    the yaw value of the path at the x location.
    """
    def cal_yaw(self, a, b, x):
        up = (6*a*x+2*b)
        down = ( (1 + ( 3*a*(x)**2 + 2*b*x )**2 ) )**(3/2)
        return (abs( up/down ))[0]
    
    
    """
        Given the path's function coefficient a, b, and x ,the function will calculate
    the curvature value of the path at the x location.
    """
    def cal_curvature(self, a, b, x):
        return math.tan(3*a*(x)**2 + 2*b*x)
    
    
    """
        Given the path points, the function will calculate
    the yaw value of the path.
    """
    def cal_yaw_with_pathpoints(self, path):
    
        yaw = []
        
        for i in range(len(path)-1):
            #print(i)
            dx = path[i+1][0]-path[i][0]
            dy = path[i+1][1]-path[i][1]
            yaw.append(np.arctan2(dy, dx))
            
        yaw.append(0)
        
        return yaw
    
    def cal_yaw_and_curvature(self, path):
        
        #k_array = []

        for point in path:
            self.path_x.append(point[0])
            self.path_y.append(point[1])
            #yaw_array.append( cal_yaw(a, b, point[0]) )
            #k_array.append( cal_curvature(a, b, point[0]) )

        self.path_yaw = self.cal_yaw_with_pathpoints(path)
    
    
    def calc_target_index(self, state, cx, cy):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + state.car.lf * np.cos(state.yaw)
        fy = state.y + state.car.lf * np.sin(state.yaw)
    
        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)
    
        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                          -np.sin(state.yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
    
        return target_idx, error_front_axle
    
    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi
    
        while angle < -np.pi:
            angle += 2.0 * np.pi
    
        return angle
    
    def pid_control(self, target, current):
        """
        Proportional control for the speed.
        :param target: (float)
        :param current: (float)
        :return: (float)
        """
        return self.Kp * (target - current)
    
    def stanley_control(self, last_target_idx):
        """
        Stanley steering control.
        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = self.calc_target_index(self.car_state, self.path_x, self.path_y)
    
        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx
    
        # theta_e corrects the heading error
        theta_e = self.normalize_angle(self.path_yaw[current_target_idx] - self.car_state.yaw)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.k * error_front_axle, self.car_state.v)
        # Steering control
        delta = theta_e + theta_d
    
        return delta, current_target_idx
    
    
    def update_path(self, path):
        
        self.cal_yaw_and_curvature(path)
        return
    
    def path_tracking(self, desired_speed):
        
        target_speed = desired_speed # km/hr
        
        """
        last_idx = len(self.path_x)-1

        x = [self.car_state.x]
        y = [self.car_state.y]
        yaw = [self.car_state.yaw]
        v = [self.car_state.v]
        t = [0]
        """
        target_idx, _ = self.calc_target_index(self.car_state, self.path_x, self.path_y)
        
        ai = self.pid_control(target_speed, self.car_state.v)
        di, target_idx = self.stanley_control(target_idx)
        self.car_state.update(ai, di)
        
        return
    
  
if __name__ == "__main__":
    
    import matplotlib.pyplot as plt
    import WaypointsPlanner as wp
    import Simulation as si 
    import VelocityProfile
    import MotionPlanner
    import Utils
    import stanley_controller

    vel_config = VelocityProfile.VelocityConfig(10, 0.1, 0.1, -0.5)
    way = wp.WaypointsPlanner(vel_config)
    mp = MotionPlanner.MotionPlanner()
    stm = stanley_controller.controller()

    sim = si.Simulation(way, 10)
    
    #Initialize car state
    current_state = np.array([0, 100, 1, 10], dtype=np.float64)
    # Use first way point as first state
    current_state = way.load_waypoint(current_state)
    print("current_state  : ", current_state)
    # simulate some obstacles
    sim_obs = way.waypoints[:,0:2] + 50*(np.random.rand(way.waypoints.shape[0], 2) - 0.5)
    
    plt.figure(num="Global Coordinate Map")
    
    counter = 0
    
    while np.linalg.norm(current_state[0:2] - way.waypoints[-1][0:2])>3:
        # clear the window
        plt.cla()
        sim.plot_ways()
        sim.plot_obs(sim_obs)
        sim.plot_vehicle(current_state)
        
        if counter == 0:
            
            #Initialize the state of local controller
            stm.car_state.x = 0
            stm.car_state.y = 0
            stm.car_state.yaw = 0
            stm.path_x.clear()
            stm.path_y.clear()
            stm.path_yaw.clear()
            #print("stm.path_x : ", stm.path_x, "stm.path_y : ", stm.path_y, "stm.path_yaw : ", stm.path_yaw)
            #stm.car_state.v = 0
            
            counter = 4
            current_goal_waypoint = way.load_waypoint(current_state)
            target_set, path_pts, danger = mp.generate_path(current_state, current_goal_waypoint, sim_obs, 11, 2)
            
                       
            goal_state = Utils.trans_global_to_local(current_state, current_goal_waypoint[0:2])
            path_validity = mp.collision_checker(path_pts, danger)
            score = mp.make_score(path_pts, goal_state, path_validity)
            [best_path, best_idx] = mp.choose_best_path(score, path_pts)
            

            danger = Utils.trans_local_to_global(current_state, danger)
            sim.plot_obs(danger, "yellow")
            sim.scatter_with_local(current_state, target_set[:,0:2], '.', 'red')

            for path in path_pts[path_validity]:
                sim.plot_with_local(current_state, path, 'g-')

            stm.update_path(best_path)

        stm.path_tracking(current_goal_waypoint[3])

        # plot the best path on global frame
        sim.plot_with_local(current_state, best_path, 'y-')

        current_state[0:2] = Utils.trans_local_to_global(current_state, np.array([stm.car_state.x, stm.car_state.y]))
        current_state[2] += stm.car_state.yaw
        current_state[2] = stm.normalize_angle(current_state[2])
        current_state[3] = stm.car_state.v
        #current_state = current_goal_waypoint
        
        print("\nstm.car_state.x : ", stm.car_state.x, "\nstm.car_state.y : ", stm.car_state.y,
              "\nstm.car_state.yaw : ", stm.car_state.yaw, "\nstm.car_state.v : ", stm.car_state.v)
        
        print("\ncurrent_state.x : ", current_state[0], "\ncurrent_state.y : ", current_state[1],
              "\ncurrent_state.yaw : ", current_state[2], "\ncurrent_state.v : ", current_state[3])
        
        counter-=1
        
        plt.pause(0.3)
    
    #plt.close()
        
    """
        #plt.subplots(1)
        plt.plot(stm.path_x, stm.path_y, ".r", label="course")
        plt.plot(stm.car_state.x, stm.car_state.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        plt.show()
        """
    
    """
    stm = controller()
    
    target_set = np.array([[100.0, -100.0, 0.01]], np.float32)

    path, a, b = stm.interpolate_path(target_set, 100)
    #print(path)
    
    path_x = []
    path_y = []
    path_yaw = []
    path_k = []
    path_v = []
    
    path_x, path_y, path_yaw = stm.cal_yaw_and_curvature(path, a, b)
    #print(path_x, "\n\n", path_y, "\n\n", path_yaw, "\n\n", path_k)
    
    target_speed = 25 # km/hr
    target_speed = target_speed / 3.6 # m/s
    
    #curvature = calculate_curvature(path)
    #print("\n", curvature)
    #plt.cla()
    #plt.plot(path_x, path_y, "-r", label="course")
    
    # Initial state
    #state = Car_State(x=0.0, y=0.0, yaw=np.radians(0.0), v=0.0)
    last_idx = len(path_x) - 1
    time = 0
    max_simulation_time = 100
    show_animation = True
    
    x = [stm.car_state.x]
    y = [stm.car_state.y]
    yaw = [stm.car_state.yaw]
    v = [stm.car_state.v]
    t = [0.0]
    
    target_idx, _ = stm.calc_target_index(stm.car_state, path_x, path_y)
    
    while max_simulation_time >= time and last_idx > target_idx:
        stm.path_tracking(path, target_speed)

        time += stm.dt

        x.append(stm.car_state.x)
        y.append(stm.car_state.y)
        yaw.append(stm.car_state.yaw)
        v.append(stm.car_state.v)
        t.append(time)

        if show_animation:  
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(path_x, path_y, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(path_x[target_idx], path_y[target_idx], "xg", label="target")
            #print("\npath_x[target_idx]: ", path_x[target_idx], "\npath_y[target_idx]: ", path_y[target_idx])
            #print("\nx: ", state.x, "y: ", state.y, "yaw: ", state.yaw)
            
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(stm.car_state.v * 3.6)[:4])
            plt.pause(0.001)
    
    """
    