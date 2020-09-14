import numpy as np
import Utils
import Path 
import matplotlib.pyplot as plt

class Car():
    def __init__(self):
        """      ^
                 |
        rl ______|______ lt(left top) 
          |    (0,0)    |   car moving direction
          |------+------|------->x
        rb|______|______|rt(right top)
          |--lr--|--lf--| 
        
        unit: m
        """
        self.car_length = 1.3
        self.car_width = 1
        self.lr = self.car_length/2
        self.lf = self.car_length/2
        self.lt = np.array(( self.lf, self.car_width/2))
        self.rt = np.array(( self.lf, -self.car_width/2))
        self.lb = np.array(( -self.lr,  self.car_width/2))
        self.rb = np.array(( -self.lr, -self.car_width/2))

        # Kinectic Parameters
        self.x = 0
        self.y = 0
        self.v = 0      # Rear wheel velocity
        self.vf = 0     # Center velocity
        self.beta = 0   # Velocity angle relative to yaw
        self.yaw = 0    # Heading
        self.steering = 0   # in Radiun
        # Define maximum steering angle
        self.max_steer = np.radians(30)

    def set_state(self, current_state):
        self.x = current_state[0]
        self.y = current_state[1]
        self.yaw = current_state[2]
        self.v = current_state[3]
    """
    Update kinectic parameters according to current steering 
    angle and rear wheel speed.
    """
    def update(self, steer, acc, dt):
        # Constrain steering angle
        self.steering = np.clip(steer, -self.max_steer, self.max_steer)

        self.beta = np.arctan(self.lr * np.tan(self.steering) / self.car_length)
        self.vf = self.v / np.cos(self.steering)
        local_p = self.vf * dt * np.array([np.cos(self.beta), np.sin(self.beta)])
        [self.x, self.y] = Utils.trans_local_to_global(
                np.array([self.x, self.y, self.yaw, self.vf]), 
                local_p)
        self.yaw += self.v * dt * np.tan(self.steering) / self.car_length 

        # Update velocity
        self.v += acc * dt 

    def get_current_state(self):
        return np.array([self.x, self.y, self.yaw, self.v])

class Simulation():
    def __init__(self, wayplanner, car_scale):
        if wayplanner is not None: 
            self.waypoints = wayplanner.waypoints
        self.car = Car()
        self.scale = car_scale

    """
    Plot waypoints given by waypoints planner.
    """
    def plot_ways(self):
        color = 0.1*np.ones((self.waypoints.shape[0],3))
        color[:,2] = np.clip(self.waypoints[:,2]/10, 0, 1)
        color[:,1] = 1-np.clip(self.waypoints[:,2]/10, 0, 1)
        plt.scatter(self.waypoints[:, 0], self.waypoints[:, 1], marker=".", c=color)
        return

    def plot_obs(self, obs, color="purple"):
        obs = np.squeeze(obs)
        if len(obs.shape) >= 2:
            plt.scatter(obs[:,0], obs[:,1], marker="o", color=color)
        else:
            plt.scatter(obs[0], obs[1], marker="o", color=color)
        return
    """
    Plot line segment with point given in local coordinate.
    """
    def plot_with_local(self, current_state, points, args, scale=1):
        n_pts = Utils.trans_local_to_global(current_state, points, scale)
        plt.plot(n_pts[:,0], n_pts[:,1], args)
        return 

    """
    Plot scatter with points given in local coordinate.
    Input:
        points: 1d or 2d nparray in local coordinate.
            1d for 1 point, 2d for multiple points.
    """
    def scatter_with_local(self, current_state, points, marker, color, scale=1):
        n_points = Utils.trans_local_to_global(current_state, points, scale)
        if len(n_points.shape) >= 2:
            plt.scatter(n_points[:,0], n_points[:,1], color=color, marker=marker)
        else:
            plt.scatter(n_points[0], n_points[1], color=color, marker=marker)
        return 

    def plot_vehicle(self, current_state, scale=10):
        self.plot_with_local(current_state, np.array( [self.car.rb, self.car.lb]), 'r-', scale)
        self.plot_with_local(current_state, np.array( [self.car.rt, self.car.lt]), 'g-', scale)
        self.plot_with_local(current_state, np.array( [self.car.lt, self.car.lb]), 'k-', scale)
        self.plot_with_local(current_state, np.array( [self.car.rt, self.car.rb]), 'k-', scale)

        vec = current_state[3] * np.array(
                [np.cos(current_state[2]),
                 np.sin(current_state[2])])
        plt.arrow(current_state[0],
                  current_state[1],
                  vec[0], vec[1], width=scale/10)
        return
    
    def plot_vehicle_velocity(self, current_state, beta, scale=10):
        self.plot_vehicle(current_state, scale)
        vec = current_state[3] * np.array(
                [np.cos(beta + current_state[2]),
                 np.sin(beta + current_state[2])])
        plt.arrow(current_state[0],
                  current_state[1],
                  vec[0], vec[1], width=scale/10, 
                  color="orange")
    """
    Plot the car's marching line and the best_path.
    compare the path and the algorithm of car trying to catch the path
    Input:
        path: A 2d nparray in local coordinate, Content is a list of points.
            format: [[x_points, y_point, t, velocity]
                     [x_points, y_point, t, velocity]......]
            
       car_info: 2d nparray, contains the velocity of the car, front steer angle,
                 current time.
            format: [[velocity, steer angle, time]
                     [velocity, steer angle, time]......]
    """
    """
    def plot_best_path_and_estimated_lane(path, car_info):
        local_yaw = 0
        for i in range(len(car_info)-1):
            
        return
    """

    def plot_path(self, path, args='g-'):
        [x, y, yaw, v] = path.sampling().T
        self.plot_with_local(
                path.origin, 
                np.array([x, y]).T, args, scale=1)

