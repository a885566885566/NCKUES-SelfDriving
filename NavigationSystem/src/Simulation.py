import numpy as np
import Utils
import matplotlib.pyplot as plt

class Car():
    def __init__(self):
        """
        y ^
          |
        lb|_____________ lt(left top) 
          |             |   car moving direction
    (0,0) |-------------|------->x
        rb|_____________|rt(right top)
           
        unit: m
        """
        self.car_length = 1.3
        self.car_width = 1
        self.lt = np.array(( self.car_length, self.car_length//2))
        self.rt = np.array(( self.car_length, -self.car_length//2))
        self.lb = np.array(( 0,  self.car_length//2))
        self.rb = np.array(( 0, -self.car_length//2))

class Simulation():
    def __init__(self, wayplanner, car_scale):
        self.waypoints = wayplanner.waypoints
        self.car = Car()
        self.scale = car_scale

    """
    Plot waypoints given by waypoints planner.
    """
    def plot_ways(self):
        plt.scatter(self.waypoints[:, 0], self.waypoints[:, 1], marker=".")
        return

    """
    Plot line segment with point given in local coordinate.
    """
    def plot_with_local(self, current_state, start, end, args, scale=1):
        n_start = Utils.trans_local_to_global(current_state, start, scale)
        n_end   = Utils.trans_local_to_global(current_state, end, scale)
        plt.plot([n_start[0], n_end[0]], [n_start[1], n_end[1]], args)
        return 

    """
    Plot scatter with points given in local coordinate.
    Input:
        points: 1d or 2d nparray in local coordinate.
            1d for 1 point, 2d for multiple points.
    """
    def scatter_with_local(self, current_state, points, marker, color, scale=1):
        n_points   = Utils.trans_local_to_global(current_state, points, scale)
        plt.scatter(n_points[:,0], n_points[:,1], color=color, marker=marker)
        return 

    def plot_vehicle(self, current_state, scale=10):
        self.plot_with_local(current_state, self.car.rb, self.car.lb, 'r-', scale)
        self.plot_with_local(current_state, self.car.rt, self.car.lt, 'g-', scale)
        self.plot_with_local(current_state, self.car.lt, self.car.lb, 'k-', scale)
        self.plot_with_local(current_state, self.car.rt, self.car.rb, 'k-', scale)

        vec = current_state[3] * np.array(
                [np.cos(current_state[2]),
                 np.sin(current_state[2])])
        plt.arrow(current_state[0],
                  current_state[1],
                  vec[0], vec[1], width=1)
        return

