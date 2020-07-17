import numpy as np
import Utils
import matplotlib.pyplot as plt

class Car():
    def __init__(self):
        """
      lt  _______ rt
          |     |
          |     |
          |     |
          |     |
      lb  |__.__| rb
           (0,0)

        unit: m
        """
        self.lt = np.array((-0.5, 1.3))
        self.rt = np.array(( 0.5, 1.3))
        self.lb = np.array((-0.5, 0))
        self.rb = np.array(( 0.5, 0))

class Simulation():
    def __init__(self, wayplanner):
        self.waypoints = wayplanner.waypoints
        self.car = Car()

    def plot_ways(self):
        plt.scatter(self.waypoints[:, 0], self.waypoints[:, 1], marker=".")
        return

    def plot_vehicle(self, current_state):
        RM = Utils.get_rotation_matrix(current_state[2])
        p_rb = current_state[0:2] + np.matmul(RM, self.car.rb)
        p_lb = current_state[0:2] + np.matmul(RM, self.car.lb)
        p_rt = current_state[0:2] + np.matmul(RM, self.car.rt)
        p_lt = current_state[0:2] + np.matmul(RM, self.car.lt)
        
        plt.plot([p_rb[0], p_lb[0]], [p_rb[1], p_lb[1]], 'k-')
        plt.plot([p_rt[0], p_lt[0]], [p_rt[1], p_lt[1]], 'k-')
        plt.plot([p_lt[0], p_lb[0]], [p_lt[1], p_lb[1]], 'k-')
        plt.plot([p_rt[0], p_rb[0]], [p_rt[1], p_rb[1]], 'k-')

        vec = current_state[3] * np.array(
                [np.cos(current_state[2]),
                 np.sin(current_state[2])])
        plt.arrow(current_state[0],
                  current_state[1],
                  vec[0], vec[1], width=1)
        return

