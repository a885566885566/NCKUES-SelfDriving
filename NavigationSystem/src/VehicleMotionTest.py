import numpy as np
import Simulation
import matplotlib.pyplot as plt
import Utils 

class Controller():
    def __init__(self, a, b):
        self.a = a
        self.b = b

    def calc_slope(self, x):
        return 3 * self.a * x**2 + 2 * self.b * x

    def calc_y(self, x):
        return self.a * (x**3) + self.b * (x**2)

    """
    y = s * x + b
    """
    def calc_offset(self, slope, x, y):
        return y - slope * x

    """
    Giving the slope and one point that pass through it, 
    calculate the offset of the line equation.
    y = s1 * x + b1
    y = s2 * x + b2 
    """
    def solve_intersect(self, s1, b1, s2, b2):
        xi = (b2 - b1)/(s1 - s2)
        yi = s1 * xi + b1
        return xi, yi 

    def angle_normalize(self, angle):
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle


class LookAheadStanley(Controller):
    def __init__(self, a, b):
        super().__init__(a, b)
        self.max_steering_angle = 0.7

    """
    Calculate the approximation look ahead point.
    """
    def calc_lookahead(self, x, y, v, yaw, t):
        # Calculate the line on the reference trajectory refer to current position 
        # y = s1 * x + b1, x1 and y1 pass through the line
        s1 = self.calc_slope(x)
        y1 = self.calc_y(x) 
        b1 = self.calc_offset(s1, x, y1)

        # Calculate the line perpencular to the look ahead vector
        # y = s2 * x + b2, x1 and y1 pass through the line
        x2 = x + v * t * np.cos(yaw)
        y2 = y + v * t * np.sin(yaw)
        s2 = np.tan(yaw)
        if s2 < 0.0001:
            # Horizontal, line2: x=x2
            xh = x2
            yh = s1 * x2 + b1  
        else:
            s2 = -1 / s2 
            b2 = self.calc_offset(s2, x2, y2)
            [xh, yh] = self.solve_intersect(s1, b1, s2, b2)
            #plt.plot([x, x+1], [y1, s1*(x+1)+b1])
            #plt.plot([x2, x2-1], [y2, s2*(x2-1)+b2])
            #plt.scatter(x2, y2)
        return xh, yh 

    def stanley(self, car, look, k):
        # Inner dot
        e = (car.x-look[0])*(np.cos(car.yaw+np.pi/2)) + (car.y-look[1])*(np.sin(car.yaw+np.pi/2))
        theta = car.yaw - np.arctan(self.calc_slope(look[0]))

        steering = -( theta + np.arctan(k*e/(car.vf+1)) )
        s = self.max_steering_angle 
        steering = s if steering>s else -s if steering<-s else steering
        return steering 

class StanleyControl(Controller):
    def __init__(self, a, b):
        super().__init__(a, b)
        self.max_steering_angle = 0.7

    def calc_lookahead(self, x, y, v, yaw, t):
        self.front = Utils.trans_local_to_global(
                np.array([x, y, yaw, v]), 
                np.array([0.65, 0]))
        return self.front[0], self.calc_y(self.front[0])

    def stanley(self, car, look, k):
        target_yaw = np.arctan(self.calc_slope(look[0]))
        #target_yaw = self.angle_normalize(target_yaw)
        theta_e = car.yaw - target_yaw
        theta_e = self.angle_normalize(theta_e)
        delta = self.front - look
        e = delta[0]*(-np.cos(car.yaw+np.pi/2)) + \
            delta[1]*(-np.sin(car.yaw+np.pi/2))

        theta_d = np.arctan(k * e / car.vf)
        steering = self.angle_normalize(theta_e + theta_d)
        return steering

def main():
    import VelocityProfile 
    sim = Simulation.Simulation(None, 0.1)

    a = 0.01
    b = -0.1
    dt = 0.1    # Loop interval

    car = Simulation.Car()
    TrajCtrl = LookAheadStanley(a, b)
    #TrajCtrl = StanleyControl(a, b)

    # Plot trajectory
    traj_x = np.arange(100)/10  # 10 meters away
    traj_y = TrajCtrl.calc_y(traj_x)

    # Initial condition
    car.v = 1
    car.y = -0.1
    car.steering = 0
    car.yaw = 0

    plt.xlim(0, 10)
    plt.ylim(0, 10)
    for i in range(100):
        plt.cla()
        plt.plot(traj_x, traj_y)
        sim.plot_vehicle_velocity(car.get_current_state(), car.beta, scale=0.1)

        # Calculate reference look ahead point
        look = TrajCtrl.calc_lookahead(car.x, car.y, car.vf, car.beta+car.yaw, 1)

        # Calculate steering angle command
        steering = TrajCtrl.stanley(car, look, 5)
        print("steer= ", steering)
        car.steering = steering

        plt.scatter(look[0], look[1])
        car.update(dt)
        plt.pause(dt)
    plt.show()

if __name__ == "__main__":
    main()
