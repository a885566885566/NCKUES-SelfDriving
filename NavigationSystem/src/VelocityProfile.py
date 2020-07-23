import numpy as np

class VelocityConfig():
    def __init__(self, max_vel, max_angular_vel, max_acc, min_acc):
        self.max_a = max_acc
        self.min_a = min_acc
        self.max_v = max_vel 
        self.max_w = max_angular_vel

    """
    Calculate proper velocity and time span according to given
    velocity upper bound(v0, v1), acceleration limit and distance
    need to travel.

            v1         2*ds 
   v0     /|     t = ---------
       /   |          v0 + v1
    /  ds  |         -v0+sqrt(v0^2 + 2*a*ds)
    |______|     t = -----------------------   ,by ds=v0*t+0.5*a*t^2

    Input:
        v0: basement velocity
        v1: the upper limit of the other point's velocity
        ds: the distance between the two points
    Return:
        v1: new velocity of point two
        t:  the time span of the two point should be 
    """
    def cal_velocity_and_time(self, v0, v1, ds):
        t = 2*ds / (v0+v1)
        # v1 > v0
        if (v1-v0)/t > self.max_a:
            t = (-v0 + np.sqrt(v0**2+2*self.max_a*ds))/self.max_a
            v1 = v0 + self.max_a * t
        elif (v1-v0)/t < self.min_a:
            t = (-v0 + np.sqrt(v0**2+2*self.min_a*ds))/self.min_a
            v1 = v0 + self.min_a * t
        return v1, t
    
    """
    Calculate velocity profile with the turning radius of the given 
    waypoints.
    Input:
        waypoints: 2d nparray, m
    Return:
        velocity: 1d nparray, m/s 
        timeline: 1d nparray, expected timestamp of each point 
    """
    def get_velocity_profile(self, waypoints):
        radii = self.calculate_turn_radius(waypoints)
        vel = self.max_v * np.ones(waypoints.shape[0])
        turn_limit = radii * self.max_w 
        # Velocity were all set to its upper bound, determined
        # by the velocity limitation.
        vel = np.where(turn_limit < self.max_v, 
                            turn_limit, vel)
        vel[0] = 0      # start point
        vel[-1] = 0     # terminal point
        ds = np.linalg.norm(waypoints[1:]-waypoints[0:-1], axis=1)
        
        # Time span between each point
        time = np.zeros(waypoints.shape[0])

        # not dealed points were set to true
        remain = np.ones(waypoints.shape[0], dtype=bool)
        indices = np.arange(waypoints.shape[0])
        last = waypoints.shape[0]-1

        # Loop until all points processed
        while np.any(remain):
            # Choose the smallest velocity one
            idx = np.argmin(vel[remain])
            idx = indices[remain][idx]
            
            remain[idx] = False # Mark as dealed
            # Left hand side(previous)
            if idx-1 > 0 and remain[idx-1]:
                [v1,t]=self.cal_velocity_and_time(vel[idx], vel[idx-1], ds[idx-1])
                vel[idx-1] = v1 
                time[idx] = t
            if idx+1 <= last and remain[idx+1]:
                [v1,t]=self.cal_velocity_and_time(vel[idx], vel[idx+1], ds[idx])
                vel[idx+1] = v1 
                time[idx+1] = t
        # Timeline from 0 to end
        timeline = np.add.accumulate(time)
        return vel, timeline 

    """
    Calculate curvature from discrete waypoints 
    curvature = sqrt(ddx^2+ddy^2)
    radius = 1 / curvature 
    Input:
        waypoints: 2d nparray
    Return:
        turning radius: 1d nparray
    """
    def calculate_turn_radius(self, waypoints):
        curvature = np.zeros((waypoints.shape[0]))
        d = (waypoints[1:,0:2]-waypoints[0:-1,0:2])
        ds = np.linalg.norm(d, axis=1)
        d = d/ds[:,np.newaxis]
        dd = (d[1:]-d[0:-1])/((ds[0:-1]+ds[1:])[:,np.newaxis]/2)
        curvature[1:-1] = np.linalg.norm(dd, axis=1)
        curvature = np.clip(curvature, 0.0001, 1)
        # Filter out extreme values 
        for i in range(1, curvature.shape[0]-1):
            if curvature[i] > 0.4:
                curvature[i] = curvature[i-1]
            if curvature[i] > curvature[i-1] + curvature[i+1]:
                curvature[i] = curvature[i-1]

        """
        import matplotlib.pyplot as plt 
        plt.figure()
        plt.plot(range(curvature.shape[0]), curvature, 'o')
        plt.plot(range(d.shape[0]), d, '.')
        plt.plot(range(ds.shape[0]), ds, '.')
        plt.show()
        """
        return 1/curvature 

if __name__ == "__main__":
    import WaypointsPlanner as mp
    import matplotlib.pyplot as plt 
    vel_config = VelocityConfig(10, 0.1, 1, -1)
    way = mp.WaypointsPlanner(vel_config)
    [vel, timeline] = vel_config.get_velocity_profile(way.waypoints)
    plt.plot(timeline, vel, '.')
    plt.show()
