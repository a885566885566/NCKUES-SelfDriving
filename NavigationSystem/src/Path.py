import numpy as np
import Utils

class Path():
    def __init__(self, origin, target):
        # Local scale
        self.a = 0
        self.b = 0
        # Local coordinate
        self.origin = origin 
        self.target = target 
        self.interpolate()

    def interpolate(self):
        """
        Interpolate path with cubic spline in local coordinate, which
        start from (0, 0) end to target_set(Local coord), with second 
        and third derivative to be zero. 
        Other boundary conditions:
        (0, 0):   f(0) =0,  f'(0) =0
        (x1, y1): f(x1)=y1, f'(x1)=dy/dx=tan(theta)
        S(x) = a*x^3 + b*x^2
            3*y1   tan(t)        tan(t) - 2*b*x1 
        a = ---- - ------   b =  --------------- 
            x1^2     x1               3*x1^2
    
        Input:
            target_set: 2d nparray
        Output:
            trajectory_coefficient: each a b coefficient of the path, shape in 
                                    [numb_of_path, num_of_coefficient]
                format: [[a1, b1]
                         [a2, b2]...]
        """

        tan_t = np.tan(self.target[2])
        x1 = self.target[0]
        y1 = self.target[1]
        x1_2 = np.square(x1)    # square of x1

        self.b = 3*y1/x1_2 - tan_t/x1
        self.a = (tan_t-2*self.b*x1)/(3*x1_2)
        return np.stack((self.a.T, self.b.T), axis=-1)

    def calc_curvature(self, x):
        """
        Calculate curvature with the given sample x.
                 6 a x + 2 b
        k = ------------------------
            [1+(3ax^2 +2bx)^2]^(3/2)
        """
        #return np.tan(3*self.a*(x)**2 + 2*self.b*x)
        return (6*self.a*x + 2*self.b)/np.power(1+(3*self.a*(x**2) + 2*self.b*x)**2, 3/2)

    def sampling(self, num_of_sample=10):
        """
        This function samplize the path with `num_of_sample` of points 
            Input:
                num_of_interpolate: integer, number of point in each spline
            Output:
                path_pts: each point of the path, shape in 
                          [numb_of_points, num_of_interpolate, 4]
                each point containing x position, y position, yaw, velocity
                in local coordinate.
        """
        x = np.linspace(0, self.target[0], num_of_sample)
        y = self.a*np.power(x, 3) + self.b*np.square(x)

        yaw = np.arctan(3*self.a*np.square(x) + 2*self.b*x)
        vel = 10 * np.ones(num_of_sample)
        
        #print("\na:", a, "\nb:", b, "\ncoe:", np.stack((a.T, b.T), axis=-1))
        return np.stack((x.T,y.T, yaw.T, vel), axis=-1) 

    def collision_check(self, obstacles, protect=5):
        """
        Get the information of obstacles and check if the path's validity.
        Input:
    	    obstacles: 2d nparray, stores the obstacle's position in 
                local coordinate axis, its safe radius, and the current 
                time.
    	        format: [[x, y, radius, time]
    			 [x, y, radius, time]......]
       Ouput:
            collision: integer, return number of obstacles that collision 
                may occur
        """
        #TODO: 1.change the protect variable to the obstacle's radius value.
        collision = 0
        path_pts = self.sampling()
        """
        obstacles = np.squeeze(obstacles)
        if len(obstacles.shape) < 2:
            dis = np.linalg.norm(obstacles[0:2] - path_pts[:,:,0:2], axis=2)
            path_validity |= np.any(dis<protect, axis=1)
        else:
        """
        for obs in obstacles:
            dis = np.linalg.norm(obs[0:2] - path_pts[:,0:2], axis=1)
            collision += 1 if np.any(dis<protect) else 0
        return collision 
 
