from communication.py import socket_server
import threading
import math
import numpy
import time

radius_table = [ 1,  # 1 mean person, so safty radius is 1m
                 2   # 2 mean vehicle, .. .... ...   ..  2m
                 ]

class Listener():
    def __init__(self):
        self.system_booted = False
        self.receiver = socket_server.Communicator()
        self.pose = []
        self.obstacles = {}
        self.coord_record_x = [0, 0, 0, 0]
        self.coord_record_y = [0, 0, 0, 0]
        self.daemon = threading.Thread(target = self.update_data)
        
        while not self.system_booted:
            data = self.receiver.check()
            if data != None:
                print("Starting perception.")
                self.system_booted = True
                self.daemon.start()
            else:
                print("Waiting the zed finishes initilization.\n")

    
    def update_data(self):
        while True:
            data = self.receiver.check()
           # print("\ndata:", data, "\n")
           # print("\ndata[0]:", data[0])
           # print("\ndata[1]:", data[1])
           # print("\ndata[1]['x']:", data[1]['x'])
            if data[0] == "POSE":
                # pose : [x, y, z, yaw, velocity]
                self.pose = [ data[1]['x'],
                              data[1]['y'], 
                              data[1]['z'],
                              self.quaternion_to_yaw(data[1]['ox'], 
                                                        data[1]['oy'],
                                                        data[1]['oz'],
                                                        data[1]['ow']),
                              self.calculate_velocity(data[1]['x']/1000,
                                                      data[1]['y']/1000)
                              ]
                # print("POSE is coming:", self.pose)
            elif data[0] == "OBS":
                #print(data)
                for i in range(len(data)-1):
                    label_id = data[i+1]['id']
                    if not label_id in self.obstacles:
                        self.obstacles[label_id] = {}
                    self.obstacles[label_id]['label'] = data[i+1]['label']
                    self.obstacles[label_id]['x'] = data[i+1]['x']
                    self.obstacles[label_id]['y'] = data[i+1]['y']
                    self.obstacles[label_id]['z'] = data[i+1]['z']
                    self.obstacles[label_id]['vx'] = data[i+1]['vx']
                    self.obstacles[label_id]['vy'] = data[i+1]['vy']
                    self.obstacles[label_id]['vz'] = data[i+1]['vz']
                    self.obstacles[label_id]['update'] = 50
                   # print("\nobstacle'", label_id, "updated.")
                #print("\nobs detected:", self.obstacles)

            for label_id in self.obstacles.copy():
                self.obstacles[label_id]['update'] -= 1
                if self.obstacles[label_id]['update'] == 0:
                    del self.obstacles[label_id]
    
    def calculate_velocity(self, xi , yi):
        # This function return velocity with unit m/s
        self.coord_record_x.pop(0)
        self.coord_record_x.append(xi)
        self.coord_record_y.pop(0)
        self.coord_record_y.append(yi)
        # dx/dt = (11/6)*x(i) + (-3)*x(i-1) + (3/2)*x(i-2) + (-1/3)*x(i-3)
        dx = (11/6)*self.coord_record_x[3] + \
             (-3)  *self.coord_record_x[2] + \
             (3/2) *self.coord_record_x[1] + \
             (-1/3)*self.coord_record_x[0]
        dy = (11/6)*self.coord_record_y[3] + \
             (-3)  *self.coord_record_y[2] + \
             (3/2) *self.coord_record_y[1] + \
             (-1/3)*self.coord_record_y[0]

        velocity =  math.sqrt( dx**2 + dy**2 )
        print("re_x:", self.coord_record_x)
        print("re_y:", self.coord_record_y)
        print("dx:", dx, "\ndy:", dy)
        print("v : ", velocity)
        return velocity

    def quaternion_to_yaw(self, x, y, z, w):
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def get_current_pose(self):
        #[x, y, yaw, velocity]
        return numpy.array([self.pose[0]/1000, self.pose[1]/1000, self.pose[3]*52.79, self.pose[4]])

    def get_obstacles(self):
        obstacles = []
        for label_id in self.obstacles:
            temp = [self.obstacles[label_id]['x']/1000,
                    self.obstacles[label_id]['y']/1000,
                    radius_table[self.obstacles[label_id]['label']-1]]
            obstacles.append(temp)
        return numpy.array(obstacles)

if __name__ == '__main__':
    
    import Perception_socket_version

    listener = Perception_socket_version.Listener()

    time.sleep(5)

    while True:
        pose = listener.get_current_pose()
        obs = listener.get_obstacles()
        #print("\npose:\nx:", pose[0])
        #print("y:", pose[1])
        #print("z:", listener.pose[2]/1000)
        #print("yaw:", pose[2])
        #print("velocity:", pose[3])
        #print(len(obs), " obs dected.")
        for id in obs:
             print("obs:", id, "\n")
             time.sleep(1)

