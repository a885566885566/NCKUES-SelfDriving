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
                              0 ]
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
    
    def quaternion_to_yaw(self, x, y, z, w):
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def get_current_pose(self):
        return numpy.array([self.pose[0], self.pose[1], self.pose[3], self.pose[4]])

    def get_obstacles(self):
        obstacles = []
        for label_id in self.obstacles:
            temp = [self.obstacles[label_id]['x'],
                    self.obstacles[label_id]['y'],
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
        print("pose:\nx:", pose[0])
        print("y:", pose[1])
        print("yaw:", pose[2])
        print("velocity:", pose[3])
        print("obs:\n", obs)

