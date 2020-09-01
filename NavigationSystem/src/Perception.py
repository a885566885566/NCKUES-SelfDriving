#!/usr/bin/env python3
import rospy
import numpy as np
import math
from zed_interfaces.msg import Objects
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
#from tf.transformations import euler_from_quaternion 

radius_table={
            'Person':1,
            }
label_table={
        'Person':1
        }

class Listener():
    def __init__(self):
        self.output = {}
        self.position = np.zeros(5)
        self.sub=rospy.Subscriber("/zed2/zed_node/obj_det/objects", Objects, self.echo)
        self.pos_sub=rospy.Subscriber("/zed2/zed_node/pose", PoseStamped, self.pose_callback)
    #get the position of collision from ROS 
    def echo(self, data):
        for i in range(len(data.objects)):
            label_id = data.objects[i].label_id
            if not label_id in self.output:
                self.output[label_id] = {}
            self.output[label_id]['label'] = data.objects[i].label
            self.output[label_id]['x'] = data.objects[i].position.x
            self.output[label_id]['y'] = data.objects[i].position.y
            self.output[label_id]['z'] = data.objects[i].position.z
            self.output[label_id]['update'] = rospy.get_rostime().secs
            
        #use the copy of output to avoid changing size of dictionary during iteration 
        for label_id in self.output.copy():
            if rospy.get_rostime().secs - self.output[label_id]['update'] > 1: #limit_time
                del self.output[label_id]
        #rospy.loginfo(self.output) 
    
    #get the position of ZED2 camera from ROS(without coordinate transtormation)
    def pose_callback(self, data):
        self.position[0]=data.pose.position.x
        self.position[1]=data.pose.position.y        
        self.position[2]=data.pose.position.z
        euler=self.quaternion_to_euler(data.pose.orientation.x, 
                data.pose.orientation.y,
                data.pose.orientation.z, 
                data.pose.orientation.w)
        self.position[3]=euler[2]*57.29
        self.position[4]=rospy.get_rostime().secs
        #rospy.loginfo(self.position)
    
    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [yaw, pitch, roll]

    
    def get_current_collision(self):
        collision = []
        for label_id in self.output:
            temp = [label_id,
                label_table[self.output[label_id]['label']],
                self.output[label_id]['x'],
                self.output[label_id]['z'],
                self.output[label_id]['update'],
                radius_table[self.output[label_id]['label']]
                ]
            collision.append(temp)  
        return np.array(collision)

    def get_current_position(self):
        return np.array(self.position)
#import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt

if __name__ == '__main__': 
    rospy.init_node('listener', anonymous=True)
    objectlist=Listener()  
    #rospy.spin()
    
    for i in range(100):
        rospy.sleep(0.05)
        position = objectlist.get_current_position()
        print(position)
        plt.scatter(position[0], position[2])
        plt.pause(0.001)
        #rospy.spin()

    
    
    '''    
    for i in range(5):
        rospy.sleep(1) #try to subscribe with delay
        rospy.loginfo(rospy.get_rostime().secs)
        collision=objectlist.get_current_collision()
        rospy.loginfo(collision)
    '''
    
        
    

