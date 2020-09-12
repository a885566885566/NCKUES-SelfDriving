# -*- coding: utf-8 -*-
import socket
import struct
import time

import signal

MSG_POSE_LEN = 56
MSG_OBS_LEN = 56

def convert_pose(data):
    msg = struct.unpack('ddddddd', data)
    pose = []
    info = {}
    #info['mode'] = "POSE"
    pose.append("POSE")
    info['x'] =  msg[0]
    info['y'] =  msg[1] 
    info['z'] =  msg[2] 
    info['ox'] = msg[3] 
    info['oy'] = msg[4] 
    info['oz'] = msg[5] 
    info['ow'] = msg[6]
    pose.append(info)
    return pose

def convert_obs(data):
    msg_num = len(data) // MSG_OBS_LEN
    obs = []
    obs.append("OBS")
    for i in range(0, msg_num):
        info = {}
        s = struct.unpack('iidddddd', data[MSG_OBS_LEN*i: MSG_OBS_LEN*(i+1)])
        info['id']    = s[0] 
        info['label'] = s[1]
        info['x']     = s[2]
        info['y']     = s[3]
        info['z']     = s[4]
        info['vx']    = s[5]
        info['vy']    = s[6]
        info['vz']    = s[7]
        obs.append(info)
    return obs
 
class Communicator:
    def __init__(self, port=8787):
        self.HOST = '0.0.0.0'    # Bind to any IP address 
        self.PORT = port

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.HOST, self.PORT))
        self.server.setblocking(True)
        self.server.listen(10)

    def convert_msg(self, data):
        if len(data) == MSG_POSE_LEN:
            # MSG_POSE
            return convert_pose(data)
        elif len(data) % MSG_OBS_LEN == 0:
            # MSG_OBS
            return convert_obs(data)
        else:
            print("Bad message")
            print(data)
            print(len(data))
            return 
    
    def check(self):
        msg = None
        conn, addr = self.server.accept()
        raw = conn.recv(1024)
        msg = self.convert_msg(raw)
        serverMessage = 'ok'
        conn.sendall(serverMessage.encode())
        #if msg is not None:
            #print("\nlen of msg:", len(msg))
            #print("\nmsg:", msg,)
            #print("\nmsg[0]:", msg[0])
            #print("\nmsg[1]:", msg[1])
            #print("\nmsg[1]['x']:", msg[1]['x'])
        conn.close()
        return msg

    def handler(self, signum, frame):
        print('Signal handler called with signal', signum)
        self.server.shutdown()
        self.server.close()


if __name__ == '__main__':

    com = Communicator()
    signal.signal(signal.SIGINT, com.handler)
    while True:
        com.check()
        print("check")
        time.sleep(0.1)

