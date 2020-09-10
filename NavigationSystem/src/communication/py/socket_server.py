# -*- coding: utf-8 -*-
import socket
import struct

MSG_POSE_LEN = 56
MSG_OBS_LEN = 52

def convert_pose(data):
    msg = struct.unpack('ddddddd', data)
    info = {}
    info['mode'] = "POSE"
    info['x'] =  msg[0]
    info['y'] =  msg[1] 
    info['z'] =  msg[2] 
    info['ox'] = msg[3] 
    info['oy'] = msg[4] 
    info['oz'] = msg[5] 
    info['ow'] = msg[6] 
    return info

def convert_obs(data):
    msg_num = len(data) // MSG_OBS_LEN
    obs = []
    for i in range(0, msg_num):
        info = {}
        s = struct.unpack('iidddddd', msg[MSG_OBS_LEN*i: MSG_OBS_LEN*(i+1)])
        info['mode']  = "OBS"
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
        self.server.bind((HOST, PORT))
        self.server.listen(10)

   
    def convert_msg(data):
        if len(data) == MSG_POSE_LEN:
            # MSG_POSE
            return convert_pose(data)
        elif len(data) % MSG_OBS_LEN == 0:
            # MSG_OBS
            return convert_obs(data)
        else:
            print("Bad message")
            return 
    
    def check(self):
        conn, addr = server.accept()
        raw = conn.recv(1024)
        msg = convert_msg(raw)
        serverMessage = 'ok'
        conn.sendall(serverMessage.encode())
        conn.close()
        return msg
