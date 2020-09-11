#!/usr/bin/env python
# coding: utf-8

# Import python-can module
import can	
import numpy as np
import time
import struct

from enum import Enum

NAVIGATOR_OFFSETED_ID = 0x30
STM_BUFFER_LEN = 5

class Terminator(Enum):
    RESET = 0
    ORIGIN = 1
    UPDATE = 2
    PATH = 3

class CanHandle(can.Listener):
    def __init__(self):
        # Total number of message including terminate message
        self.num_msg = STM_BUFFER_LEN
        
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

        # msg buffer for receiving
        self.recv_msg = can.Message(arbitration_id=0, 
                data=[0,0,0,0], is_extended_id=False)

        # msg buffer for tranciving
        self.tran_msg = []
        for i in range(self.num_msg):
            self.tran_msg.append(can.Message(
                arbitration_id=NAVIGATOR_OFFSETED_ID + i, 
                data=[0,0,0,0], is_extended_id=False))

    """
    Send msg to navigator, the message format require the following format
    msg={
        mode= `Terminator`
        data= 1d numpy_array, the length must be smaller than CAN_BUFFER_LEN
    }
    """
    def send_msg(self, msg):
        for i in range(min(self.num_msg-1, msg['data'].shape[0])):
            self.tran_msg[i].data = bytearray(struct.pack("f", msg['data'][i]))
            self.bus.send(self.tran_msg[i])
        # Terminate message
        self.tran_msg[self.num_msg-1].data = bytearray(struct.pack("I", msg['mode'].value))
        self.bus.send(self.tran_msg[self.num_msg-1])

    """
    Close all bus task and shutdown can bus
    """
    def close(self):    
        self.bus.stop_all_periodic_tasks()
        self.bus.shutdown()

    def on_message_received(self, msg):
        # Abstract function provided by Listener class
        self.recv_msg = msg
        print("Recv msg")

def main():
    can = CanHandle()
    msg = {}
    msg['mode'] = Terminator.PATH
    msg['data'] = np.array([6.3, 5.2, 3.7, 4.4])

    ori = {}
    ori['mode'] = Terminator.ORIGIN
    ori['data'] = np.array([6.3, 5.2, 3.7, 4.4])
    for i in range(20):
        for j in range(4):
            msg['data'][j] = j + 0.1*i
            ori['data'][j] = 0.1*j + i
        can.send_msg(msg)
        can.send_msg(ori)
        print(msg)
        print(ori)
        time.sleep(0.5)
    time.sleep(10)
    can.close()

if __name__ == "__main__":
    main()
