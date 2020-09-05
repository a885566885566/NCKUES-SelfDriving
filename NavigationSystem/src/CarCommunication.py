#!/usr/bin/env python
# coding: utf-8

import can	# Import python-can module
import time
import struct

class CanHandle(can.Listener):
    def __init__(self):
        self.send_period = 1
        self.num_msg = 4


        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
        self.recv_msg = can.Message(arbitration_id=0, data=[0,0,0,0,0,0,0,0], is_extended_id=False)
        self.tran_msg = []
        for i in range(self.num_msg):
            self.tran_msg.append(can.Message(arbitration_id=0x30 + i, data=[0,0,0,0,0,0,0,0], is_extended_id=False))

    def start(self):
        for i in range(self.num_msg):
            #self.bus.send_periodic(self.tran_msg[i], self.send_period)
            pass

    def send_msg(self, msg):
        for i in range(self.num_msg):
            self.tran_msg[i].data = bytearray(struct.pack("d", msg[i]))
            self.bus.send(self.tran_msg[i])
            

    def close(self):
        """
        Close all bus task
        """
        self.bus.stop_all_periodic_tasks()
        self.bus.shutdown()

    def on_message_received(self, msg):
        # Abstract function provided by Listener class
        self.recv_msg = msg
        print("Recv msg")

def main():
    can = CanHandle()
    msg = [6.3, 5.2, 3.7, 4.4]
    can.set_msg(msg)
    can.start()
    for i in range(20):
        print("Set")
        for j in range(4):
            msg[j] = j + 0.1*i
        can.send_msg(msg)
        print(msg)
        time.sleep(0.5)
    can.close()

if __name__ == "__main__":
    main()
