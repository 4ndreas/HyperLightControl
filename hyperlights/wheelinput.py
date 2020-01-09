import pybullet
import math
import socket
import os
import threading

import numpy as np
import socket , pickle
import json
import time

# this is the wheel feedback thread 
# position feedback is send via udp on a special port

class wheelInput:

    def __init__(self):

        self.wheel_ip = "192.168.2.82"
        self.wheel_port = 50582
        self.wheel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.angle = 0.0
        self.alive = 0
        self.running = True
    

    def inputDataLoop(self):
        self.wheel_sock.settimeout(5.0)

        while (self.running == True):
            

            try:
                (data, addr) = self.wheel_sock.recvfrom(10240)
          
                # print(data)

                strData = data.decode("utf-8")
                a = strData.split()
                ai = int(a[1]) + 25
                # offset = math.pi/2 * 3
                # self.angle = -(float(a[1]) / 4096.0) * (math.pi *2) + offset
                offset = math.pi
                self.angle = -(float(ai) / 4096.0) * (math.pi *2) + offset
                # strData = str(data.decode("utf-8"))
                # print("wheel data")
                # print("angle: %i - %f" % (ai ,self.angle))

            except Exception as e:
                # print.error(traceback.format_exc())
                if str(e) != "timed out":
                    print(e)
    
            self.alive += 1
            if self.alive > 5:
                self.running = False
    
        print("end wheel input thread")

    def startInput(self):

        self.wheel_sock.bind(('',self.wheel_port))   
        x = threading.Thread(target=self.inputDataLoop )
        x.start()
        print("starting input Thread")

    def endInput(self):
        if self.running:
            self.running = False
            packet = "close"
            self.wheel_sock.sendto(bytes(packet), ("127.0.0.1", self.wheel_port))

