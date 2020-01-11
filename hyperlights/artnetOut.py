import pybullet
import math
import os
import threading
import numpy as np
import socket
import time

# this script contains the artnet output thread

class artnetOut:
    def __init__(self):
        # self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lights = []
        self.running = False
        self.alive = 0
        self.t = threading.Timer(1, self.sendData)

    def sendDataLoop(self):
        while (self.running == True):
            for light in self._lights:
                # print(light.universe)
                # light.doCollisionFilter(p,collider1.col)
                
                light.sendData()
                # light.sendData(self._sock)
                # if light.enable: 
                #     light.createArtnet()
                #     light.make_header()
                #     light.createArtnet()
                #     if light.universe == 0:
                #         self._sock.sendto(bytes(light.packet), (light.ip, light.artnetPort))
            
            self.alive += 1
            if self.alive > 100:
                self.running = False
            time.sleep(1./ 40.0)    # 40 fps
        print("end artnet thread")

    def sendData(self):
        # if(self.running == False):
        print("output data")
            # self.running = True
        for light in self._lights:
            light.createArtnet()
            light.sendData(self._sock)
            # self.running = False
        # self.t.start()

    def startOut(self,p, lights):
        self._lights = lights
        self.running = True
        # start_new_thread(self.sendData, ())
        x = threading.Thread(target=self.sendDataLoop )
        x.start()
        print("starting artnet Output Thread")
        
        # self.t.start()
        # self.t.join()
    
    def endOut(self):
        if self.running:
            self.running = False



