import pybullet
import math
import socket
import os
import threading
import numpy as np
import socket , pickle
import json
import time
from hyperlights.comm import inputData

# this is the midi input thread 

class inputMidi:

    def __init__(self):
        self.port = 50505
        self.ip = "127.0.0.1"
        self.input_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._lights = lambda: None
        self._collider = lambda: None
        self.alive = 0
        self.running = True
    
    def inputDataLoop(self):

        # data_variable = inputData()

        while (self.running == True):
            self.input_sock.settimeout(5.0)

            try:
                (data, addr) = self.input_sock.recvfrom(10240)
                # print(data)
                
                # data_string = data.decode("utf-8")
                data_variable = pickle.loads(data)

                # data_variable = pickle.loads(data.decode("utf-8"))
                # data_variable = json.loads(data)
                # print(data_variable)
                # print(str(data_variable.inpID))
            # except:
            #     print("something went wrong")
                # self.alive += 1
                if data_variable.devID == 0:
                    self.alive = 0

                    if data_variable.inpID == 0:
                        if data_variable.inpVal != 0:
                            self._collider[0].enabled = True
                            # print("enable collider 0")
                        else:
                            self._collider[0].enabled = False
                            # print("disable collider 0")
                    if data_variable.inpID == 12:
                        speed = float(data_variable.inpVal) / (12.70 * 3.0 )
                        # print("collider 0 speed %f" % speed)
                        self._collider[0].setSpeed(speed,speed,speed)

                    if data_variable.inpID == 1:
                        if data_variable.inpVal != 0:
                            self._collider[1].enabled = True
                            # print("enable collider 1")
                        else:
                            self._collider[1].enabled = False
                            # print("disable collider 1")
                    if data_variable.inpID == 13:
                        speed = float(data_variable.inpVal) / (12.70 * 3.0 )
                        # print("collider 0 speed %f" % speed)
                        self._collider[1].setSpeed(speed,speed,speed)

                if data_variable.devID == 1:
                    self.alive = 0

                    if data_variable.inpID == 0:
                        if data_variable.inpVal != 0:
                            self._lights[0].enableMotor(True)
                        else:
                            self._lights[0].enableMotor(False)
                    if data_variable.inpID == 1:
                        if data_variable.inpVal != 0:
                            self._lights[0].setMotor1Dir(True)
                        else:
                            self._lights[0].setMotor1Dir(False)                                   

                    if data_variable.inpID == 12:
                        speed = int(data_variable.inpVal) * 2
                        self._lights[0].setMotor1Speed(speed)

            except Exception as e:
                # print.error(traceback.format_exc())
                if str(e) != "timed out":
                    print(e)
    
            self.alive += 1
            if self.alive > 10:
                self.running = False

            # time.sleep(0.015)
        print("end input thread")


    def startInput(self, lights, collider):
        self._lights = lights
        self._collider = collider

        # self.input_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.input_sock.bind(('',self.port))   
        # start_new_thread(self.sendData, ())
        x = threading.Thread(target=self.inputDataLoop )
        x.start()
        print("starting input Thread")

    def endInput(self):
        if self.running:
            self.running = False
            packet = ""
            self.input_sock.sendto(bytes(packet, 'utf-8'), (self.ip, self.port))

