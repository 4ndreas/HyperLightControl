import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet
import pybullet_data
import time
import math
import os
import atexit
import threading
# import concurrent.futures
# from concurrent.futures import ThreadPoolExecutor 
# from concurrent.futures import ProcessPoolExecutor 
# from multiprocessing import Pool
import numpy as np
import socket
import copy

# this is the main file

p.connect(p.GUI,"localhost", 7234)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

def main():
    while (p.isConnected()):
        time.sleep(1. / 40.)  # set to 40 fps
        p.stepSimulation()


if __name__ == "__main__":
  main()