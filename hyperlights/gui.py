import pybullet
import pybullet_data

import pybullet as p
import pybullet_utils.bullet_client as bc
import time
import math
import os
import threading
import numpy as np
import socket
import copy

# p.connect(p.SHARED_MEMORY)
# # p.connect(p.GUI)

# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

# p.resetDebugVisualizerCamera( cameraDistance=50, cameraYaw=45, cameraPitch=-60, cameraTargetPosition=[0,-20,25])

#can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER
pgui = bc.BulletClient(connection_mode=pybullet.UDP)

while (pgui.isConnected()):
  time.sleep(1. / 240.)



# import pybullet_utils.bullet_client as bc
# import pybullet
# import pybullet_data

# p0 = bc.BulletClient(connection_mode=pybullet.DIRECT)
# p0.setAdditionalSearchPath(pybullet_data.getDataPath())

# p1 = bc.BulletClient(connection_mode=pybullet.DIRECT)
# p1.setAdditionalSearchPath(pybullet_data.getDataPath())

# #can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER
# pgui = bc.BulletClient(connection_mode=pybullet.GUI)

# p0.loadURDF("r2d2.urdf")
# p1.loadSDF("stadium.sdf")
# print(p0._client)
# print(p1._client)
# print("p0.getNumBodies()=",p0.getNumBodies())
# print("p1.getNumBodies()=",p1.getNumBodies())
# while (1):
#         p0.stepSimulation()
#         p1.stepSimulation()
#         pgui.stepSimulation()