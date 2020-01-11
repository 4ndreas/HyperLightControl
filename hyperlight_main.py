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
p.connect(p.GUI_SERVER)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.resetDebugVisualizerCamera( cameraDistance=10, cameraYaw=90, cameraPitch=-80, cameraTargetPosition=[0,20,25]) # look on a1
# p.resetDebugVisualizerCamera( cameraDistance=50, cameraYaw=45, cameraPitch=-60, cameraTargetPosition=[0,-20,25])
# p.resetDebugVisualizerCamera( cameraDistance=50, cameraYaw=45, cameraPitch=-60, cameraTargetPosition=[0,-20,25])
p.resetDebugVisualizerCamera( cameraDistance=1, cameraYaw=270, cameraPitch=-10, cameraTargetPosition=[-50,0,15]) # look on wheel

from hyperlights.hyperlights import *

from hyperlights.collider import Collider
from hyperlights.artnetOut import artnetOut
from hyperlights.inputMidi import inputMidi
from hyperlights.wheelinput import wheelInput
from hyperlights.comm import inputData

aOutput = artnetOut()
mInput = inputMidi()
wInput = wheelInput()

# create room
planeId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)

# lists to store the objects 
lights = []
colliders =[]

#create colliders

collider1 = Collider()
collider1.enabled = False
collider1.z = 15
collider1.b_step = 0.20
collider1.b_max = 65
collider1.b_min = -65
collider1.create(p,collider1.dirpath + "coll_full.urdf")
colliders.append(collider1)

collider2 = Collider()
collider2.z = 0
collider2.c_step = 0.05
collider2.c_max = 35
collider2.c_min = -5
collider2.create(p,collider2.dirpath + "coll_full_2.urdf")
colliders.append(collider2)

#create Lights
# einheit dm = 0.1m??

hlWheel = HlWheel()
hlWheel.x = -40
hlWheel.y = 0
hlWheel.z = 10
hlWheel.universe = 0
hlWheel.ip = "192.168.2.82"
hlWheel.onColor_r = 0
hlWheel.onColor_g = 255
hlWheel.onColor_b = 0
hlWheel.create(p)
lights.append(hlWheel)

# maiskolben_1 = HlMaiskoblen()
# maiskolben_1.x = 15
# maiskolben_1.y = -51.5
# maiskolben_1.z = 25
# maiskolben_1.create(p)
# maiskolben_1.universe = 7
# maiskolben_1.onColor_r = 0
# maiskolben_1.onColor_g = 255
# maiskolben_1.onColor_b = 0
# lights.append(maiskolben_1)

# maiskolben_2 = HlMaiskoblen()
# maiskolben_2.x = 15
# maiskolben_2.y = -38.5
# maiskolben_2.z = 25
# maiskolben_2.universe = 0
# maiskolben_2.create(p)
# maiskolben_2.onColor_r = 0
# maiskolben_2.onColor_g = 255
# maiskolben_2.onColor_b = 0
# lights.append(maiskolben_2)

# maiskolben_3 = HlMaiskoblen()
# maiskolben_3.x = 15
# maiskolben_3.y = -25.5
# maiskolben_3.z = 25
# maiskolben_3.universe = 2
# maiskolben_3.onColor_r = 0
# maiskolben_3.onColor_g = 255
# maiskolben_3.onColor_b = 0
# maiskolben_3.create(p)
# lights.append(maiskolben_3)

# maiskolben_4 = HlMaiskoblen()
# maiskolben_4.x = 15
# maiskolben_4.y = -12.5
# maiskolben_4.z = 25
# maiskolben_4.universe = 3
# maiskolben_4.onColor_r = 0
# maiskolben_4.onColor_g = 255
# maiskolben_4.onColor_b = 0
# maiskolben_4.create(p)
# lights.append(maiskolben_4)

# maiskolben_5 = HlMaiskoblen()
# maiskolben_5.x = 15
# maiskolben_5.y = 0.5
# maiskolben_5.z = 25
# maiskolben_5.universe = 1
# maiskolben_5.onColor_r = 0
# maiskolben_5.onColor_g = 255
# maiskolben_5.onColor_b = 0
# maiskolben_5.create(p)
# lights.append(maiskolben_5)

# PPPanel_1 = HlPPPanel()
# PPPanel_1.x = 10
# PPPanel_1.y = -55
# PPPanel_1.z = 25
# PPPanel_1.universe = 8
# PPPanel_1.onColor_r = 0
# PPPanel_1.onColor_g = 255
# PPPanel_1.onColor_b = 0
# PPPanel_1.create(p)
# lights.append(PPPanel_1)

# PPPanel_2 = HlPPPanel()
# PPPanel_2.x = -10
# PPPanel_2.y = -55
# PPPanel_2.z = 25
# PPPanel_2.universe = 10
# PPPanel_2.onColor_r = 0
# PPPanel_2.onColor_g = 255
# PPPanel_2.onColor_b = 0
# PPPanel_2.create(p)
# lights.append(PPPanel_2)

# ledGrid = []

# newMod = HlGridR()
# newMod.x = -20
# newMod.y = -19.5
# newMod.z = 25
# newMod.universe = 0
# newMod.ip = "192.168.2.81"
# newMod.onColor_r = 255
# newMod.onColor_g = 128
# newMod.onColor_b = 0
# ledGrid.append(newMod)

# for j in range(1,7):
#   newMod2 = HlGrid()
#   newMod2.x = -20
#   newMod2.y = -19.5
#   newMod2.z = 25
#   newMod2.universe = 0
#   newMod2.ip = "192.168.2.81"
#   newMod2.onColor_r = 255
#   newMod2.onColor_g = 128
#   newMod2.onColor_b = 0
#   newMod2.y += j * 6.5
#   newMod2.universe = j
#   ledGrid.append(newMod2)

# newMod3 = HlGridR()
# newMod3.x = -20
# newMod3.y = 26
# newMod3.z = 25
# newMod3.universe = 7
# newMod3.ip = "192.168.2.81"
# newMod3.onColor_r = 255
# newMod3.onColor_g = 128
# newMod3.onColor_b = 0
# ledGrid.append(newMod3)

# for ledMod in ledGrid :
#   ledMod.create(p) # this have to be done here or it will not work...
#   lights.append(ledMod)

# A1struct = Hl_A1()
# A1struct.x = 0
# A1struct.y = 20
# A1struct.z = 25
# A1struct.create(p)
# A1struct.universe = 1
# A1struct.onColor_r = 0
# A1struct.onColor_g = 255
# A1struct.onColor_b = 0
# lights.append(A1struct)

# starCoreLight = HlStarLight()
# starCoreLight.x = 0
# starCoreLight.y = -45
# starCoreLight.z = 25
# starCoreLight.ip= "192.168.2.60"
# starCoreLight.universe = 3
# starCoreLight.create(p)
# starCoreLight.onColor_r = 0
# starCoreLight.onColor_g = 255
# starCoreLight.onColor_b = 0
# lights.append(starCoreLight)

# starCoreRed = HlStarCore()
# starCoreRed.x = 15
# starCoreRed.y = 45
# starCoreRed.z = 25
# starCoreRed.ip= "192.168.2.62"
# starCoreRed.universe = 2
# starCoreRed.onColor_r = 0
# starCoreRed.onColor_g = 255
# starCoreRed.onColor_b = 0
# starCoreRed.create(p)
# lights.append(starCoreRed)

# starCoreBlue = HlStarCore()
# starCoreBlue.x = -15
# starCoreBlue.y = 45
# starCoreBlue.z = 25
# starCoreBlue.ip= "192.168.2.63"
# starCoreBlue.universe = 2
# starCoreBlue.onColor_r = 0
# starCoreBlue.onColor_g = 255
# starCoreBlue.onColor_b = 0
# starCoreBlue.create(p)
# lights.append(starCoreBlue)


def getRayFromTo(mouseX, mouseY):

  width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera(
  )
  camPos = [
      camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
      camTarget[2] - dist * camForward[2]
  ]
  farPlane = 10000
  rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
  invLen = farPlane * 1. / (math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] *
                                      rayForward[1] + rayForward[2] * rayForward[2]))
  rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
  rayFrom = camPos
  oneOverWidth = float(1) / float(width)
  oneOverHeight = float(1) / float(height)
  dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
  dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
  rayToCenter = [
      rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
  ]
  rayTo = [
      rayToCenter[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] -
      float(mouseY) * dVer[0], rayToCenter[1] - 0.5 * horizon[1] + 0.5 * vertical[1] +
      float(mouseX) * dHor[1] - float(mouseY) * dVer[1], rayToCenter[2] - 0.5 * horizon[2] +
      0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
  ]
  return rayFrom, rayTo

def endThreads():
  aOutput.endOut()
  mInput.endInput()
  wInput.endInput()

def doCollision(p, col, light):
  light.doCollisionFilter(p,col)


def main():
  p.setRealTimeSimulation(1)
  # p.setGravity(0, 0, -10) # we don't use gravity yet

  atexit.register(endThreads)
  
  for light in lights:
      print("universe %d" % light.universe)

  aOutput.startOut(p, lights)
  mInput.startInput(lights, colliders)
  wInput.startInput()

  # debug colors only for show
  colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
  currentColor = 0

  try:
    while (p.isConnected()):
      time.sleep(1. / 40.)  # set to 40 fps
      # reset output timeout, hack for threads...
      aOutput.alive = 0 
      mInput.alive = 0
      wInput.alive = 0

      # update the wheel feedback
      hlWheel.setRotation(p,wInput.angle ) 

      # this is the main part of the sim
      p.stepSimulation()
      for col in colliders:
        col.simulate(p)
        if col.enabled : 
          for light in lights:
            light.doCollisionFilter(p,col)
      
      # get mouse events 
      mouseEvents = p.getMouseEvents()
      for e in mouseEvents:
        if ((e[0] == 2) and (e[3] == 0) and (e[4] & p.KEY_WAS_TRIGGERED)):
          mouseX = e[1]
          mouseY = e[2]
          rayFrom, rayTo = getRayFromTo(mouseX, mouseY)
          rayInfo = p.rayTest(rayFrom, rayTo)
          for l in range(len(rayInfo)):
            hit = rayInfo[l]
            objectUid = hit[0]
            jointUid = hit[1]
            if (objectUid >= 1):
              # this is for debug click on an object to get id 
              # oject will change color this has no effect
              # changing color real time seems to slow
              print("obj %i joint %i" % (objectUid , jointUid))
              p.changeVisualShape(objectUid, jointUid, rgbaColor=colors[currentColor])
              currentColor += 1
              if (currentColor >= len(colors)):
                currentColor = 0

  except KeyboardInterrupt:
    print("KeyboardInterrupt has been caught.")
  finally:
    endThreads()

if __name__ == "__main__":
  main()