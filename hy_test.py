import pybullet as p
import time
import math

p.connect(p.GUI)

# p.resetDebugVisualizerCamera( cameraDistance=10, cameraYaw=90, cameraPitch=-80, cameraTargetPosition=[0,20,25]) # look on a1
# p.resetDebugVisualizerCamera( cameraDistance=50, cameraYaw=45, cameraPitch=-60, cameraTargetPosition=[0,-20,25])
p.resetDebugVisualizerCamera( cameraDistance=50, cameraYaw=45, cameraPitch=-60, cameraTargetPosition=[0,-45,25])


from hyperlights.hyperlights import Hyperlights
from hyperlights.hyperlights import HlMaiskoblen
from hyperlights.hyperlights import HlStarCore
from hyperlights.hyperlights import HlStarLight
from hyperlights.hyperlights import Hl_A1
from hyperlights.hyperlights import HlGrid
from hyperlights.hyperlights import HlPPPanel

from hyperlights.collider import Collider


planeId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
# col = p.loadURDF("cube_collisionfilter.urdf", [0, 3, 3], useMaximalCoordinates=False)
# col = p.loadURDF("coll_full.urdf", [0, 0, 15], useMaximalCoordinates=False)

collider1 = Collider()
collider1.z = 15
collider1.create(p,"coll_full.urdf")

# col2 = p.loadURDF("colCube.urdf", [0, 3, 3], useMaximalCoordinates=False)

# einheit dm = 0.1m??

# maiskolben_1 = HlMaiskoblen()
# maiskolben_1.x = 15
# maiskolben_1.y = -51.5
# maiskolben_1.z = 25
# maiskolben_1.create(p)

# maiskolben_2 = HlMaiskoblen()
# maiskolben_2.x = 15
# maiskolben_2.y = -38.5
# maiskolben_2.z = 25
# maiskolben_2.create(p)

# maiskolben_3 = HlMaiskoblen()
# maiskolben_3.x = 15
# maiskolben_3.y = -25.5
# maiskolben_3.z = 25
# maiskolben_3.create(p)

# maiskolben_4 = HlMaiskoblen()
# maiskolben_4.x = 15
# maiskolben_4.y = -12.5
# maiskolben_4.z = 25
# maiskolben_4.create(p)

# maiskolben_5 = HlMaiskoblen()
# maiskolben_5.x = 15
# maiskolben_5.y = 0.5
# maiskolben_5.z = 25
# maiskolben_5.create(p)

# PPPanel_1 = HlPPPanel()
# PPPanel_1.x = -10
# PPPanel_1.y = -60
# PPPanel_1.z = 25
# PPPanel_1.create(p)


# PPPanel_2 = HlPPPanel()
# PPPanel_2.x = 10
# PPPanel_2.y = -60
# PPPanel_2.z = 25
# PPPanel_2.create(p)

Grid = HlGrid()
Grid.x = -20
Grid.y = 0
Grid.z = 25
Grid.create(p)

A1struct = Hl_A1()
A1struct.x = 0
A1struct.y = 20
A1struct.z = 25
A1struct.create(p)

starCoreLight = HlStarLight()
starCoreLight.x = 0
starCoreLight.y = -45
starCoreLight.z = 25


starCoreRed = HlStarCore()
starCoreRed.x = 15
starCoreRed.y = 45
starCoreRed.z = 25

starCoreBlue = HlStarCore()
starCoreBlue.x = -15
starCoreBlue.y = 45
starCoreBlue.z = 25

starCoreLight.create(p)
starCoreRed.create(p)
starCoreBlue.create(p)


collisionFilterGroup = 0
collisionFilterMask = 0

# p.setCollisionFilterGroupMask(col, -1, collisionFilterGroup, collisionFilterMask)
# p.setCollisionFilterGroupMask(col2, -1, collisionFilterGroup, collisionFilterMask)

collisionFilterGroup = 0
collisionFilterMask = 0


enableCollision = 1

# p.setCollisionFilterPair(planeId, col, -1, -1, enableCollision)
# p.setCollisionFilterPair(planeId, col2, -1, -1, enableCollision)


p.setRealTimeSimulation(1)
# p.setGravity(0, 0, -10) 

# a = 0
# b = 0
# orn = p.getQuaternionFromEuler([0, 0, 0])
# pivot = [0, 0, 10]


# cid = p.createConstraint(col, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
# p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=500)

# cid2 = p.createConstraint(col2, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])


while (p.isConnected()):
  time.sleep(1. / 240.)

  # maiskolben_1.doCollisionFilter(p,col2)
  # print(maiskolben_1.hit)

  # starCoreRed.doCollisionFilter(p,col2)
  # print(starCoreRed.hit)

#  trigger
  # a = a + 0.2
  # pivot = [0, a, 15]
  
  # if (a > 80):
  #   a = -80
  #   # cid.btTransform([0, 0, -1])
  #   # resetBasePositionAndOrientation
  #   p.resetBasePositionAndOrientation(cid, [0, -80, 15], orn)
  #   # p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=5000)


  # p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=500)

  collider1.simulate(p)

  # b = b + 0.02
  # pivot = [0, 0, 5.5]
  # orn2 = p.getQuaternionFromEuler([0, 0, b])
  

  # p.changeConstraint(cid2, pivot, jointChildFrameOrientation=orn2, maxForce=50)

  p.stepSimulation()

#   p.setGravity(0, 0, -10)
