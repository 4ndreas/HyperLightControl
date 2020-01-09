import pybullet as p
import time
p.connect(p.GUI)

from hyperlights.hyperlights import Hyperlights
from hyperlights.hyperlights import HlMaiskoblen

planeId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
col = p.loadURDF("cube_collisionfilter.urdf", [0, 3, 3], useMaximalCoordinates=False)
col2 = p.loadURDF("colCube.urdf", [0, 3, 3], useMaximalCoordinates=False)

test = HlMaiskoblen()
# test = Maiskolben()
test.x = 5
test.z = 5

test.create(p)

maiskolben_1 =[]
maiskolben_1_hit = []

# def createMaiskolben( x, y, z):
  



# for i in (0,80):
#   pixel
#   maiskolben_id = 
for i in range(10):
  h = i * 0.4
  pixel_1 = p.loadURDF("pixel.urdf", [0.52263, 0, 3+h], useMaximalCoordinates=False)
  pixel_2 = p.loadURDF("pixel.urdf", [-0.52263, 0, 3+h], useMaximalCoordinates=False)
  pixel_3 = p.loadURDF("pixel.urdf", [0, 0.52263, 3+h], useMaximalCoordinates=False)
  pixel_4 = p.loadURDF("pixel.urdf", [0, -0.52263, 3+h], useMaximalCoordinates=False)
  pixel_5 = p.loadURDF("pixel.urdf", [0.36995, 0.36995, 3+h], useMaximalCoordinates=False)
  pixel_6 = p.loadURDF("pixel.urdf", [-0.36995, -0.36995, 3+h], useMaximalCoordinates=False)
  pixel_7 = p.loadURDF("pixel.urdf", [0.36995, -0.36995, 3+h], useMaximalCoordinates=False)
  pixel_8 = p.loadURDF("pixel.urdf", [-0.36995, 0.36995, 3+h], useMaximalCoordinates=False)

  maiskolben_1.append(pixel_1)
  maiskolben_1.append(pixel_2)
  maiskolben_1.append(pixel_3)
  maiskolben_1.append(pixel_4)
  maiskolben_1.append(pixel_5)
  maiskolben_1.append(pixel_6)
  maiskolben_1.append(pixel_7)
  maiskolben_1.append(pixel_8)
  
j = 0
for pixel in maiskolben_1:
  body = p.getBodyUniqueId(pixel)
  while len(maiskolben_1_hit) < body:
    maiskolben_1_hit.append(j)
maiskolben_1_hit.append(j)



collisionFilterGroup = 0
collisionFilterMask = 0

p.setCollisionFilterGroupMask(col, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(col2, -1, collisionFilterGroup, collisionFilterMask)

collisionFilterGroup = 0
collisionFilterMask = 0

p.setCollisionFilterGroupMask(pixel_1, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(pixel_2, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(pixel_3, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(pixel_4, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(pixel_5, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(pixel_6, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(pixel_7, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(pixel_8, -1, collisionFilterGroup, collisionFilterMask)

enableCollision = 1

p.setCollisionFilterPair(planeId, col, -1, -1, enableCollision)
# p.setCollisionFilterPair(planeId, col2, -1, -1, enableCollision)

# p.setCollisionFilterPair(planeId, pixel_1, -1, -1, enableCollision)
# p.setCollisionFilterPair(planeId, pixel_2, -1, -1, enableCollision)
# p.setCollisionFilterPair(planeId, pixel_3, -1, -1, enableCollision)
# p.setCollisionFilterPair(planeId, pixel_4, -1, -1, enableCollision)
# p.setCollisionFilterPair(planeId, pixel_5, -1, -1, enableCollision)
# p.setCollisionFilterPair(planeId, pixel_6, -1, -1, enableCollision)
# p.setCollisionFilterPair(planeId, pixel_7, -1, -1, enableCollision)
# p.setCollisionFilterPair(planeId, pixel_8, -1, -1, enableCollision)

p.setRealTimeSimulation(1)
# p.setGravity(0, 0, -10) 

cid = p.createConstraint(col, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
cid2 = p.createConstraint(col2, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])

a = 0
b = 0

while (p.isConnected()):
  time.sleep(1. / 240.)
  # res = p.getOverlappingObjects(cubeId2, cubeId2)

  # aabb = p.getAABB(pixel_1)
  # res = p.getOverlappingObjects(aabb[0],aabb[1])
  # print(res)


  # for test in res:
  #   p.getContactPoints()

  # res = p.getContactPoints(col2)
  # res = p.getClosestPoints(col,pixel_1,0.1 )
  # print(res)

  for pixel in maiskolben_1:
    res = p.getClosestPoints(col2,pixel,0.1 )
    body = p.getBodyUniqueId(pixel)
    if res:
      maiskolben_1_hit[body] = 1
      # p.changeVisualShape(body, -1, rgbaColor=[1, 0, 0, 1])
      # print(body)
    else:
      maiskolben_1_hit[body] = 0
      # p.changeVisualShape(body, -1, rgbaColor=[1, 1, 1, 1])
      # p.changeVisualShape(objectUid, -1, rgbaColor=colors[currentColor])

  # print(maiskolben_1_hit)

  a = a + 0.02
  pivot = [0, 0, a]
  orn = p.getQuaternionFromEuler([0, 0, 1])
  if (a > 10):
    a = -1
    # cid.btTransform([0, 0, -1])
    p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=5000)

  p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=50)

  b = b + 0.02
  pivot = [0, 0, 5.5]
  orn2 = p.getQuaternionFromEuler([0, 0, b])
  

  p.changeConstraint(cid2, pivot, jointChildFrameOrientation=orn2, maxForce=50)
  p.stepSimulation()

#   p.setGravity(0, 0, -10)
