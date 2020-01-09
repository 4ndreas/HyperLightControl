import pybullet as p
import time
p.connect(p.GUI)
planeId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
cubeId = p.loadURDF("maiskolben.urdf", [-3, 0, 3], useMaximalCoordinates=False)
cubeId2 = p.loadURDF("maiskolben.urdf", [3, 0, 3], useMaximalCoordinates=False)
cubeId3 = p.loadURDF("maiskolben.urdf", [3, 3, 3], useMaximalCoordinates=False)
col = p.loadURDF("cube_collisionfilter.urdf", [0, 3, 3], useMaximalCoordinates=False)

collisionFilterGroup = 0
collisionFilterMask = 0

p.setCollisionFilterGroupMask(cubeId, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(cubeId2, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(cubeId3, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(col, -1, collisionFilterGroup, collisionFilterMask)

enableCollision = 1

p.setCollisionFilterPair(planeId, cubeId, -1, -1, enableCollision)
p.setCollisionFilterPair(planeId, cubeId2, -1, -1, enableCollision)
p.setCollisionFilterPair(planeId, cubeId3, -1, -1, enableCollision)
p.setCollisionFilterPair(planeId, col, -1, -1, enableCollision)

p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)


while (p.isConnected()):
  time.sleep(1. / 240.)
  # res = p.getOverlappingObjects(cubeId2, cubeId2)
  aabb = p.getAABB(col)
  res = p.getOverlappingObjects(aabb[0],aabb[1])
  print(res)

  p.stepSimulation()
  p.setGravity(0, 0, -10)
