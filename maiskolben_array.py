import pybullet as p
import time
p.connect(p.GUI)

planeId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
col = p.loadURDF("cube_collisionfilter.urdf", [0, 3, 3], useMaximalCoordinates=False)
col2 = p.loadURDF("colCube.urdf", [0, 3, 15], useMaximalCoordinates=False)


def createMaiskolben( x, y, z, m_list, m_hit):
    for i in range(10):
        h = i * 0.4 + z
        pixel_1 = p.loadURDF("pixel.urdf", [0.52263+x, 0+y, 3+h], useMaximalCoordinates=False)
        pixel_2 = p.loadURDF("pixel.urdf", [-0.52263+x, 0+y, 3+h], useMaximalCoordinates=False)
        pixel_3 = p.loadURDF("pixel.urdf", [0+x, 0.52263+y, 3+h], useMaximalCoordinates=False)
        pixel_4 = p.loadURDF("pixel.urdf", [0+x, -0.52263+y, 3+h], useMaximalCoordinates=False)
        pixel_5 = p.loadURDF("pixel.urdf", [0.36995+x, 0.36995+y, 3+h], useMaximalCoordinates=False)
        pixel_6 = p.loadURDF("pixel.urdf", [-0.36995+x, -0.36995+y, 3+h], useMaximalCoordinates=False)
        pixel_7 = p.loadURDF("pixel.urdf", [0.36995+x, -0.36995+y, 3+h], useMaximalCoordinates=False)
        pixel_8 = p.loadURDF("pixel.urdf", [-0.36995+x, 0.36995+y, 3+h], useMaximalCoordinates=False)

        m_list.append(pixel_1)
        m_list.append(pixel_2)
        m_list.append(pixel_3)
        m_list.append(pixel_4)
        m_list.append(pixel_5)
        m_list.append(pixel_6)
        m_list.append(pixel_7)
        m_list.append(pixel_8)
        
    j = 0
    for pixel in m_list:
        body = p.getBodyUniqueId(pixel)
        while len(m_hit) < body:
            m_hit.append(j)
    m_hit.append(j)

maiskolben_1 =[]
maiskolben_1_hit = []

maiskolben_2 =[]
maiskolben_2_hit = []

maiskolben_3 =[]
maiskolben_3_hit = []


createMaiskolben( 0,0,0, maiskolben_1 , maiskolben_1_hit)
createMaiskolben( 5,0,0, maiskolben_2 , maiskolben_2_hit)
createMaiskolben( -5,0,0, maiskolben_3 , maiskolben_3_hit)

collisionFilterGroup = 0
collisionFilterMask = 0

p.setCollisionFilterGroupMask(col, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(col2, -1, collisionFilterGroup, collisionFilterMask)

collisionFilterGroup = 0
collisionFilterMask = 0

enableCollision = 1

p.setCollisionFilterPair(planeId, col, -1, -1, enableCollision)


p.setRealTimeSimulation(1)
# p.setGravity(0, 0, -10)

cid = p.createConstraint(col, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
cid2 = p.createConstraint(col2, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])

a = 0
b = 0

while (p.isConnected()):
  time.sleep(1. / 240.)


  for pixel in maiskolben_1:
    res = p.getClosestPoints(col2,pixel,0.1 )
    body = p.getBodyUniqueId(pixel)
    if res:
      maiskolben_1_hit[body] = 1
    else:
      maiskolben_1_hit[body] = 0

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
