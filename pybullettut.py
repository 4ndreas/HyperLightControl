
import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,0)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
# trigger1 = p.loadURDF("cube.urdf", [2, 2, 5])

shift = [0, -0.02, 0]
meshScale = [0.1, 0.1, 0.1]

triggerCube = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="cube.urdf",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shift,
                                    meshScale=meshScale)

visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                    radius=5,
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0]
                                    )

for i in range (10000):
    p.stepSimulation()
    res = p.getOverlappingObjects(triggerCube, triggerCube)
    print(res)
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
