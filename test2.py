# msg = "new test"

# print(msg)
# msg += " bla"
# print(msg)


import pybullet_utils.bullet_client as bc
import pybullet as p
import pybullet
import pybullet_data


# #can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER
# pgui = bc.BulletClient(connection_mode=pybullet.GUI)



# p0 = bc.BulletClient(connection_mode=pybullet.SHARED_MEMORY_SERVER)
# p0.setAdditionalSearchPath(pybullet_data.getDataPath())



# pgui = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.connect(p.GUI_SERVER )#or p.DIRECT for non-graphical version

# p0 = p.connect(p.SHARED_MEMORY_SERVER )#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p0 = bc.BulletClient(connection_mode=pybullet.SHARED_MEMORY)
# p0.setAdditionalSearchPath(pybullet_data.getDataPath())

# p1 = bc.BulletClient(connection_mode=pybullet.SHARED_MEMORY)
# p1.setAdditionalSearchPath(pybullet_data.getDataPath())

# p1 = p.connect(p.SHARED_MEMORY )#or p.DIRECT for non-graphical version
# p1.setAdditionalSearchPath(pybullet_data.getDataPath())

p0.loadURDF("r2d2.urdf", [0,0,1])
# p1.loadSDF("stadium.sdf")
print(p0._client)
# print(p1._client)
print("p.getNumBodies()=",p.getNumBodies())
print("p0.getNumBodies()=",p0.getNumBodies())
# print("p1.getNumBodies()=",p1.getNumBodies())
while (1):
        # p0.stepSimulation()
        # p1.stepSimulation()
        # pgui.stepSimulation()
        p.stepSimulation()