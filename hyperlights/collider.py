import pybullet
import math
import os

# this script contains the colliders

class Collider :
    def __init__(self):
        self.enabled = True
        self.dirpath = os.getcwd() + "\\data\\"
        # Positon
        self.x = 0
        self.y = 0
        self.z = 0
        # rotation (euler)
        self.r = 0
        self.p = 0
        self.w = 0

        # movement
        self.a = 0
        self.b = 0
        self.c = 0
        self.orn = [0,0,0,0]
        self.pivot = [0, 0, 10]
        self.a_step = 0.0
        self.a_min = -80
        self.a_max = 80

        self.b_step = 0.0
        self.b_min = -80
        self.b_max = 80

        self.c_step = 0.0
        self.c_min = -80
        self.c_max = 80

        self.max_force = 500

        self.a_step_ = 0.0
        self.b_step_ = 0.0
        self.c_step_ = 0.0

        # pybullet object id
        self.col = -1
        # pybullet constrain id
        self.cid = -1

        self.collisionFilterGroup = 0
        self.collisionFilterMask = 0
        # 

    # def scale(self, s_x, s_y, s_z):
        # not supported by pybullet

    def setSpeed(self, s_x, s_y, s_z):
        self.a_step = self.a_step_ * s_x
        self.b_step = self.b_step_ * s_y
        self.c_step = self.c_step_ * s_z
    
    def simulate(self, p):
        # if enabled:
        reset = False
        self.a = self.a + self.a_step
        self.b = self.b + self.b_step
        self.c = self.c + self.c_step

        self.pivot = [self.x + self.a, self.y + self.b, self.z + self.c]
        if (self.a > self.a_max):
            self.a = self.a_min
            reset = True

        if (self.b > self.b_max):
            self.b = self.b_min
            reset = True

        if (self.c > self.c_max):
            self.c = self.c_min
            reset = True            

        if reset:
            p.resetBasePositionAndOrientation(self.col, [self.x + self.a, self.y + self.b, self.z + self.c], self.orn)

        p.changeConstraint(self.cid, self.pivot, jointChildFrameOrientation=self.orn, maxForce=self.max_force)


    def create(self, p , model):

        self.a_step_ = self.a_step
        self.b_step_ = self.b_step
        self.c_step_ = self.c_step

        self.pivot = [self.x, self.y, self.z]
        self.orn = p.getQuaternionFromEuler([self.r, self.p, self.w])
        self.col = p.loadURDF(model, [self.x, self.y, self.z], useMaximalCoordinates=False)
        p.setCollisionFilterGroupMask(self.col, -1, self.collisionFilterGroup, self.collisionFilterMask)
        # p.changeVisualShape(self.col, -1, rgbaColor=[100, 100, 100, 100])
        
        self.cid = p.createConstraint(self.col, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
        p.changeConstraint(self.cid, self.pivot, jointChildFrameOrientation=self.orn, maxForce=self.max_force)
