import pybullet
import math

class Collider :
    # Positon
    x = 0
    y = 0
    z = 0
    # rotation (euler)
    r = 0
    p = 0
    w = 0

    # movement
    a = 0
    orn = [0,0,0,0]
    pivot = [0, 0, 10]
    a_step = 0.2
    a_min = -80
    a_max = 80

    b_step = 0.0
    b_min = -80
    b_max = 80

    c_step = 0.0
    c_min = -80
    c_max = 80
    # pybullet id
    cid = -1

    collisionFilterGroup = 0
    collisionFilterMask = 0
    def simulate(self, p):
        self.a = self.a + self.a_step
        self.b = self.b + self.b_step
        self.c = self.c + self.c_step

        self.pivot = [self.x + self.a , self.y + self.b, self.z + self.c]
        
        if (self.a > self.a_max):
            self.a = self.a_min
            p.resetBasePositionAndOrientation(self.cid, [self.x, self.y, self.z], self.orn)

        p.changeConstraint(self.cid, self.pivot, jointChildFrameOrientation=self.orn, maxForce=500)


    def create(self, p , model):
        self.pivot = [self.x, self.y, self.z]
        self.orn = p.getQuaternionFromEuler([self.r, self.p, self.w])
        self.cid = p.loadURDF(model, [self.x, self.y, self.z], useMaximalCoordinates=False)
        p.setCollisionFilterGroupMask(self.cid, -1, self.collisionFilterGroup, self.collisionFilterMask)