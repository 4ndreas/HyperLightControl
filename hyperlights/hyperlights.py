import pybullet
import math
import os
import numpy as np
import socket
from threading import Lock, Thread

#this script contains all the lights

class Hyperlights :
    def __init__(self):
        self.dirpath = os.getcwd() + "\\data\\"
        # print(self.dirpath)

        self.hit = []
        self.pixel = []
        self.pixelNum = 0
        self.pixelMap = []
        self.pixelOffset = 0
        self.numJoints = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.orn = [0,0,0]
        self.colSize = 0.1

        # Color
        self.enable = True
        self.Master = 1.0

        self.onColor_r = 255
        self.onColor_g = 255
        self.onColor_b = 255

        self.offColor_r = 0
        self.offColor_g = 0
        self.offColor_b = 0


        # Artnet part 
        self.update = True
        self.artnetSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip = "127.0.0.1"
        self.universe = 0
        self.artnetPort = 6454
        self.artnetData = []
        self.artnetData2 = []

        self.bIsSimplified = True
        self.TARGET_IP = self.ip
        self.SEQUENCE = 0
        self.PHYSICAL = 0
        self.UNIVERSE = self.universe
        self.SUB = 0
        self.NET = 0
        self.PACKET_SIZE = 512
        self.HEADER = bytearray()
        self.BUFFER = bytearray(self.PACKET_SIZE)

        for i in range(512):
            self.artnetData.append(0)
            self.artnetData2.append(0)

        self.make_header()

    def doCollisionFilter(self, p, obj):
        i = 0

        # _hit =[self.pixelNum]
        if self.enable:

            # lock = Lock()
            # lock.acquire() # will block if lock is already held
            for px in self.pixel:
                # check for subpixel
                subPixelCount = p.getNumJoints(px) 
                if subPixelCount > 0:
                    for j in range(0, subPixelCount):
                        resSub = p.getClosestPoints(obj.col, px,self.colSize, -1, j)
                        if resSub:
                            self.hit[i] = 1
                        else:
                            self.hit[i] = 0
                        i += 1
                else:
                    res = p.getClosestPoints(obj.col, px, self.colSize )
                    # body = p.getBodyUniqueId(px)
                    if i >= self.pixelNum:
                        break
                    hit = 0
                    if res:
                        hit = 1
                    
                    self.hit[i] = hit
                    i += 1
            self.update = True
            # lock.release()

                # self.make_header()
                # self.createArtnet()
                # if(hit != self.hit[i]):
                #     if hit == 1:
                #         self.hit[i] = hit
                #         # p.changeVisualShape(px, -1, rgbaColor=[self.onColor_r, self.onColor_g, self.onColor_b, 255])
                #     else:
                #         self.hit[i] = hit
                        # p.changeVisualShape(px, -1, rgbaColor=[50, 50, 50, 255])


    def sendData(self):
        if self.enable:
            if self.update:
                self.make_header()
                self.createArtnet()
                # self.artnetSocket.sendto(bytes(self.packet), (self.ip, self.artnetPort))

                # print("artnet to %s, uni %i" % (self.ip, self.universe))
                # artnetSocket.sendto(bytes(self.packet), (self.ip, self.artnetPort))

    ##
    # UTILS
    ##
    @staticmethod
    def shift_this(number, high_first=True):
		# """Utility method: extracts MSB and LSB from number.
		# Args:
		# number - number to shift
		# high_first - MSB or LSB first (true / false)
		# Returns:
		# (high, low) - tuple with shifted values
		# """
        low = (number & 0xFF)
        high = ((number >> 8) & 0xFF)
        if (high_first):
            return((high, low))
        else:
            return((low, high))
        print("Something went wrong")
        return False

    def make_header(self, uni=0):
	    # """Make packet header."""
	    # 0 - id (7 x bytes + Null)
        self.HEADER = bytearray()
        self.HEADER.extend(bytearray('Art-Net', 'utf8'))
        self.HEADER.append(0x0)
		# 8 - opcode (2 x 8 low byte first)
        self.HEADER.append(0x00)
        self.HEADER.append(0x50)  # ArtDmx data packet
		# 10 - prototocol version (2 x 8 high byte first)
        self.HEADER.append(0x0)
        self.HEADER.append(14)
		# 12 - sequence (int 8), NULL for not implemented
        self.HEADER.append(self.SEQUENCE)
		# 13 - physical port (int 8)
        self.HEADER.append(0x00)
		# 14 - universe, (2 x 8 low byte first)

        # self.HEADER.append(self.universe)
        # self.HEADER.append(0)
        if (self.bIsSimplified):
			# not quite correct but good enough for most cases:
			# the whole net subnet is simplified
			# by transforming a single uint16 into its 8 bit parts
			# you will most likely not see any differences in small networks
            v = self.shift_this(self.universe + uni)			# convert to MSB / LSB
            # print(self.ip)
            # print(self.universe)
            # print(v)
            self.HEADER.append(v[1])
            self.HEADER.append(v[0])
		# 14 - universe, subnet (2 x 4 bits each)
		# 15 - net (7 bit value)
        else:
			# as specified in Artnet 4 (remember to set the value manually after):
			# Bit 3  - 0 = Universe (1-16)
			# Bit 7  - 4 = Subnet (1-16)
			# Bit 14 - 8 = Net (1-128)
			# Bit 15     = 0
			# this means 16 * 16 * 128 = 32768 universes per port
			# a subnet is a group of 16 Universes
			# 16 subnets will make a net, there are 128 of them
            self.HEADER.append(self.SUB << 4 | self.universe)
            self.HEADER.append(self.NET & 0xFF)
        # 16 - packet size (2 x 8 high byte first)
        v = self.shift_this(self.PACKET_SIZE)		# convert to MSB / LSB
        self.HEADER.append(v[0])
        self.HEADER.append(v[1])

    def createArtnet(self):
        i = self.pixelOffset

        # lock = Lock()
        # lock.acquire() # will block if lock is already held
        # ... access shared resource
        extended = False

        for res in self.hit:
            if i < 510:
                if res == 1:
                    self.artnetData[i] = self.onColor_r
                    self.artnetData[i+1] = self.onColor_g
                    self.artnetData[i+2] = self.onColor_b
                else:
                    self.artnetData[i] = self.offColor_r
                    self.artnetData[i+1] = self.offColor_g
                    self.artnetData[i+2] = self.offColor_b
                i = i+3
            elif i < 1020:
                extended = True
                # print(i)
                if res == 1:
                    self.artnetData2[i-510] = self.onColor_r
                    self.artnetData2[i+1-510] = self.onColor_g
                    self.artnetData2[i+2-510] = self.onColor_b
                else:
                    self.artnetData2[i-510] = self.offColor_r
                    self.artnetData2[i+1-510] = self.offColor_g
                    self.artnetData2[i+2-510] = self.offColor_b
                i = i+3

        self.update = False
        # lock.release()


        self.packet = bytearray()
        self.packet.extend(self.HEADER)
        self.packet.extend(self.artnetData)
        self.artnetSocket.sendto(bytes(self.packet), (self.ip, self.artnetPort))

        if extended:
            self.packet2 = bytearray()
            self.packet2.extend(self.HEADER)
            self.packet2.extend(self.artnetData2)

            v = self.shift_this(self.universe + 1)			# convert to MSB / LSB
            self.packet2[14] = v[1]
            self.packet2[15] = v[0]
            self.artnetSocket.sendto(bytes(self.packet2), (self.ip, self.artnetPort))


    def addPixel(self, p, model, pos, orn):
        self.pixelNum += 1
        pixel_1 = p.loadURDF(model, pos, orn, useMaximalCoordinates=False)
        self.pixel.append(pixel_1)
        # print("pixelID %d" % pixel_1)

    def setCollisionFilterMask(self,p, enableCollision):
        collisionFilterGroup = 0
        collisionFilterMask = 0
        for px in self.pixel:
            p.setCollisionFilterGroupMask(px, -1, collisionFilterGroup, collisionFilterMask)
            # p.setCollisionFilterPair(obj, px, -1, -1, enableCollision)

    def fillHit(self, p):
        j = 0
        # self.hit = []
        

        for px in self.pixel:
            self.numJoints = p.getNumJoints(px)
            # print("numJoints %i " % (px.numJoints))

            # print(p.getJointInfo(px,0)) # extract name?

            # print("add hit px:%d" % px)
            # body = p.getBodyUniqueId(px)
            # pixelMap.append(body)

            # while len(self.hit) <= body:
                #  self.hit.append(j)
            if p.getNumJoints(px) > 0:
                self.pixelNum += p.getNumJoints(px)+1
                for i in range(0, p.getNumJoints(px)+1):
                    self.hit.append(i)
                    # n = p.getJointInfo(px,i)
                    # print("led %i , joint %s" % (i, n[1])) # extract name?

            else:
                self.hit.append(px)
            
            self.setCollisionFilterMask(p,0)
        
        print("pixnum: %d pixel: %d, hit:%d" % (self.pixelNum, len(self.pixel), len(self.hit)))
        # print(self.pixel)

    def create(self, p):
        self.addPixel(p,self.dirpath + "pixel.urdf,", [self.x, self.y, self.z], [0,0,0,0])
        # pixel_1 = p.loadURDF("pixel.urdf", [self.x, self.y, self.z], useMaximalCoordinates=False)
        # self.pixel.append(pixel_1)
        self.fillHit(p)


class HlGrid (Hyperlights):
    # hit = []
    # pixel = []

    def create (self, p):
        self.make_header()

        d = math.pi/4
        orn = p.getQuaternionFromEuler([d, 0, math.pi/2])

        _x = self.x
        _y = self.y
        _z = self.z
        dx = 9.0
        dz = -3.0
        
        # for i in range(-7,7):
        #     # self.addPixel(p,"pixel_Line.urdf", [_x, i * dy + _y, _z], orn)
        #     self.addPixel(p,"tristar_48g.urdf", [_x, i * dy + _y, _z], orn)

        self.addPixel(p,self.dirpath + "tristar_48g.urdf", [_x, _y, _z], orn)
        orn = p.getQuaternionFromEuler([math.pi/2, 0, math.pi/2])
        i = -1        
        self.addPixel(p,self.dirpath + "tristar_48g.urdf", [i * dx + _x, _y, dz + _z], orn)
        i = -2
        self.addPixel(p,self.dirpath + "tristar_48g.urdf", [i * dx + _x,  _y, dz + _z], orn)

        self.fillHit(p)

class HlGridR (Hyperlights):
    def create (self, p):
        self.make_header()

        d = math.pi/4
        orn = p.getQuaternionFromEuler([d, 0, math.pi/2])

        _x = self.x
        _y = self.y
        _z = self.z
        dx = -3.0
        
        

        self.addPixel(p,self.dirpath + "tristar_48g.urdf", [_x, _y, _z], orn)
        orn = p.getQuaternionFromEuler([0, 0, 0])  
        dz = -8.0
        self.addPixel(p,self.dirpath + "tristar_48g.urdf", [dx + _x, _y, dz + _z], orn)
        dz = -16.5
        self.addPixel(p,self.dirpath + "tristar_48g.urdf", [dx + _x,  _y, dz + _z], orn)

        self.fillHit(p)


class HlWheel (Hyperlights):
    def setRotation(self, p, angle):
        _x = self.x
        _y = self.y
        _z = self.z
        d = math.pi/2
        self.orn = p.getQuaternionFromEuler([0,angle,d])
        p.resetBasePositionAndOrientation(self.pixel[0], [_x, _y, _z ], self.orn)

    def enableMotor(self, en):
        if en == True:
            self.enMotor = 1
        else:
            self.enMotor = 0

        self.sendMotor()

    def setMotor1Speed(self, speed):
        self.speedMotor1 = speed
        self.sendMotor()

    def setMotor1Dir(self, dir):
        if dir == True:
            self.dirMotor1 = 1
        else:
            self.dirMotor1 = 0

        self.sendMotor()


    def sendMotor(self):
        packet = bytearray()
        packet.extend(self.HEADER)
        v = self.shift_this(2)
        packet[14] = v[1]
        packet[15] = v[0]
        packet.append(self.enMotor)   # enalbe Motor
        packet.append(self.speedMotor1)   # Motor 1 speed
        packet.append(self.speedMotor2)   # Motor 2 speed
        packet.append(self.dirMotor1 )   # Motor 1 dir
        packet.append(self.dirMotor2 )   # Motor 2 dir
        # print("motor Control")
        # print(packet)

        self.artnetSocket.sendto(bytes(packet), (self.ip, self.artnetPort))

    def create (self, p):

        self.enMotor = 0
        self.speedMotor1 = 0
        self.speedMotor2 = 0
        self.dirMotor1 = 0
        self.dirMotor2 = 0

        self.make_header()

        d = math.pi/2
        self.orn = p.getQuaternionFromEuler([0, 0, d])
        _x = self.x
        _y = self.y
        _z = self.z
        
        self.addPixel(p,self.dirpath + "hlWheel.urdf", [_x, _y, _z], self.orn)
        self.fillHit(p)


class Hl_A1 (Hyperlights):
    # hit = []
    # pixel = []

    def create (self, p):
        self.ip = "192.168.2.73"
        self.universe = 0
        self.make_header()

        self.pxiel = []
        d = 0
        orn = p.getQuaternionFromEuler([d, 0, 0])

        _x = self.x
        _y = self.y
        _z = self.z
        dx = 0.75
        dy = 0.75
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-1 * dx + _x, 1.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 1 * dx + _x, 1.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-1 * dx + _x,-1.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 1 * dx + _x,-1.0 * dy + _y, _z], orn)

        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-2 * dx + _x,-2.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 2 * dx + _x, 0.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-2 * dx + _x, 2.0 * dy + _y, _z], orn)

        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 0 * dx + _x, 2.0 * dy + _y, _z], orn)
        
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 2 * dx + _x, 2.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-2 * dx + _x, 0.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 2 * dx + _x, -2.0 * dy + _y, _z], orn)
        
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 0 * dx + _x,-2.0 * dy + _y, _z], orn)

        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-3 * dx + _x,-3.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-3 * dx + _x,-1.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-3 * dx + _x, 1.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-3 * dx + _x, 3.0 * dy + _y, _z], orn)

        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-1 * dx + _x, 3.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-1 * dx + _x, -3.0 * dy + _y, _z], orn)

        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 3 * dx + _x, 3.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 3 * dx + _x, 1.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 3 * dx + _x,-1.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 3 * dx + _x,-3.0 * dy + _y, _z], orn)

        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [1 * dx + _x, 3.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [1 * dx + _x, -3.0 * dy + _y, _z], orn)


        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 4 * dx + _x, 2.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 4 * dx + _x, 0.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 4 * dx + _x,-2.0 * dy + _y, _z], orn)

        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 5 * dx + _x, 1.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 5 * dx + _x,-1.0 * dy + _y, _z], orn)

        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ 6 * dx + _x,  0.0 * dy + _y, _z], orn)

        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ -4 * dx + _x, 2.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ -4 * dx + _x, 0.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ -4 * dx + _x,-2.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ -5 * dx + _x, 1.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ -5 * dx + _x,-1.0 * dy + _y, _z], orn)
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [ -6 * dx + _x, 0.0 * dy + _y, _z], orn)


        self.fillHit(p)


class HlPPPanel (Hyperlights):
    # hit = []
    # pixel = []

    def create (self, p):
        self.ip = "192.168.2.80"
        self.make_header()

        self.pxiel = []
        orn = [0,0,0,1]
        d = 0.4

        for i in range( -8 , 8):
            h = (-i) * d + self.z
            for j in range(-8,8 ):
                self.addPixel(p,self.dirpath + "pixel.urdf", [j * d + self.x, self.y, h], orn)

        self.fillHit(p)


class HlMaiskoblen (Hyperlights):
    # hit = []
    # pixel = []

    def create (self, p):
        self.ip = "192.168.2.80"
        self.pxiel = []
        self.make_header()

        for i in range(10):
            h = (-i) * 0.4 + self.z + 4
            orn = [0,0,0,1]

            self.addPixel(p,self.dirpath + "pixel.urdf", [0.52263 +  self.x, 0 +self.y, h], orn)
            self.addPixel(p,self.dirpath + "pixel.urdf", [-0.52263+ self.x, 0 + self.y, h], orn)
            self.addPixel(p,self.dirpath + "pixel.urdf", [0 +  self.x, 0.52263 + self.y, h], orn)
            self.addPixel(p,self.dirpath + "pixel.urdf", [0 +  self.x, -0.52263+ self.y, h], orn)
            self.addPixel(p,self.dirpath + "pixel.urdf", [0.36995 + self.x, 0.36995 + self.y, h], orn)
            self.addPixel(p,self.dirpath + "pixel.urdf", [-0.36995+ self.x, -0.36995+ self.y, h], orn)
            self.addPixel(p,self.dirpath + "pixel.urdf", [0.36995 + self.x, -0.36995+ self.y, h], orn)
            self.addPixel(p,self.dirpath + "pixel.urdf", [-0.36995+ self.x, 0.36995 + self.y, h], orn)
        
        self.fillHit(p)


class HlStarCore (Hyperlights):
    # hit = []
    # pixel = []

    def create (self, p):
        self.make_header()
        self.pixelOffset = 1

        self.pxiel = []
        h = self.z
        # White Core
        self.addPixel(p,self.dirpath + "pixel.urdf", [self.x, self.y, h], [0,0,0,1])
        # RGB
 
        offset = 2
        d = math.pi / 2
        rot = math.pi 
        sx = math.sin(math.pi/3) * offset
        sy = math.cos(math.pi/3) * offset
        orn = p.getQuaternionFromEuler([d, 0, 0])
        self.addPixel(p,self.dirpath + "pixel_core.urdf", [self.x, offset + self.y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core.urdf", [sx + self.x, sy + self.y, h], orn)
        rot = math.pi /3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core.urdf", [-sx + self.x, sy + self.y, h], orn)
        rot = math.pi
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core.urdf", [self.x, -offset + self.y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core.urdf", [-sx + self.x, -sy + self.y, h], orn)
        rot = math.pi / 3 
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core.urdf", [sx + self.x, -sy + self.y, h], orn)

        self.fillHit(p)
        

class HlStarLight (Hyperlights):
    # hit = []
    # pixel = []

    def create (self, p):
        self.make_header()
        self.pixelOffset = 7

        self.pxiel = []
        h = self.z
        _x = self.x
        _y = self.y

        offset = 1.25
        d = math.pi / 2
        rot = math.pi 
        sx = math.sin(math.pi/3) * offset
        sy = math.cos(math.pi/3) * offset

        # White Core 0 - center
        self.addPixel(p,self.dirpath + "pixel.urdf", [_x, _y, h], [0,0,0,1] )
        # RGB
        orn = p.getQuaternionFromEuler([d, 0, 0])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, offset + _y, h], orn )
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, sy + _y, h], orn)        
        rot = math.pi /3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, sy + _y, h], orn)
        rot = math.pi
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, -offset + _y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, -sy + _y, h], orn)
        rot = math.pi / 3 
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, -sy + _y, h], orn) 


        _x = self.x + 0
        _y = self.y + 4

        # White Core 1
        self.addPixel(p,self.dirpath + "pixel.urdf", [_x, _y, h], [0,0,0,1] )
        # RGB
        orn = p.getQuaternionFromEuler([d, 0, 0])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, offset + _y, h], orn )
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, sy + _y, h], orn)        
        rot = math.pi /3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, sy + _y, h], orn)
        rot = math.pi
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, -offset + _y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, -sy + _y, h], orn)
        rot = math.pi / 3 
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, -sy + _y, h], orn) 



        _x = self.x + math.sin(math.pi/3) * 4
        _y = self.y + 2

        # White Core center
        self.addPixel(p,self.dirpath + "pixel.urdf", [_x, _y, h], [0,0,0,1] )
        # RGB
        orn = p.getQuaternionFromEuler([d, 0, 0])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, offset + _y, h], orn )
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, sy + _y, h], orn)        
        rot = math.pi /3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, sy + _y, h], orn)
        rot = math.pi
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, -offset + _y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, -sy + _y, h], orn)
        rot = math.pi / 3 
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, -sy + _y, h], orn) 


        _x = self.x + math.sin(math.pi/3) * 4
        _y = self.y - 2

        # White Core 1
        self.addPixel(p,self.dirpath + "pixel.urdf", [_x, _y, h], [0,0,0,1] )
        # RGB
        orn = p.getQuaternionFromEuler([d, 0, 0])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, offset + _y, h], orn )
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, sy + _y, h], orn)        
        rot = math.pi /3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, sy + _y, h], orn)
        rot = math.pi
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, -offset + _y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, -sy + _y, h], orn)
        rot = math.pi / 3 
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, -sy + _y, h], orn) 

        _x = self.x + math.sin(math.pi/3) * 4
        _y = self.y - 2

        # White Core 2
        self.addPixel(p,self.dirpath + "pixel.urdf", [_x, _y, h], [0,0,0,1] )
        # RGB
        orn = p.getQuaternionFromEuler([d, 0, 0])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, offset + _y, h], orn )
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, sy + _y, h], orn)        
        rot = math.pi /3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, sy + _y, h], orn)
        rot = math.pi
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, -offset + _y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, -sy + _y, h], orn)
        rot = math.pi / 3 
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, -sy + _y, h], orn) 


        _x = self.x + math.sin(math.pi/3) * 4
        _y = self.y - 2

        # White Core 3
        self.addPixel(p,self.dirpath + "pixel.urdf", [_x, _y, h], [0,0,0,1] )
        # RGB
        orn = p.getQuaternionFromEuler([d, 0, 0])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, offset + _y, h], orn )
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, sy + _y, h], orn)        
        rot = math.pi /3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, sy + _y, h], orn)
        rot = math.pi
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, -offset + _y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, -sy + _y, h], orn)
        rot = math.pi / 3 
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, -sy + _y, h], orn) 


        _x = self.x
        _y = self.y - 4

        # White Core 4
        self.addPixel(p,self.dirpath + "pixel.urdf", [_x, _y, h], [0,0,0,1] )
        # RGB
        orn = p.getQuaternionFromEuler([d, 0, 0])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, offset + _y, h], orn )
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, sy + _y, h], orn)        
        rot = math.pi /3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, sy + _y, h], orn)
        rot = math.pi
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, -offset + _y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, -sy + _y, h], orn)
        rot = math.pi / 3 
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, -sy + _y, h], orn) 


        _x = self.x - math.sin(math.pi/3) * 4
        _y = self.y + 2

        # White Core 6
        self.addPixel(p,self.dirpath + "pixel.urdf", [_x, _y, h], [0,0,0,1] )
        # RGB
        orn = p.getQuaternionFromEuler([d, 0, 0])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, offset + _y, h], orn )
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, sy + _y, h], orn)        
        rot = math.pi /3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, sy + _y, h], orn)
        rot = math.pi
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, -offset + _y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, -sy + _y, h], orn)
        rot = math.pi / 3 
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, -sy + _y, h], orn) 


        _x = self.x - math.sin(math.pi/3) * 4
        _y = self.y - 2

        # White Core 7
        self.addPixel(p,self.dirpath + "pixel.urdf", [_x, _y, h], [0,0,0,1] )
        # RGB
        orn = p.getQuaternionFromEuler([d, 0, 0])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, offset + _y, h], orn )
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, sy + _y, h], orn)        
        rot = math.pi /3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, sy + _y, h], orn)
        rot = math.pi
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [_x, -offset + _y, h], orn)
        rot = math.pi / -3
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [-sx + _x, -sy + _y, h], orn)
        rot = math.pi / 3 
        orn = p.getQuaternionFromEuler([d, 0, rot])
        self.addPixel(p,self.dirpath + "pixel_core_s.urdf", [sx + _x, -sy + _y, h], orn) 



        self.fillHit(p)
