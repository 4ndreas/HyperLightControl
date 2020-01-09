import math

from odio_urdf import *

# https://github.com/hauptmech/odio_urdf

# this script generates the urdf file for the rotating wheel 

filepath = ""
robot_name = "LED"

def LED(link_name,origin,material,geom_origin, color="1 1 1 1"):
    """
        Most of the links are the same except for the passed in info.
        This function just allows grouping the important numbers better. 
    """
    mass = 1.0
    # <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"/>
    # I = [1.0,0,0,1.0,0,1.0]
    I = [0,0,0,0,0,0]
    ret = Link(
        Inertial(
            Origin(origin),
            Mass(value=mass),
            Inertia(I)),
        Visual(
            Origin(geom_origin),
            Geometry(Box(size = "0.5 0.5 0.16")),
            Material(material, Color(rgba=color))),
        Collision(
            Origin(geom_origin),
            Geometry(Box(size = "0.5 0.5 0.16")),
            Material(material, Color(rgba=color))),
        name = link_name)
    return ret



#math.pi/2
mass = 1.0
origin = [0,0,0  ,0,0,0,]
geom_origin = [0,0,0  ,math.pi/2,0,0,]
I = [1.0,0,0,1.0,0,1.0]

material = "white"
link_name= "baseLink"

baseLink = Link(
        Inertial(
            Origin(origin),
            Mass(value=mass),
            Inertia(I)),
        Visual(
            Origin(geom_origin),
            # Geometry(Cylinder(size = "20.0 2.0 20.0")),
            Geometry(Cylinder(radius = "10.0",  length="2.0")),
            Material(material, Color(rgba="1 1 1 0.5"))),
        Collision(
            Origin(geom_origin),
            Geometry(Cylinder(radius = "10.0",  length="2.0")),
            Material(material, Color(rgba="1 1 1 0.5"))),
        name = link_name)

my_robot = Robot("hlWheel")
my_robot(baseLink)
base = Parent("baseLink")

N = 49
i = 0
for j in range(0,6):
    for k in range(1,N):
        d = 0.17
        d2 = 1.0
        link_name = robot_name+"_link_"+ str(i)
        joint_name = robot_name+"_joint_"+ str(i)

        rot = j * ((math.pi * 2 )/ 6)

        # origin = [0 , 0 , 0 ,rot,0,0]
        # geom_origin = [0,0,k*d + 9  ,0,0,0]   
             
        origin = [0,0,0  ,0,0,0]
        geom_origin = [0 , 0 , 0 ,0,0,0]
        f =  48 / (k) - 1 
        color = "%f %f %f 1" % (f,1,1)
        # led_link = LED(link_name,origin,"White",geom_origin, color)
        led_link = LED(link_name,origin,"White",geom_origin)

        dy = math.sin(rot) * (k*d + d2 )
        dz = math.cos(rot) * (k*d + d2 )
        jorigin = [dy,0,dz  ,0,rot,0]
        joint1 = Joint(joint_name, base, Child(link_name), Origin(jorigin), type="fixed")
        # joint1 = Joint(joint_name, base, Child(link_name), type="fixed") 

        my_robot(led_link)
        my_robot(joint1)
        i+= 1


f= open("data\\hlWheel.urdf","w+")
print(my_robot, file=f)
f.close() 
print(my_robot)
