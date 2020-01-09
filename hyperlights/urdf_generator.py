# import xml.etree.cElementTree as ET

# root = ET.Element("root")
# doc = ET.SubElement(root, "doc")

# ET.SubElement(doc, "field1", name="blah").text = "some value1"
# ET.SubElement(doc, "field2", name="asdfasd").text = "some vlaue2"

# tree = ET.ElementTree(root)
# tree.write("filename.xml")

# If you have OCD and need a clean namespace, change to standard import
from odio_urdf import * 
# https://github.com/hauptmech/odio_urdf


# my_robot = Robot(
#     Link(name="link1"),
#     Link(name="link2"),
#     Link("link3"), #String arguments without keys are assumed to be 'name'
#     Link(name="link4"),

#     Joint("joint1", Parent("link1"), Child("link2"), type="continuous"),
#     Joint("joint2", 
#         Parent("link1"),
#         Child("link2"),
#         type="continuous" #KeyValue arguments go at the end of a call
#     ),
#     Joint("joint3", Parent("link3"), Child("link4"), type="continuous")
# ) 

filepath = ""
robot_name = "LED"


def link(N,robot_name,origin,mass,I,material,geom_origin):
    """
        Most of the links are the same except for the passed in info.
        This function just allows grouping the important numbers better. 
    """
    N = str(N)
    ret = Link(
        Inertial(
            Origin(origin),
            Mass(value=mass),
            Inertia(I)),
        Visual(
            Origin(geom_origin),
            Geometry(Mesh(filename = filepath+"visual/link_"+N+".stl")),
            Material(material)),
        Collision(
            Origin(geom_origin),
            Geometry(Mesh(filename = filepath+"collision/link_"+N+".stl")),
            Material(material, Color(rgba="1,1,1,1"))),
        name = robot_name+"_link_"+N)
    return ret


def LED(link_name,origin,material,geom_origin):
    """
        Most of the links are the same except for the passed in info.
        This function just allows grouping the important numbers better. 
    """
    mass = 1.0
    # <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"/>
    I = [1.0,0,0,1.0,0,1.0]
    ret = Link(
        Inertial(
            Origin(origin),
            Mass(value=mass),
            Inertia(I)),
        Visual(
            Origin(geom_origin),
            Geometry(Box(size = "0.5 0.5 0.16")),
            Material(material, Color(rgba="1 1 1 1"))),
        Collision(
            Origin(geom_origin),
            Geometry(Box(size = "0.5 0.5 0.16")),
            Material(material, Color(rgba="1 1 1 1"))),
        name = link_name)
    return ret


# print(my_robot) #Dump urdf to stdout

my_robot = Robot("Tristar48")
# link1 = Link() #name 'link1' will be used unless link1.name is set 
# link1 = link(1,"hl","0,0,0",1,1,"White","0,0,0")
# link1 = LED("hl","0,0,0","White","0,0,0")
# link2 = Link() 
# link3 = Link("special_name") #This link has the name 'special_name' 

mass = 1.0
origin = [0,0,0  ,0,0,0]
geom_origin = "0,0,0"
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
            Geometry(Box(size = "0.5 0.5 8.6")),
            Material(material, Color(rgba="1 1 1 0.5"))),
        Collision(
            Origin(geom_origin),
            Geometry(Box(size = "0.5 0.5 8.6")),
            Material(material, Color(rgba="1 1 1 0.5"))),
        name = link_name)



my_robot(baseLink)
base = Parent("baseLink")

N = 48
for i in range(1,N):
    d = -0.17
    link_name = robot_name+"_link_"+ str(i)
    joint_name = robot_name+"_joint_"+ str(i)

    geom_origin = [0 , 0 , i*d + 4.3]
    origin = [0,0,0  ,0,0,0]

    led_link = LED(link_name,origin,"White",geom_origin)
    joint1 = Joint(joint_name, base, Child(link_name), type="fixed") 
    my_robot(led_link)
    my_robot(joint1)



#Add first elements to robot

# my_robot(link1,link2,link3)
# my_robot(link1)

# base = Parent("link1")
# joint1 = Joint(base, Child("link2"), type="fixed") 
# joint2 = Joint(base, Child("link3"), type="continuous")
# joint3 = Joint(Parent("link3"), Child("special_name"), type="continuous")

# my_robot(joint1,joint2,joint3)

# my_robot.write("filename.xml")
f= open("tristar_48g.urdf","w+")
print(my_robot, file=f)
# f.write(my_robot)
f.close() 

print(my_robot)
