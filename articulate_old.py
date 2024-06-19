from vpython import *
import numpy as np

scene = canvas(title='ROBOTICS HW2', width=1200, height=800)

x = vector(1, 0, 0)
y = vector(0, 1, 0)
z = vector(0, 0, 1)

angle1 = 0
angle2 = 0
angle3 = 0

# joint 1
joint1 = cylinder(pos=vector(0, 0, 0), axis=y, radius=0.1, color=color.red)
line1 = cylinder(pos=vector(0, 1, 0), axis=y, radius=0.01, color=color.white)

# joint 2
joint2 = cylinder(pos=vector(0, 2, -0.5), axis=z, radius=0.1, color=color.green)
line2_1 = cylinder(pos=vector(0, 2, -0.5), axis=x, radius=0.01, color=color.white)
line2_2 = cylinder(pos=vector(0, 2, 0.5), axis=x, radius=0.01, color=color.white)
line2_3 = cylinder(pos=vector(1, 2, -0.5), axis=z, radius=0.01, color=color.white)
line2_4 = cylinder(pos=vector(1, 2, 0), axis=x, radius=0.01, color=color.white)

# joint 3
joint3 = cylinder(pos=vector(2, 2, -0.5), axis=z, radius=0.1, color=color.blue)
line3_1 = cylinder(pos=vector(2, 2, -0.5), axis=x, radius=0.01, color=color.white)
line3_2 = cylinder(pos=vector(2, 2, 0.5), axis=x, radius=0.01, color=color.white)
line3_3 = cylinder(pos=vector(3, 2, -0.5), axis=z, radius=0.01, color=color.white)
line3_4 = cylinder(pos=vector(3, 2, 0), axis=x, radius=0.01, color=color.white)


scene.autoscale = True
scene.camera.pos = vector(4, 2, 4)
scene.camera.axis = vector(-1, -1, -4)

def key_input(event):
    if event.key == 'q' or event.key == 'Q':
        # Rotate around the joint1 by -5 degrees
        joint1.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        line1.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        joint2.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        line2_1.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        line2_2.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        line2_3.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        line2_4.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        joint3.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        line3_1.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        line3_2.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        line3_3.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)
        line3_4.rotate(angle=radians(-5), axis=joint1.axis, origin=joint1.pos)

    if event.key == 'w' or event.key == 'W':
        # Rotate around the joint1 by 5 degrees
        joint1.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        line1.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        joint2.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        line2_1.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        line2_2.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        line2_3.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        line2_4.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        joint3.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        line3_1.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        line3_2.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        line3_3.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)
        line3_4.rotate(angle=radians(5), axis=joint1.axis, origin=joint1.pos)

    if event.key == 'a' or event.key == 'A':
        # Rotate around the joint2 by -5 degrees
        joint2.rotate(angle=radians(-5), axis=joint2.axis, origin=joint2.pos)
        line2_1.rotate(angle=radians(-5), axis=joint2.axis, origin=joint2.pos)
        line2_2.rotate(angle=radians(-5), axis=joint2.axis, origin=joint2.pos)
        line2_3.rotate(angle=radians(-5), axis=joint2.axis, origin=joint2.pos)
        line2_4.rotate(angle=radians(-5), axis=joint2.axis, origin=joint2.pos)
        joint3.rotate(angle=radians(-5), axis=joint2.axis, origin=joint2.pos)
        line3_1.rotate(angle=radians(-5), axis=joint2.axis, origin=joint2.pos)
        line3_2.rotate(angle=radians(-5), axis=joint2.axis, origin=joint2.pos)
        line3_3.rotate(angle=radians(-5), axis=joint2.axis, origin=joint2.pos)
        line3_4.rotate(angle=radians(-5), axis=joint2.axis, origin=joint2.pos)
    
    if event.key == 's' or event.key == 'S':
        # Rotate around the joint2 by 5 degrees
        joint2.rotate(angle=radians(5), axis=joint2.axis, origin=joint2.pos)
        line2_1.rotate(angle=radians(5), axis=joint2.axis, origin=joint2.pos)
        line2_2.rotate(angle=radians(5), axis=joint2.axis, origin=joint2.pos)
        line2_3.rotate(angle=radians(5), axis=joint2.axis, origin=joint2.pos)
        line2_4.rotate(angle=radians(5), axis=joint2.axis, origin=joint2.pos)
        joint3.rotate(angle=radians(5), axis=joint2.axis, origin=joint2.pos)
        line3_1.rotate(angle=radians(5), axis=joint2.axis, origin=joint2.pos)
        line3_2.rotate(angle=radians(5), axis=joint2.axis, origin=joint2.pos)
        line3_3.rotate(angle=radians(5), axis=joint2.axis, origin=joint2.pos)
        line3_4.rotate(angle=radians(5), axis=joint2.axis, origin=joint2.pos)

    if event.key == 'z' or event.key == 'Z':
        # Rotate around the joint3 by -5 degrees
        joint3.rotate(angle=radians(-5), axis=joint3.axis, origin=joint3.pos)
        line3_1.rotate(angle=radians(-5), axis=joint3.axis, origin=joint3.pos)
        line3_2.rotate(angle=radians(-5), axis=joint3.axis, origin=joint3.pos)
        line3_3.rotate(angle=radians(-5), axis=joint3.axis, origin=joint3.pos)
        line3_4.rotate(angle=radians(-5), axis=joint3.axis, origin=joint3.pos)

    if event.key == 'x' or event.key == 'X':
        # Rotate around the joint3 by 5 degrees
        joint3.rotate(angle=radians(5), axis=joint3.axis, origin=joint3.pos)
        line3_1.rotate(angle=radians(5), axis=joint3.axis, origin=joint3.pos)
        line3_2.rotate(angle=radians(5), axis=joint3.axis, origin=joint3.pos)
        line3_3.rotate(angle=radians(5), axis=joint3.axis, origin=joint3.pos)
        line3_4.rotate(angle=radians(5), axis=joint3.axis, origin=joint3.pos)


scene.bind('keydown', key_input)

while True:
    location = vector(line3_4.pos + line3_4.axis)
    #how to round location vector to 2 decimal places
    location.x = round(location.x, 2)
    location.y = round(location.y, 2)
    location.z = round(location.z, 2)

    scene.title = 'ROBOTICS HW2 - Location: ' + str(location)
    rate(30)