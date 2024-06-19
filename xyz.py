from vpython import *
import serial

scene = canvas(title='ROBOTICS HW1', width=800, height=800)

x = vector(1, 0, 0)
y = vector(0, 1, 0)
z = vector(0, 0, 1)

cylinder_x = cylinder(pos=vector(0, 0, 0), axis=x, radius=0.05, color=color.red)
cone_x = cone(pos=x, axis=x/5, radius=0.1, color=color.red)

cylinder_y = cylinder(pos=vector(0, 0, 0), axis=y, radius=0.05, color=color.green)
cone_y = cone(pos=y, axis=y/5, radius=0.1, color=color.green)

cylinder_z = cylinder(pos=vector(0, 0, 0), axis=z, radius=0.05, color=color.blue)
cone_z = cone(pos=z, axis=z/5, radius=0.1, color=color.blue)

scene.autoscale = True
scene.camera.pos = vector(0, 0, 2.2)
scene.camera.axis = vector(0, 0, -2.2)

def key_input(event):
    if event.key == 'q' or event.key == 'Q':
        # Rotate around the X-axis by -5 degrees
        cylinder_y.rotate(angle=radians(-5), axis=cylinder_x.axis, origin=vector(0, 0, 0))
        cone_y.rotate(angle=radians(-5), axis=cylinder_x.axis, origin=vector(0, 0, 0))

        cylinder_z.rotate(angle=radians(-5), axis=cylinder_x.axis, origin=vector(0, 0, 0))
        cone_z.rotate(angle=radians(-5), axis=cylinder_x.axis, origin=vector(0, 0, 0))

    if event.key == 'w' or event.key == 'W':
        # Rotate around the X-axis by 5 degrees
        cylinder_y.rotate(angle=radians(5), axis=cylinder_x.axis, origin=vector(0, 0, 0))
        cone_y.rotate(angle=radians(5), axis=cylinder_x.axis, origin=vector(0, 0, 0))

        cylinder_z.rotate(angle=radians(5), axis=cylinder_x.axis, origin=vector(0, 0, 0))
        cone_z.rotate(angle=radians(5), axis=cylinder_x.axis, origin=vector(0, 0, 0))

    if event.key == 'a' or event.key == 'A':
        # Rotate around the Y-axis by -5 degrees
        cylinder_x.rotate(angle=radians(-5), axis=cylinder_y.axis, origin=vector(0, 0, 0))
        cone_x.rotate(angle=radians(-5), axis=cylinder_y.axis, origin=vector(0, 0, 0))

        cylinder_z.rotate(angle=radians(-5), axis=cylinder_y.axis, origin=vector(0, 0, 0))
        cone_z.rotate(angle=radians(-5), axis=cylinder_y.axis, origin=vector(0, 0, 0))

    if event.key == 's' or event.key == 'S':
        # Rotate around the Y-axis by 5 degrees
        cylinder_x.rotate(angle=radians(5), axis=cylinder_y.axis, origin=vector(0, 0, 0))
        cone_x.rotate(angle=radians(5), axis=cylinder_y.axis, origin=vector(0, 0, 0))

        cylinder_z.rotate(angle=radians(5), axis=cylinder_y.axis, origin=vector(0, 0, 0))
        cone_z.rotate(angle=radians(5), axis=cylinder_y.axis, origin=vector(0, 0, 0))

    if event.key == 'z' or event.key == 'Z':
        # Rotate around the Z-axis by -5 degrees
        cylinder_x.rotate(angle=radians(-5), axis=cylinder_z.axis, origin=vector(0, 0, 0))
        cone_x.rotate(angle=radians(-5), axis=cylinder_z.axis, origin=vector(0, 0, 0))

        cylinder_y.rotate(angle=radians(-5), axis=cylinder_z.axis, origin=vector(0, 0, 0))
        cone_y.rotate(angle=radians(-5), axis=cylinder_z.axis, origin=vector(0, 0, 0))
    
    if event.key == 'x' or event.key == 'X':
        # Rotate around the Z-axis by 5 degrees
        cylinder_x.rotate(angle=radians(5), axis=cylinder_z.axis, origin=vector(0, 0, 0))
        cone_x.rotate(angle=radians(5), axis=cylinder_z.axis, origin=vector(0, 0, 0))

        cylinder_y.rotate(angle=radians(5), axis=cylinder_z.axis, origin=vector(0, 0, 0))
        cone_y.rotate(angle=radians(5), axis=cylinder_z.axis, origin=vector(0, 0, 0))

scene.bind('keydown', key_input)

while True:
    rate(30)