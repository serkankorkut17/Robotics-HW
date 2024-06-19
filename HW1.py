from vpython import *
import serial
import numpy as np

ser = serial.Serial('/dev/cu.usbserial-110', 9600)
angles = vector(0,0,0)

scene = canvas(title='ROBOTICS HW1: Gyro', width=1200, height=800)

x = vector(1, 0, 0)
y = vector(0, 1, 0)
z = vector(0, 0, 1)

x2 = np.array([1, 0, 0])
y2 = np.array([0, 1, 0])
z2 = np.array([0, 0, 1])
matrix = np.column_stack((x2, y2, z2))

cylinder_x = cylinder(pos=vector(0, 0, 0), axis=x, radius=0.05, color=color.red)
cone_x = cone(pos=x, axis=x/5, radius=0.1, color=color.red)

cylinder_y = cylinder(pos=vector(0, 0, 0), axis=y, radius=0.05, color=color.green)
cone_y = cone(pos=y, axis=y/5, radius=0.1, color=color.green)

cylinder_z = cylinder(pos=vector(0, 0, 0), axis=z, radius=0.05, color=color.blue)
cone_z = cone(pos=z, axis=z/5, radius=0.1, color=color.blue)

scene.autoscale = True
scene.camera.pos = vector(0, 0, 2.2)
scene.camera.axis = vector(0, 0, -2.2)

# Define rotation matrices
def rotation_matrix_x(angle):
    return np.array([[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle), np.cos(angle)]])

def rotation_matrix_y(angle):
    return np.array([[np.cos(angle), 0, np.sin(angle)],
                     [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]])

def rotation_matrix_z(angle):
    return np.array([[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0],
                     [0, 0, 1]])

def key_input(event):
    s = event.key
    if s == 'up':
        scene.camera.pos += vector(0, 0.1, 0)
        scene.camera.axis += vector(0, 0.1, 0)
    elif s == 'down':
        scene.camera.pos -= vector(0, 0.1, 0)
        scene.camera.axis -= vector(0, 0.1, 0)
    elif s == 'left':
        scene.camera.pos -= vector(0.1, 0, 0)
        scene.camera.axis -= vector(0.1, 0, 0)
    elif s == 'right':
        scene.camera.pos += vector(0.1, 0, 0)
        scene.camera.axis += vector(0.1, 0, 0)
    elif s == 'w':
        scene.camera.pos += vector(0, 0, 0.1)
        scene.camera.axis += vector(0, 0, 0.1)
    elif s == 's':
        scene.camera.pos -= vector(0, 0, 0.1)
        scene.camera.axis -= vector(0, 0, 0.1)
    elif s == 'a':
        scene.camera.pos -= vector(0, 0, 0.1)
        scene.camera.axis -= vector(0, 0, 0.1)
    elif s == 'd':
        scene.camera.pos += vector(0, 0, 0.1)
        scene.camera.axis += vector(0, 0, 0.1)
    elif s == 'r':
        scene.camera.pos = vector(0, 0, 2.2)
        scene.camera.axis = vector(0, 0, -2.2)

scene.bind('keydown', key_input)

while True:
    line = ser.readline().decode('utf-8').rstrip()
    # check if line starts with euler
    if line.startswith('euler'):
        parts = line.split('\t')
        angles = vector(float(parts[1]), float(parts[2]), float(parts[3]))
        print(angles)
        
    x_angle = np.radians(angles.x)  # Convert degrees to radians
    y_angle = np.radians(angles.y)
    z_angle = np.radians(angles.z)
    Rx = rotation_matrix_x(x_angle)
    Ry = rotation_matrix_y(y_angle)
    Rz = rotation_matrix_z(z_angle)
    # Apply rotations
    rotated_matrix = np.dot(Rz, np.dot(Ry, np.dot(Rx, matrix)))
    
    # now rotate the cylinders and cones
    cylinder_x.axis = vector(rotated_matrix[0][0], rotated_matrix[1][0], rotated_matrix[2][0])
    cone_x.pos = cylinder_x.pos + cylinder_x.axis
    cone_x.axis = cylinder_x.axis/5
    
    cylinder_y.axis = vector(rotated_matrix[0][1], rotated_matrix[1][1], rotated_matrix[2][1])
    cone_y.pos = cylinder_y.pos + cylinder_y.axis
    cone_y.axis = cylinder_y.axis/5
    
    cylinder_z.axis = vector(rotated_matrix[0][2], rotated_matrix[1][2], rotated_matrix[2][2])
    cone_z.pos = cylinder_z.pos + cylinder_z.axis
    cone_z.axis = cylinder_z.axis/5
    
    rate(30)
