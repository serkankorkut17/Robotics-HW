from vpython import *
import numpy as np

scene = canvas(title='ROBOTICS HW2: Articulate', width=1200, height=800)

x = vector(1, 0, 0)
y = vector(0, 1, 0)
z = vector(0, 0, 1)

angle1 = 0
angle2 = 0
angle3 = 0

def matrix_vp():
    # Orientation and location of the base frame with respect to the world frame
    return np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])
    
def rotation_matrix_z(angle):
    # Rotation matrix around z-axis
    return np.array([[np.cos(np.radians(angle)), -np.sin(np.radians(angle)), 0],
                     [np.sin(np.radians(angle)), np.cos(np.radians(angle)), 0],
                     [0, 0, 1]])

def matrix_R12(angle):
    # Orientation and location of joint 2 frame with respect to joint 1 frame
    R1_2 = np.dot(rotation_matrix_z(angle), np.array([[1, 0, 0],
                                               [0, 0, -1],
                                               [0, 1, 0]]))
    matrix = np.zeros((4, 4))
    matrix[:3, :3] = R1_2
    matrix[3, 3] = 1
    matrix[2, 3] = 2
    return matrix

def matrix_R23(angle):
    # Orientation and location of joint 3 frame with respect to joint 2 frame
    R2_3 = np.dot(rotation_matrix_z(angle), np.array([[1, 0, 0],
                                               [0, 1, 0],
                                               [0, 0, 1]]))
    matrix = np.zeros((4, 4))
    matrix[:3, :3] = R2_3
    matrix[3, 3] = 1
    matrix[0, 3] = 2 * np.cos(np.radians(angle))
    matrix[1, 3] = 2 * np.sin(np.radians(angle))
    return matrix

def matrix_R34(angle):
    # Orientation and location of end effector frame with respect to joint 3 frame
    R3_end = np.dot(rotation_matrix_z(angle), np.array([[1, 0, 0],
                                               [0, 1, 0],
                                               [0, 0, 1]]))
    matrix = np.zeros((4, 4))
    matrix[:3, :3] = R3_end
    matrix[3, 3] = 1
    matrix[0, 3] = 2 * np.cos(np.radians(angle))
    matrix[1, 3] = 2 * np.sin(np.radians(angle))
    return matrix

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

def update_articulate(angle1, angle2, angle3):
    # Position and orientation of joint 1 frame with respect to the world frame
    matrix_joint1 = np.dot(matrix_vp(), matrix_R12(angle1))
    # Position and orientation of joint 2 frame with respect to the world frame
    matrix_joint2 = np.dot(matrix_joint1, matrix_R23(angle2))
    # Position and orientation of joint 3 frame with respect to the world frame
    matrix_joint3 = np.dot(matrix_joint2, matrix_R34(angle3))
        
    # Joint 2 position and orientation update according to joint 1
    joint2.axis = vector(matrix_joint1[0, 2], matrix_joint1[1, 2], matrix_joint1[2, 2])
    joint2.pos = vector(matrix_joint1[0, 3], matrix_joint1[1, 3], matrix_joint1[2, 3]) - joint2.axis/2
    
    # Other joint 2 elements update according to joint 2
    line2_1.axis = vector(matrix_joint2[0, 0], matrix_joint2[1, 0], matrix_joint2[2, 0])
    line2_1.pos = joint2.pos
    line2_2.axis = vector(matrix_joint2[0, 0], matrix_joint2[1, 0], matrix_joint2[2, 0])
    line2_2.pos = joint2.pos + joint2.axis
    line2_3.axis = vector(matrix_joint2[0, 2], matrix_joint2[1, 2], matrix_joint2[2, 2])
    line2_3.pos = joint2.pos + line2_1.axis
    line2_4.axis = vector(matrix_joint2[0, 0], matrix_joint2[1, 0], matrix_joint2[2, 0])
    line2_4.pos = vector(matrix_joint1[0, 3], matrix_joint1[1, 3], matrix_joint1[2, 3]) + line2_1.axis
    
    # Joint 3 position and orientation update according to joint 2
    joint3.axis = vector(matrix_joint2[0, 2], matrix_joint2[1, 2], matrix_joint2[2, 2])
    joint3.pos = vector(matrix_joint2[0, 3], matrix_joint2[1, 3], matrix_joint2[2, 3]) - joint3.axis/2
        
    # Other joint 3 elements update according to joint 3
    line3_1.axis = vector(matrix_joint3[0, 0], matrix_joint3[1, 0], matrix_joint3[2, 0])
    line3_1.pos = joint3.pos
    line3_2.axis = vector(matrix_joint3[0, 0], matrix_joint3[1, 0], matrix_joint3[2, 0])
    line3_2.pos = joint3.pos + joint3.axis
    line3_3.axis = vector(matrix_joint3[0, 2], matrix_joint3[1, 2], matrix_joint3[2, 2])
    line3_3.pos = joint3.pos + line3_1.axis
    line3_4.axis = vector(matrix_joint3[0, 0], matrix_joint3[1, 0], matrix_joint3[2, 0])
    line3_4.pos = vector(matrix_joint2[0, 3], matrix_joint2[1, 3], matrix_joint2[2, 3])+ line3_1.axis


scene.autoscale = True
scene.camera.pos = vector(4, 2, 4)
scene.camera.axis = vector(-1, -1, -4)

def key_input(event):
    global angle1, angle2, angle3
    if event.key == 'q' or event.key == 'Q':
        # Rotate around the joint1 by -5 degrees
        angle1 -= 5

    if event.key == 'w' or event.key == 'W':
        # Rotate around the joint1 by 5 degrees
        angle1 += 5
        
    if event.key == 'a' or event.key == 'A':
        # Rotate around the joint2 by -5 degrees
        angle2 -= 5
        
    if event.key == 's' or event.key == 'S':
        # Rotate around the joint2 by 5 degrees
        angle2 += 5
        
    if event.key == 'z' or event.key == 'Z':
        # Rotate around the joint3 by -5 degrees
        angle3 -= 5

    if event.key == 'x' or event.key == 'X':
        # Rotate around the joint3 by 5 degrees
        angle3 += 5
    # Update the position of the robot
    update_articulate(angle1, angle2, angle3)

scene.bind('keydown', key_input)

while True:
    location = vector(line3_4.pos + line3_4.axis)
    #how to round location vector to 2 decimal places
    location.x = round(location.x, 2)
    location.y = round(location.y, 2)
    location.z = round(location.z, 2)

    scene.title = 'ROBOTICS HW2: Articulate - Location: ' + str(location)
    rate(30)