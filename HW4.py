from vpython import *
import numpy as np
from scipy.spatial.transform import Rotation as R

scene = canvas(title='ROBOTICS HW4: Jacobian', width=1200, height=800)

x = vector(1, 0, 0)
y = vector(0, 1, 0)
z = vector(0, 0, 1)

angle1 = 0
angle2 = 0
angle3 = 0

point = vector(0, 0, 0)
mycurve = curve()

def rotation_x(angle):
    return np.array([[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle), np.cos(angle)]])

def rotation_y(angle):
    return np.array([[np.cos(angle), 0, np.sin(angle)],
                     [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]])

def rotation_z(angle):
    return np.array([[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0],
                     [0, 0, 1]])

def rotation_matrix_z(angle):
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
    R3_4 = np.dot(rotation_matrix_z(angle), np.array([[1, 0, 0],
                                               [0, 1, 0],
                                               [0, 0, 1]]))
    matrix = np.zeros((4, 4))
    matrix[:3, :3] = R3_4
    matrix[3, 3] = 1
    matrix[0, 3] = 2.25 * np.cos(np.radians(angle))
    matrix[1, 3] = 2.25 * np.sin(np.radians(angle))
    return matrix

def matrix_vp():
    # Orientation and location of the base frame with respect to the world frame
    return np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])
    
def inverse_kinematics(x, y, z):
    a1 = 2
    a2 = 2
    a3 = 2.25
    
    #** Calculate the first angle
    angle1find = np.arctan2(z, x)
    # Calculate the distance to the point in the yz-plane
    r = np.sqrt(x**2 + z**2)
    # Calculate the distance from the base to the point (x, y, z)
    D = np.sqrt(r**2 + (y - a1)**2)
    
    # Check if the position is reachable
    if D > a2 + a3 or D < abs(a2 - a3):
        raise ValueError("The point is out of the reach of the robotic arm")
    
    #** Calculate the second angle
    beta1 = np.arctan2(y - a1, r)
    cos_beta2 = (a2**2 + D**2 - a3**2) / (2 * a2 * D)
    # Clamp the value to the valid range of arccos to avoid NaN
    cos_beta2 = np.clip(cos_beta2, -1.0, 1.0)
    beta2 = np.arccos(cos_beta2)
    
    angle2find = beta1 - beta2
    
    #** Calculate the third angle
    cos_beta3 = (a2**2 + a3**2 - D**2) / (2 * a2 * a3)
    # Clamp the value to the valid range of arccos to avoid NaN
    cos_beta3 = np.clip(cos_beta3, -1.0, 1.0)
    beta3 = np.arccos(cos_beta3)
    
    angle3find = np.pi - beta3
    
    # clip angles to the valid range
    angle1find = np.clip(angle1find, -np.pi, np.pi)
    angle2find = np.clip(angle2find, -np.pi, np.pi)
    angle3find = np.clip(angle3find, -np.pi, np.pi)
    
    return angle1find, angle2find, angle3find

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

# hand
handjoint1 = cylinder(pos=vector(4, 2, 0), axis=x/4, radius=0.1, color=color.orange)
handjoint2 = cylinder(pos=vector(4.25,2,-0.05), axis=z/10, radius=0.05, color=color.cyan)
handjoint3 = cylinder(pos=vector(4.25,2,0), axis=x/10, radius=0.03, color=color.purple)

axis_location = handjoint3.pos + handjoint3.axis
x_axis = arrow(pos=axis_location, axis=x, color=color.red)
y_axis = arrow(pos=axis_location, axis=y, color=color.green)
z_axis = arrow(pos=axis_location, axis=z, color=color.blue)
x_axis.shaftwidth = 0.01
y_axis.shaftwidth = 0.01
z_axis.shaftwidth = 0.01

def update_articulate(angle1, angle2, angle3, location, hand_matrix):
    matrix_joint1 = np.dot(matrix_vp(), matrix_R12(angle1))
    # round the matrix_joint1 to 2 decimal places
    matrix_joint1 = np.round(matrix_joint1, 2)
    matrix_joint2 = np.dot(matrix_joint1, matrix_R23(angle2))
    matrix_joint2 = np.round(matrix_joint2, 2)
    matrix_joint3 = np.dot(matrix_joint2, matrix_R34(angle3))
    matrix_joint3 = np.round(matrix_joint3, 2)
        
    #  Joint 2 position and orientation update according to joint 1
    joint2.axis = vector(matrix_joint1[0, 2], matrix_joint1[1, 2], matrix_joint1[2, 2])
    joint2.pos = vector(matrix_joint1[0, 3], matrix_joint1[1, 3], matrix_joint1[2, 3]) - joint2.axis/2
        
    # Joint 2 elements update according to joint 2
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
        
    # Joint 3 elements update according to joint 3
    line3_1.axis = vector(matrix_joint3[0, 0], matrix_joint3[1, 0], matrix_joint3[2, 0])
    line3_1.pos = joint3.pos
    line3_2.axis = vector(matrix_joint3[0, 0], matrix_joint3[1, 0], matrix_joint3[2, 0])
    line3_2.pos = joint3.pos + joint3.axis
    line3_3.axis = vector(matrix_joint3[0, 2], matrix_joint3[1, 2], matrix_joint3[2, 2])
    line3_3.pos = joint3.pos + line3_1.axis
    line3_4.axis = vector(matrix_joint3[0, 0], matrix_joint3[1, 0], matrix_joint3[2, 0])
    line3_4.pos = vector(matrix_joint2[0, 3], matrix_joint2[1, 3], matrix_joint2[2, 3])+ line3_1.axis
    
    # inverse of matrix_R1_4
    matrix_joint3_inv = np.linalg.inv(matrix_joint3)

    # matrix_R1_7 = np.array([[1, 0, 0, location.x],
    #                  [0, 1, 0, location.y],
    #                  [0, 0, 1, location.z],
    #                  [0, 0, 0, 1]])
    
    matrix_R1_7 = np.zeros((4, 4))
    matrix_R1_7[:3, :3] = hand_matrix
    matrix_R1_7[:3, 3] = [location.x, location.y, location.z]
    matrix_R1_7[3, 3] = 1
    
    matrix_R4_7 = np.dot(matrix_joint3_inv, matrix_R1_7)

    # Extract the rotation (3x3) and translation (3x1) components
    R4_7 = matrix_R4_7[:3, :3]
    t4_7 = matrix_R4_7[:3, 3]

    # Use scipy Rotation to decompose the rotation matrix
    rotation = R.from_matrix(R4_7)
    angles = rotation.as_euler('xyz', degrees=True)  # Extract XYZ Euler angles in degrees

    # Construct individual rotation matrices
    Rz = R.from_euler('z', angles[2], degrees=True).as_matrix()
    Ry = R.from_euler('y', angles[1], degrees=True).as_matrix()
    Rx = R.from_euler('x', angles[0], degrees=True).as_matrix()

    # Construct the individual homogeneous transformation matrices
    def construct_homogeneous_matrix(R, t):
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    matrix_R4_5 = construct_homogeneous_matrix(Rz, [0, 0, 0])
    matrix_R5_6 = construct_homogeneous_matrix(Ry, [0, 0, 0])
    matrix_R6_7 = construct_homogeneous_matrix(Rx, t4_7)
    
    handjoint1.axis = vector(matrix_joint3[0, 0], matrix_joint3[1, 0], matrix_joint3[2, 0]) / 4
    handjoint1.pos = line3_4.pos + line3_4.axis
    
    handjoint2.axis = vector(matrix_R4_5[0, 2], matrix_R4_5[1, 2], matrix_R4_5[2, 2]) / 10
    handjoint2.pos = handjoint1.pos + handjoint1.axis - handjoint2.axis/2
    
    handjoint3.axis = vector(matrix_R5_6[0, 0], matrix_R5_6[1, 0], matrix_R5_6[2, 0]) / 10
    handjoint3.pos = handjoint1.pos + handjoint1.axis
    
    mycurve.append(pos=handjoint3.pos)
    
    axis_location = handjoint3.pos + handjoint3.axis
    x_axis.pos = axis_location
    y_axis.pos = axis_location
    z_axis.pos = axis_location
    
    x_axis.axis = vector(matrix_R6_7[0, 0], matrix_R6_7[1, 0], matrix_R6_7[2, 0])
    y_axis.axis = vector(matrix_R6_7[0, 1], matrix_R6_7[1, 1], matrix_R6_7[2, 1])
    z_axis.axis = vector(matrix_R6_7[0, 2], matrix_R6_7[1, 2], matrix_R6_7[2, 2])
    
    
def jacobian(angle1, angle2, angle3):
    matrix_H0_1 = np.dot(matrix_vp(), matrix_R12(angle1))
    matrix_H1_2 = matrix_R23(angle2)
    matrix_H2_3 = matrix_R34(angle3)

    matrix_H0_2 = np.dot(matrix_H0_1, matrix_H1_2)
    matrix_H0_3 = np.dot(matrix_H0_2, matrix_H2_3)

    jac = np.array([0, 0, 1])

    # Calculate the Jacobian
    J = np.zeros((3, 3))
    # Revolute R0_0 . [0 0 1] * ( d0_3 - d0_0) 
    J[:, 0] = np.cross(np.dot(matrix_vp()[:3, :3], jac), matrix_H0_3[:3, 3])
    # Revolute R0_1 . [0 0 1] * ( d0_3 - d0_1)
    J[:, 1] = np.cross(np.dot(matrix_H0_1[:3, :3], jac), matrix_H0_3[:3, 3] - matrix_H0_1[:3, 3])
    # Revolute R0_2 . [0 0 1] * ( d0_3 - d0_2)
    J[:, 2] = np.cross(np.dot(matrix_H0_2[:3, :3], jac), matrix_H0_3[:3, 3] - matrix_H0_2[:3, 3])

    return J


def forward_kinematics(theta1, theta2, theta3):
    matrix_H0_1 = np.dot(matrix_vp(), matrix_R12(theta1))
    matrix_H0_2 = np.dot(matrix_H0_1, matrix_R23(theta2))
    matrix_H0_3 = np.dot(matrix_H0_2, matrix_R34(theta3))
    
    x, y, z = matrix_H0_3[:3, 3]

    return x, y, z
    

def key_input(event):
    # move the camera
    global speed
    if event.key == 'up':
        scene.camera.pos += vector(0, 0, 1)
    elif event.key == 'down':
        scene.camera.pos += vector(0, 0, -1)
    elif event.key == 'left':
        scene.camera.pos += vector(-1, 0, 0)
    elif event.key == 'right':
        scene.camera.pos += vector(1, 0, 0)
    elif event.key == 'w':
        # move the camera forward
        scene.camera.pos += scene.camera.axis * speed
    elif event.key == 's':
        # move the camera backward
        scene.camera.pos -= scene.camera.axis * speed
    elif event.key == 'a':
        # move the camera left
        scene.camera.pos -= scene.camera.right * speed
    elif event.key == 'd':
        # move the camera right
        scene.camera.pos += scene.camera.right * speed
    elif event.key == 'r':
        scene.camera.pos = vector(4, 2, 4)
        scene.camera.axis = vector(-1, -1, -4)
    elif event.key == 'z':
        speed += 0.1
    elif event.key == 'x':
        speed -= 0.1
    elif event.key == 'p':
        print(scene.camera.pos)
        print(scene.camera.axis)
        print(scene.camera.up)
        print(scene.camera.right)
    elif event.key == 'q':
        scene.camera.rotate(angle=radians(1), axis=scene.camera.axis)
    elif event.key == 'e':
        scene.camera.rotate(angle=radians(1), axis=-scene.camera.axis)  
        
scene.autoscale = True
scene.camera.pos = vector(0, 0, 8)
scene.camera.axis = vector(0, 0, -8)
speed = 0.1

scene.bind('keydown', key_input)


count = 0
hand = np.eye(3)
angle1find, angle2find, angle3find = 0, 0, 0

def on_button_click(s):
    global hand
    x2 = np.array([1, 0, 0])
    y2 = np.array([0, 1, 0])
    z2 = np.array([0, 0, 1])
    matrix = np.column_stack((x2, y2, z2))
    
    angle_end_x = endx_slider.value
    angle_end_y = endy_slider.value
    angle_end_z = endz_slider.value
    Rx = rotation_x(np.radians(angle_end_x))
    Ry = rotation_y(np.radians(angle_end_y))
    Rz = rotation_z(np.radians(angle_end_z))
    hand = np.dot(Rz, np.dot(Ry, np.dot(Rx, matrix)))
    
endx_slider = slider(min=0, max=360, bind=on_button_click, length=300, right=5)
endy_slider = slider(min=0, max=360, bind=on_button_click, length=300, right=5)
endz_slider = slider(min=0, max=360, bind=on_button_click, length=300, sright=5)

def draw(t):
    radius = 4.0  # Radius of the circle
    radius2 = 2.0
    omega = 0.01   # Angular speed

    x = radius * np.cos(omega * t)
    y = 2.0
    z = radius * np.sin(omega * t)
    
    x_speed = -1 * radius * np.sin(omega * t) * omega
    y_speed = 0
    z_speed = radius * np.cos(omega * t) * omega
    #** draw ellipse **#
    # x = radius * np.cos(omega * t)
    # y = radius2 * np.sin(omega * t)
    # z = -2.0
    # x_speed = -1 * radius * np.sin(omega * t) * omega
    # y_speed = radius2 * np.cos(omega * t) * omega
    # z_speed = 0
    return x, y, z, x_speed, y_speed, z_speed

#** 1 - PATH OF CIRCLE **#
x, y, z, _, _, _ = draw(count)

#** 2 - STARTING POINT **#
loc = vector(x, y, z)
print("Starting Point: ",loc)
    
#** 3 - INVERSE KINEMATICS **#
angle1find, angle2find, angle3find = inverse_kinematics(x, y, z)
update_articulate(np.degrees(angle1find), np.degrees(angle2find), np.degrees(angle3find), loc, hand)

location = vector(handjoint3.pos)
location.x = round(location.x, 2)
location.y = round(location.y, 2)
location.z = round(location.z, 2)
scene.title = 'ROBOTICS HW4 - Location: ' + str(location) + 'Angles: ' + str(np.degrees(angle1find)) + ' ' + str(np.degrees(angle2find)) + ' ' + str(np.degrees(angle3find))

while True:
    count += 0.01
    
    #** 4 - JACOBIAN MATRIX **#
    J = jacobian(np.degrees(angle1find), np.degrees(angle2find), np.degrees(angle3find))
    
    #** 5 - ANGULAR SPEED **#
    x, y, z, x_speed, y_speed, z_speed = draw(count)
    loc = vector(x, y, z)
    
    v = np.array([x_speed, y_speed, z_speed])
    invJ = np.linalg.pinv(J)
    angle_speeds = np.dot(invJ, v)
    
    #** 6 - UPDATE ANGLES **#
    delta_time = 0.01  # Time step
    angle1find += angle_speeds[0] * delta_time
    angle2find += angle_speeds[1] * delta_time
    angle3find += angle_speeds[2] * delta_time
    
    #** 7 - UPDATE ARTICULATE **#
    update_articulate(np.degrees(angle1find), np.degrees(angle2find), np.degrees(angle3find), loc, hand)
    
    location = vector(handjoint3.pos)
    # how to round location vector to 2 decimal places
    location.x = round(location.x, 2)
    location.y = round(location.y, 2)
    location.z = round(location.z, 2)
    # how toround angles
    a1 = round(np.degrees(angle1find), 2)
    a2 = round(np.degrees(angle2find), 2)
    a3 = round(np.degrees(angle3find), 2)
    scene.title = 'ROBOTICS HW4: Jacobian - Location: ' + str(location) + 'Angles: ' + str(a1) + ' ' + str(a2) + ' ' + str(a3)
    rate(8000)