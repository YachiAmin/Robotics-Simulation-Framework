#2D Robot Arm Forward Kinematics 

import numpy as np
import matplotlib.pyplot as plt

# Functions for transformations 
def rotation(theta):
    rad = np.deg2rad(theta)
    return np.array([
        [np.cos(rad), -np.sin(rad), 0],
        [np.sin(rad),  np.cos(rad), 0],
        [0, 0, 1]
    ])

def translation(dx, dy):
    return np.array([
        [1, 0, dx],
        [0, 1, dy],
        [0, 0, 1]
    ])

# Robot parameters
l1 = 5  # length of first link
l2 = 5  # length of second link
theta1 = 45  # angle of first joint in degrees
theta2 = 30  # angle of second joint in degrees

# Base point(fixed)
base = np.array([0, 0, 1])

# Transformation for first link (Elbow position)
T1 = rotation(theta1) @ translation(l1, 0)
    #matirx multiplication
joint1 = T1 @ base

# Transformation for second link (arm position)
T2 = rotation(theta2) @ translation(l2, 0)
    #matrix multiplication          
end_effector = T1 @ T2 @ base

# Print joint positions
    #x,y and z coordinates 
print("Joint1 position:", joint1[:2])
print("End-effector position:", end_effector[:2])

# Plot the robot arm 
x_coords = [base[0], joint1[0], end_effector[0]]
y_coords = [base[1], joint1[1], end_effector[1]]

plt.plot(x_coords, y_coords, '-o', linewidth=3, markersize=10)
plt.xlim(-10, 10)
plt.ylim(-10, 10)
plt.grid(True)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('2D Robot Arm Forward Kinematics')
plt.show()

