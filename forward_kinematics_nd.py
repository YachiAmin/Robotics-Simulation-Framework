import numpy as np
import matplotlib.pyplot as plt

class RobotArm:

#Made up of n links, connected by joints. 

    def __init__(self, lengths, angles):
        #save length of each link 
        self.lengths = np.array(lengths)
        #save angle of each joint at which it is bent 
        self.angles = np.array(angles)
        #joints added later
        self.points = None

    def rotation(self, theta_deg):
        rad = np.deg2rad(theta_deg)
        c, s = np.cos(rad), np.sin(rad)
        return np.array([[c, -s, 0],
                         [s,  c, 0],
                         [0,  0, 1]])

    def translation(self, dx, dy):
        return np.array([[1, 0, dx],
                         [0, 1, dy],
                         [0, 0,  1]])

    def forward_kinematics(self):
        #start at the origin
        T = np.eye(3)
        points = [np.array([0.0, 0.0])]
        for length, angle in zip(self.lengths, self.angles):
            T = T @ self.rotation(angle) @ self.translation(length, 0)
            points.append((T @ np.array([0, 0, 1]))[:2])
        #saving joint positions, plot them later
        self.points = np.array(points)
        return self.points

    
#For moving the arm, passing the NEW ANGLES here
    def update(self, new_angles):
        #new angle check
        if len(new_angles) != len(self.lengths):
            raise ValueError("Number of angles must match number of links")
        #update angles and recalculate everything
        self.angles = np.array(new_angles)
        self.forward_kinematics()

    def plot(self):
        #calculating the position now
        if self.points is None:
            self.forward_kinematics()
        fig, ax = plt.subplots()
        #line through the joint positions, adding dot in every joint
        ax.plot(self.points[:, 0], self.points[:, 1], '-o',linewidth=3, markersize=10, color='steelblue')
        ax.annotate("Base", self.points[0], xytext=(5, 5),textcoords="offset points")
        ax.annotate("End-effector", self.points[-1], xytext=(5, 5),textcoords="offset points")
        total = sum(self.lengths)
        ax.set_xlim(-total - 1, total + 1)
        ax.set_ylim(-total - 1, total + 1)
        ax.set_aspect('equal')
        ax.grid(True)
        plt.show()


# any number of links and angles 

lengths = [3, 2, 1.5, 1, 0.8]
angles = [0, 45, -30, 20, -15]

robot = RobotArm(lengths, angles)
points = robot.forward_kinematics()

for i, point in enumerate(points):
    print(f"Joint {i}: ({point[0]:.3f}, {point[1]:.3f})")
    
#draw the arm 
robot.plot()