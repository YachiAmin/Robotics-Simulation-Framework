# 2D Robot Arm Forward Kinematics — n-link generalized
import numpy as np
import matplotlib.pyplot as plt


def rotation(theta_deg):
    #2D rotation matrix.
    rad = np.deg2rad(theta_deg)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])

def translation(dx, dy):
    #2D ous translation matrix.
    return np.array([[1, 0, dx],
                     [0, 1, dy],
                     [0, 0,  1]])

def forward_kinematics(link_lengths, joint_angles):
    """
    Compute joint positions for an n-link planar robot.

    Args:
        link_lengths : list of n floats
        joint_angles : list of n floats (degrees)

    Returns:
        points : (n+1, 2) array — base + one point per link
    """
    T = np.eye(3)
    points = [np.array([0.0, 0.0])]          # base always at origin

    for length, angle in zip(link_lengths, joint_angles):
        T = T @ rotation(angle) @ translation(length, 0)
        point = T @ np.array([0, 0, 1])
        points.append(point[:2])

    return np.array(points)


def plot_arm(points, title="2D Robot Arm — Forward Kinematics"):
    #Plot joint positions and links.
    fig, ax = plt.subplots()
    ax.plot(points[:, 0], points[:, 1], '-o',
            linewidth=3, markersize=10, color='steelblue')

    #Label base and end-effector
    ax.annotate("Base", points[0], textcoords="offset points",
                xytext=(5, 5), fontsize=9)
    ax.annotate("End-effector", points[-1], textcoords="offset points",
                xytext=(5, 5), fontsize=9)

    total = sum(lengths)
    ax.set_xlim(-total - 1, total + 1)
    ax.set_ylim(-total - 1, total + 1)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(title)
    plt.tight_layout()
    plt.show()


# ── Configuration ─────────────────────────────────────────────────────────────
lengths = np.ones(10)        # link lengths  (add/remove freely)
angles  = np.random.randint(0,90,10)      # joint angles in degrees (must match lengths)

# ── Run ───────────────────────────────────────────────────────────────────────
points = forward_kinematics(lengths, angles)

for i, p in enumerate(points):
    label = "Base" if i == 0 else f"Joint {i}" if i < len(points) - 1 else "End-effector"
    print(f"{label}: ({p[0]:.3f}, {p[1]:.3f})")

plot_arm(points)