import numpy as np
import matplotlib.pyplot as plt

def forward_kinematics(joint_angles):
    # Lengths of each segment (in meters)
    lengths = [1, 1, 1]

    # Initialize end-effector position
    x, y = 0, 0

    # Calculate end-effector position
    for i, angle in enumerate(joint_angles):
        x += lengths[i] * np.cos(np.sum(joint_angles[:i+1]))
        y += lengths[i] * np.sin(np.sum(joint_angles[:i+1]))

    return x, y

def plot_robot_arm(joint_angles):
    # Lengths of each segment (in meters)
    lengths = [1, 1, 1]

    # Calculate end-effector position
    end_effector = forward_kinematics(joint_angles)

    # Calculate segment endpoints
    segment_endpoints = [(0, 0)]  # Start at base
    x, y = 0, 0
    for i, angle in enumerate(joint_angles):
        x += lengths[i] * np.cos(np.sum(joint_angles[:i+1]))
        y += lengths[i] * np.sin(np.sum(joint_angles[:i+1]))
        segment_endpoints.append((x, y))

    # Plot robot arm
    plt.figure()
    plt.plot([p[0] for p in segment_endpoints], [p[1] for p in segment_endpoints], 'bo-')
    plt.plot([0], [0], 'ro')  # Base
    plt.plot(end_effector[0], end_effector[1], 'go')  # End-effector
    plt.xlim([-3.5, 3.5])  # Adjust as needed
    plt.ylim([-3.5, 3.5])  # Adjust as needed
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('3-DOF Robot Arm')
    plt.grid(True)
    plt.show()

# Example usage
joint_angles = [np.pi/2, 0, 0]  # Set joint angles
plot_robot_arm(joint_angles)
