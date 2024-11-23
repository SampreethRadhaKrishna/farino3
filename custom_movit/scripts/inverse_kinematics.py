import numpy as np

def inverse_kinematics(x, y, z, w, dh_params):
    """
    Compute joint angles for a 6-DOF robot using DH parameters and end-effector pose.
    
    Parameters:
        x, y, z (float): Position of the end effector in Cartesian space.
        w (float): Orientation of the end effector (simplified as yaw or quaternion data can be used).
        dh_params (list of tuples): Denavit-Hartenberg parameters as [(a, alpha, d, theta), ...].
        
    Returns:
        joint_positions (list): Joint angles [j1, j2, j3, j4, j5, j6].
    """
    # Initialize joint positions
    joint_positions = [0] * 6
    
    # Step 1: Solve for base rotation (j1)
    joint_positions[0] = np.arctan2(y, x)
    
    # Step 2: Solve for planar position (j2, j3)
    r = np.sqrt(x**2 + y**2)  # Distance to base
    z_offset = z - dh_params[0][2]  # Correct for base height
    L1 = dh_params[1][0]  # Length of link 1
    L2 = dh_params[2][0]  # Length of link 2
    
    cos_theta2 = (r**2 + z_offset**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(np.clip(cos_theta2, -1, 1))
    joint_positions[2] = np.pi - theta2  # Elbow up configuration

    # Solve for shoulder angle (j2)
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    joint_positions[1] = np.arctan2(z_offset, r) - np.arctan2(k2, k1)

    # Step 3: Solve for wrist orientation (j4, j5, j6)
    # Compute transformation matrix from base to wrist
    T_base_to_wrist = np.array([
        [np.cos(w), -np.sin(w), 0, x],
        [np.sin(w), np.cos(w),  0, y],
        [0,         0,          1, z],
        [0,         0,          0, 1]
    ])
    
    # Solve for j4, j5, j6 (orientation angles)
    # Assuming a standard RPY decomposition or directly solving with rotation matrices
    R_ee = T_base_to_wrist[:3, :3]
    joint_positions[3] = np.arctan2(R_ee[2, 1], R_ee[2, 2])
    joint_positions[4] = np.arccos(np.clip(R_ee[2, 0], -1, 1))
    joint_positions[5] = np.arctan2(R_ee[1, 0], R_ee[0, 0])
    
    return joint_positions


# Define the DH parameters for your robot
# [(a_i, alpha_i, d_i, theta_i), ...] for each joint
dh_params = [
    (0, 0, 0.2, 0),   # Base link
    (0.3, -np.pi/2, 0, 0),  # Shoulder
    (0.25, 0, 0, 0),  # Upper arm
    (0.2, -np.pi/2, 0, 0),  # Forearm
    (0, np.pi/2, 0, 0),     # Wrist 1
    (0, -np.pi/2, 0, 0)     # Wrist 2
]

# Target pose
x, y, z = 0.4, 0.4, 0.5
w = np.pi / 4  # Orientation (yaw angle)

# Compute joint positions
joint_positions = inverse_kinematics(x, y, z, w, dh_params)
print("Joint Angles:", joint_positions)
