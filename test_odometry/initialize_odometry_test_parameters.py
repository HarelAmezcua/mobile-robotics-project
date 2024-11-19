# initialize_odometry_test_parameters.py

import numpy as np

def initialize_parameters():
    L = 0.25
    l = 0.20
    dt = 0.00913
    Tf = 10
    N = int(Tf / dt)
    gain_matrix = np.diag([10,10, 10])  # [k_x, k_y, k_theta]
    return L, l, dt, Tf, N, gain_matrix

def initialize_states(N):
    visual_pose = np.array([0, 0, 0])  # Shape (3,)
    odometry_pose = np.zeros(3)        # Shape (3,)

    # Initial values
    fused_pose = np.zeros(3)
    previous_fused_pose = np.zeros(3)    

    visual_pose_plot = np.zeros((3, N))  # Shape (3, N)
    odometry_pose_plot = np.zeros((3, N))  # Shape (3, N)

    t_plot = np.zeros(N)  # Shape (N,)
    dt_plot = np.zeros(N)  # Shape (N,)

    return visual_pose, odometry_pose, visual_pose_plot, odometry_pose_plot, t_plot, dt_plot, fused_pose, previous_fused_pose


