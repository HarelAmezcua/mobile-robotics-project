# initialize_odometry_test_parameters.py

import numpy as np

def initialize_parameters():
    L = 0.25
    l = 0.20
    dt = 0.03
    Tf = 10
    N = int(Tf / dt)
    return dt, Tf, N

def initialize_states(N):
    visual_pose = np.array([0, 0, 0])  # Shape (3,)    

    visual_pose_plot = np.zeros((3, N))  # Shape (3, N)    

    t_plot = np.zeros(N)  # Shape (N,)
    dt_plot = np.zeros(N)  # Shape (N,)

    return visual_pose, visual_pose_plot, t_plot, dt_plot


