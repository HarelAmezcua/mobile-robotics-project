import numpy as np
from auxiliar_functions.plot_robot import animate_trajectory

# Example usage
# Create a sample trajectory (3 DOF: x, y, theta) with 100 time steps
N = 1000
trajectory = np.zeros((3, N))
trajectory[0, :] = np.linspace(0, 2, N)  # x positions
trajectory[1, :] = np.linspace(0, 2, N)  # y positions
trajectory[2, :] = np.linspace(0, np.pi, N)  # theta (orientation)

# Time step for each frame
dt = 0.01  # seconds

# Animate the trajectory
animate_trajectory(trajectory, L=0.3, l=0.25, dt=dt)