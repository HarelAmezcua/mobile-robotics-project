import numpy as np
import time
from robot_functions import q_deseada, qp_deseada, robot_movement


# Robot parameters
L = 0.25
l = 0.20

# Simulation parameters
dt = 0.01
S = 30
N = int(S / dt)

# Pre-allocate storage matrices
q_plot = np.zeros((3, N))
qp_plot = np.zeros((3, N))
t_plot = np.arange(0, N) * dt  # Fixed the size of tPlot
control_plot = np.zeros((3, N))
wheel_speed_plot = np.zeros((4, N))
q_desired_plot = np.zeros((3, N))
qp_desired_plot = np.zeros((3, N))


# Initial condition
q = np.array([-5, 5, np.pi / 4])
qp = np.zeros(3)
q_plot[:, 0] = q
qp_plot[:, 0] = qp

# Gains
k_x, k_y, k_theta = 1, 1, 1
gain_matrix = np.diag([k_x, k_y, k_theta])

# Simulation
i = 1
for t in np.arange(dt, S, dt):
    q_desired_plot[:, i] = q_deseada(t)
    qp_desired_plot[:, i] = qp_deseada(t)

    # Controller (First part - [u_x, u_y, u_theta])
    q_actual = q
    u = qp_deseada(t) + gain_matrix @ (q_deseada(t) - q_actual)

    # Convert to wheel speeds
    alpha = q_actual[2] + np.pi / 4
    v = np.array([
        [np.sqrt(2) * np.sin(alpha), -np.sqrt(2) * np.cos(alpha), -(L + l)],
        [np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), (L + l)],
        [np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), -(L + l)],
        [np.sqrt(2) * np.sin(alpha), -np.sqrt(2) * np.cos(alpha), (L + l)]
    ]) @ u

    qp = robot_movement(q_actual[2], v)
    q = q + qp * dt

    q_plot[:, i] = q
    qp_plot[:, i] = qp
    control_plot[:, i] = u
    wheel_speed_plot[:, i] = v

    i += 1