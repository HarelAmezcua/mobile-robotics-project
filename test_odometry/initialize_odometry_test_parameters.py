# config.py
import numpy as np

def initialize_parameters():
    L = 0.25
    l = 0.20
    dt = 0.01
    Tf = 20
    N = int(Tf / dt)
    gain_matrix = np.diag([10,10, 10])  # [k_x, k_y, k_theta]
    return L, l, dt, Tf, N, gain_matrix


# state.py
import numpy as np

def initialize_states(N):
    visual_pose = np.array([0, 0, 0])
    odometry_pose = np.zeros((3,1))

    visual_pose_plot = np.zeros((3, N))
    odometry_pose_plot = np.zeros((3,N))

    t_plot = np.zeros(N)
    dt_plot = np.zeros(N)

    visual_pose_plot[:, 0] = visual_pose
    odometry_pose_plot[:, 0] = odometry_pose

    return visual_pose, odometry_pose,visual_pose_plot,odometry_pose_plot, t_plot, dt_plot


# plot_results.py
import matplotlib.pyplot as plt
from robot_functions import plotting_xy, plot_wheel_speeds, plot_control_action, plot_desired_vs_actual

def plot_results(q_plot, q_desired_plot, t_plot, control_plot, wheel_speed_plot, dt_plot):
    plotting_xy(q_plot)
    plot_desired_vs_actual(t_plot, q_plot, q_desired_plot)
    plot_control_action(t_plot, control_plot)
    plot_wheel_speeds(t_plot, wheel_speed_plot)
    
    plt.figure()
    plt.plot(t_plot, dt_plot, "b", linewidth=2)
    plt.ylabel("dt (s)")
    plt.xlabel("Time [s]")
    plt.title("Actual Time Steps")
    plt.grid()
    plt.tight_layout()
    plt.show()
