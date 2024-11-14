# config.py
import numpy as np

def initialize_parameters():
    L = 0.25
    l = 0.20
    dt = 0.01
    Tf = 100
    N = int(Tf / dt)
    gain_matrix = np.diag([2, 2, 1])  # [k_x, k_y, k_theta]
    return L, l, dt, Tf, N, gain_matrix


# state.py
import numpy as np

def initialize_states(N):
    q = np.array([-0.1, 0.1, 0])
    qp = np.zeros(3)
    q_plot = np.zeros((3, N))
    qp_plot = np.zeros((3, N))
    control_plot = np.zeros((3, N))
    wheel_speed_plot = np.zeros((4, N))
    q_desired_plot = np.zeros((3, N))
    qp_desired_plot = np.zeros((3, N))
    t_plot = np.zeros(N)
    dt_plot = np.zeros(N)
    q_plot[:, 0] = q
    qp_plot[:, 0] = qp
    return q, qp, q_plot, qp_plot, control_plot, wheel_speed_plot, q_desired_plot, qp_desired_plot, t_plot, dt_plot


# plot_results.py
import matplotlib.pyplot as plt
from robot_functions import plotting_xy, plot_wheel_speeds, plot_control_action, plot_desired_vs_actual

def plot_results(q_plot, q_desired_plot, t_plot, control_plot, wheel_speed_plot, dt_plot):
    plotting_xy(q_plot, q_desired_plot)
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
