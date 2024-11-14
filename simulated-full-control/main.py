import numpy as np
import time
import matplotlib.pyplot as plt
from robot_functions import (
    q_deseada, qp_deseada, robot_movement, plotting_xy,
    plot_wheel_speeds, plot_control_action, plot_desired_vs_actual
)
from initialize_simulation import initialize_parameters, initialize_states, plot_results

# Initialize parameters and states
L, l, dt, Tf, N, gain_matrix = initialize_parameters()
q, qp, q_plot, qp_plot, control_plot, wheel_speed_plot, q_desired_plot, qp_desired_plot, t_plot, dt_plot = initialize_states(N)

# Simulation loop with precise timing for 100Hz execution
start_time = time.perf_counter()
last_control_time = start_time
i = 0

while time.perf_counter() - start_time <= Tf:
    current_time = time.perf_counter()

    # Check if it's time to run the control logic
    if current_time - last_control_time >= dt:
        loop_start_time = time.perf_counter()

        # Desired states
        q_desired_plot[:, i] = q_deseada(current_time - start_time)
        qp_desired_plot[:, i] = qp_deseada(current_time - start_time)

        # Controller (First part - [u_x, u_y, u_theta])
        q_actual = q
        u = qp_desired_plot[:, i] + gain_matrix @ (q_desired_plot[:, i] - q_actual)

        # Convert to wheel speeds
        alpha = q_actual[2] + np.pi / 4
        v = np.array([
            [np.sqrt(2) * np.sin(alpha), -np.sqrt(2) * np.cos(alpha), -(L + l)],
            [np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), (L + l)],
            [np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), -(L + l)],
            [np.sqrt(2) * np.sin(alpha), -np.sqrt(2) * np.cos(alpha), (L + l)]
        ]) @ u

        # Update robot state
        qp = robot_movement(q_actual[2], v, L, l)
        q = q + qp * dt  # use fixed dt

        # Store values for plotting
        q_plot[:, i] = q
        qp_plot[:, i] = qp
        control_plot[:, i] = u
        wheel_speed_plot[:, i] = v
        t_plot[i] = current_time - start_time

        # Record the actual time step
        current_loop_time = time.perf_counter()
        dt_plot[i] = current_loop_time - last_control_time
        last_control_time = current_loop_time

        i += 1

# Trim arrays
q_plot, qp_plot, control_plot, wheel_speed_plot = q_plot[:, :i], qp_plot[:, :i], control_plot[:, :i], wheel_speed_plot[:, :i]
q_desired_plot, qp_desired_plot, t_plot, dt_plot = q_desired_plot[:, :i], qp_desired_plot[:, :i], t_plot[:i], dt_plot[:i]
dt_plot[0] = 0

# Plot results
plot_results(q_plot, q_desired_plot, t_plot, control_plot, wheel_speed_plot, dt_plot)