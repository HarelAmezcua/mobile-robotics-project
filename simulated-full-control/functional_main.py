import multiprocessing as mp
import numpy as np
import time
from robot_functions import (
    q_deseada, qp_deseada, robot_movement, plotting_xy,
    plot_wheel_speeds, plot_control_action, plot_desired_vs_actual
)
from initialize_simulation import initialize_parameters, initialize_states, plot_results
from threads_functions import visual_perception_loop, odometry_loop
import shared_data

# Initialize a shared array (with size 3 for x, y, theta)
current_pose_shared = mp.Array('d', 3)  # 'd' for double precision
current_pose_shared[0] = 0
current_pose_shared[1] = 0
current_pose_shared[2] = 0

def low_pass_filter(current_v, previous_v, alpha=0.1):
    """Apply a low-pass filter to smooth the velocity values."""
    return alpha * current_v + (1 - alpha) * previous_v

def main():

    # Launch processes for each loop without blocking the main loop
    processes = [
        mp.Process(target=visual_perception_loop, daemon=True, args=(current_pose_shared,)),
        #mp.Process(target=odometry_loop, daemon=True),
    ]

    for process in processes:
        process.start()

    # Initialize parameters and states
    L, l, dt, Tf, N, gain_matrix = initialize_parameters()
    q, qp, q_plot, qp_plot, control_plot, wheel_speed_plot, q_desired_plot, qp_desired_plot, t_plot, dt_plot = initialize_states(N)

    # Simulation loop with precise timing for 100Hz execution
    start_time = time.perf_counter()
    last_control_time = start_time
    i = 0

    current_pose = q
    previous_v = np.zeros(4)  # Initial previous velocity for the filter

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

            # Apply low-pass filter to smooth out the wheel speeds
            v = low_pass_filter(v, previous_v)
            previous_v = v  # Update previous velocity for the next iteration

            # Update robot state
            qp = robot_movement(q_actual[2], v, L, l)
            q = q + qp * dt  # use fixed dt

            # Define the noise parameters
            mean = 0  # Mean of the normal distribution
            std_dev = 0.001  # Standard deviation of the normal distribution

            # Generate normal noise with the same shape as data
            noise = np.random.normal(mean, std_dev, current_pose.shape)
            q = q + noise

            #estimated_pose_aux = current_pose + noise
            current_pose_shared[0] = q[0]
            current_pose_shared[1] = q[1]
            current_pose_shared[2] = q[2]

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

if __name__ == '__main__':
    main()