# main.py
import matplotlib.pyplot as plt
import multiprocessing as mp
import numpy as np
import time
import queue  # To handle queue.Empty exception
from robot_functions import (
    q_deseada, qp_deseada, robot_movement, plotting_xy,
    plot_wheel_speeds, plot_control_action, plot_desired_vs_actual
)
from initialize_simulation import initialize_parameters, initialize_states, plot_results
from threads_functions import visual_perception_loop, odometry_loop

# Initialize shared pose array (size 3 for x, y, theta)
# Using multiprocessing.Array for shared memory
current_pose_shared = mp.Array('d', 3)  # 'd' for double precision
current_pose_shared[0] = 0
current_pose_shared[1] = 0
current_pose_shared[2] = 0

# Set previous pose when queue is empty
previous_pose_queue = np.array([0, 0, 0])
previous_odometry_queue = np.array([0, 0, 0])

def sensor_fusion(visual_pose, odometry_pose, visual_weight=2, odometry_weight=0.5):
    """Perform a weighted average for sensor fusion."""
    return (visual_weight * np.array(visual_pose) + odometry_weight * np.array(odometry_pose))/(visual_weight+odometry_weight)

def low_pass_filter(current_value, previous_value, alpha=0.05):
    """Apply a low-pass filter to smooth the values."""
    return alpha * current_value + (1 - alpha) * previous_value

def get_latest(q):
    """Retrieve the latest item from the queue, discarding older ones."""
    latest = None
    while True:
        try:
            latest = q.get_nowait()
        except queue.Empty:
            break
    return latest

def main():
    # Create a Manager to handle shared queues
    manager = mp.Manager()
    pose_queue = manager.Queue(maxsize=100)       # Define appropriate maxsize
    odometry_queue = manager.Queue(maxsize=100)   # Define appropriate maxsize

    # Launch processes for visual perception and odometry, passing queues as arguments
    processes = [
        mp.Process(target=visual_perception_loop, daemon=True, args=(current_pose_shared, pose_queue)),
        mp.Process(target=odometry_loop, daemon=True, args=(current_pose_shared, odometry_queue))
    ]

    for process in processes:
        process.start()

    # Add a short delay to allow the processes to start and populate the queues
    time.sleep(1)  # Adjust if necessary to ensure processes are running

    # Check if the queues have initial data
    while pose_queue.empty() or odometry_queue.empty():
        print("Waiting for initial data in queues...")
        time.sleep(0.1)  # Small delay to avoid excessive CPU usage

    print("Initial data received. Starting control loop.")

    # Initialize parameters and states
    L, l, dt, Tf, N, gain_matrix = initialize_parameters()
    q, qp, q_plot, qp_plot, control_plot, wheel_speed_plot, q_desired_plot, qp_desired_plot, t_plot, dt_plot = initialize_states(N)

    # Trajectory logs for real, visual, and odometry poses
    real_trajectory = np.zeros((3, N))
    visual_trajectory = np.zeros((3, N))
    odometry_trajectory = np.zeros((3, N))

    start_time = time.perf_counter()
    last_control_time = start_time
    i = 0

    # Initial values
    fused_pose = np.zeros(3)
    previous_fused_pose = np.zeros(3)
    previous_v = np.zeros(4)

    while time.perf_counter() - start_time <= Tf:
        current_time = time.perf_counter()

        # Check if it's time to run the control logic
        if current_time - last_control_time >= dt:
            # Retrieve the current real pose from shared memory
            real_pose = np.array([current_pose_shared[0], current_pose_shared[1], current_pose_shared[2]])

            # Retrieve the latest visual and odometry poses from the queues
            visual_pose = get_latest(pose_queue)

            if visual_pose is None:
                visual_pose = previous_pose_queue
                print("Using previous Pose as visual_pose fallback.")
            else:
                #print("Retrieved latest visual_pose from queue:", visual_pose)
                previous_pose_queue = visual_pose

            odometry_pose = get_latest(odometry_queue)
            if odometry_pose is None:
                odometry_pose = previous_odometry_queue
                print("Using Previous Pose as odometry_pose fallback.")
            else:
                #print("Retrieved latest odometry_pose from queue:", odometry_pose)
                previous_odometry_queue = odometry_pose

            # Log each pose for plotting
            real_trajectory[:, i] = real_pose
            visual_trajectory[:, i] = visual_pose
            odometry_trajectory[:, i] = odometry_pose

            # Sensor Fusion (balanced weighted average)
            fused_pose = sensor_fusion(visual_pose, odometry_pose)

            # Apply Low-Pass Filter to Fused Pose
            fused_pose = low_pass_filter(fused_pose, previous_fused_pose, alpha=0.1)
            previous_fused_pose = fused_pose

            # Desired states
            q_desired_plot[:, i] = q_deseada(current_time - start_time)
            qp_desired_plot[:, i] = qp_deseada(current_time - start_time)

            # Compute error
            error = q_desired_plot[:, i] - fused_pose

            # Update integral of error
            integral_error += error * dt
            integral_error = np.clip(integral_error, -0.1, 0.1)

            # Controller logic with integral action
            # Assuming gain_matrix is the proportional gain (K_p)
            # u = K_p * error + K_i * integral_error + qp_desired
            u = qp_desired_plot[:, i] + gain_matrix @ error + K_i * integral_error

            # Convert control action to wheel speeds
            alpha_angle = fused_pose[2] + np.pi / 4
            v = np.array([
                [np.sqrt(2) * np.sin(alpha_angle), -np.sqrt(2) * np.cos(alpha_angle), -(L + l)],
                [np.sqrt(2) * np.cos(alpha_angle), np.sqrt(2) * np.sin(alpha_angle), (L + l)],
                [np.sqrt(2) * np.cos(alpha_angle), np.sqrt(2) * np.sin(alpha_angle), -(L + l)],
                [np.sqrt(2) * np.sin(alpha_angle), -np.sqrt(2) * np.cos(alpha_angle), (L + l)]
            ]) @ u

            # Apply low-pass filter to wheel speeds
            v = low_pass_filter(v, previous_v, alpha=0.05)
            previous_v = v

            # Adding sinusoidal disturbance as before
            v[0] = v[0] + 0.05 * np.sin(1 * (current_time - start_time))
            v[1] = v[1] + 0.05 * np.sin(1 * (current_time - start_time)+1)
            v[2] = v[2] + 0.05 * np.sin(1 * (current_time - start_time)+2)
            v[3] = v[3] + 0.05 * np.sin(1 * (current_time - start_time)+3)

            # Update robot state
            qp = robot_movement(fused_pose[2], v, L, l)
            q = q + qp * dt

            noise = np.random.normal(0, 0.001, 3)  # Adjust noise as needed
            q = q + noise

            # Update the shared real pose after computing new q
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

            # Print the current robot pose for debugging
            print(f"Iteration {i}: Real Pose: {q}, Fused Pose: {fused_pose}, Error: {error}, Integral Error: {integral_error}")

            i += 1

    # Optionally, terminate subprocesses gracefully
    for process in processes:
        process.terminate()
        process.join()

    # Trim arrays
    q_plot, qp_plot, control_plot, wheel_speed_plot = q_plot[:, :i], qp_plot[:, :i], control_plot[:, :i], wheel_speed_plot[:, :i]
    q_desired_plot, qp_desired_plot, t_plot, dt_plot = q_desired_plot[:, :i], qp_desired_plot[:, :i], t_plot[:i], dt_plot[:i]
    dt_plot[0] = 0
    # Trim arrays and plot results as in previous examples
    plot_results(q_plot, q_desired_plot, t_plot, control_plot, wheel_speed_plot, dt_plot)

    # After trimming arrays
    plot_trajectories(real_trajectory[:, :i], visual_trajectory[:, :i], odometry_trajectory[:, :i], t_plot, i)

if __name__ == '__main__':
    main()
