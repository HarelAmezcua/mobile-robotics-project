# main.py
import matplotlib.pyplot as plt
import multiprocessing as mp
import numpy as np
import time
import queue  # To handle queue.Empty exception
from robot_functions import (
    plotting_visual_vs_odometry, plot_against_time, plot_dt
)
from initialize_odometry_test_parameters import initialize_parameters, initialize_states
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

"""Functions"""

def sensor_fusion(visual_pose, odometry_pose, visual_weight=2, odometry_weight=0):
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
    time.sleep(5)  # Adjust if necessary to ensure processes are running

    # Check if the queues have initial data
    while pose_queue.empty() or odometry_queue.empty():
        print("Waiting for initial data in queues...")
        time.sleep(0.1)  # Small delay to avoid excessive CPU usage

    print("Initial data received. Starting control loop.")

    # Initialize parameters and states
    L, l, dt, Tf, N, gain_matrix = initialize_parameters()
    visual_pose, odometry_pose,visual_pose_plot,odometry_pose_plot, t_plot, dt_plot = initialize_states(N)

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
            # Retrieve the latest visual and odometry poses from the queues
            visual_pose = get_latest(pose_queue)

            if visual_pose is None:
                visual_pose = previous_pose_queue
                print("Using previous Pose as visual_pose fallback.")
            else:                
                previous_pose_queue = visual_pose

            odometry_pose = get_latest(odometry_queue)

            if odometry_pose is None:
                odometry_pose = previous_odometry_queue
                print("Using Previous Pose as odometry_pose fallback.")
            else:                
                previous_odometry_queue = odometry_pose

            # Log each pose for plotting            
            visual_pose_plot[:, i] = visual_pose
            odometry_pose_plot[:, i] = odometry_pose

            # Sensor Fusion (balanced weighted average)
            fused_pose = sensor_fusion(visual_pose, odometry_pose)

            # Apply Low-Pass Filter to Fused Pose
            fused_pose = low_pass_filter(fused_pose, previous_fused_pose, alpha=0.1)
            previous_fused_pose = fused_pose                                        

            # Record the actual time step
            current_loop_time = time.perf_counter()
            dt_plot[i] = current_loop_time - last_control_time
            last_control_time = current_loop_time

            # Print the current robot pose for debugging
            print(f"Iteration {i}: Visual Pose: {visual_pose}, Odometry Pose: {odometry_pose}")

            i += 1

    # Optionally, terminate subprocesses gracefully
    for process in processes:
        process.terminate()
        process.join()

    # Trim arrays
    visual_pose_plot, odometry_pose_plot = visual_pose_plot[:, :i], odometry_pose_plot[:, :i]
    t_plot, dt_plot = t_plot[:i], dt_plot[:i]
    dt_plot[0] = 0

    plotting_visual_vs_odometry(visual_pose_plot, odometry_pose_plot)
    plot_against_time(t_plot, visual_pose_plot, odometry_pose_plot)
    plot_dt(dt_plot,t_plot)

if __name__ == '__main__':
    main()
