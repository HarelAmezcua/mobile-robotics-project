import numpy as np
import multiprocessing as mp
import queue  # To handle queue.Empty exception
import time
from threads_functions import visual_perception_loop, odometry_loop


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

def initialize_queues_processes():
    # Create a Manager to handle shared queues
    manager = mp.Manager()
    pose_queue = manager.Queue(maxsize=100)       # Define appropriate maxsize
    odometry_queue = manager.Queue(maxsize=100)   # Define appropriate maxsize

    # Create shared boolean flags
    odometry_ready = mp.Value('b',False)
    visual_ready = mp.Value('b',False)


    # Launch processes for visual perception and odometry, passing queues as arguments
    processes = [
        mp.Process(target=visual_perception_loop, daemon=True, args=(pose_queue,visual_ready)),
        mp.Process(target=odometry_loop, daemon=True, args=(odometry_queue,odometry_ready))
    ]

    for process in processes:
        process.start()

    return manager,pose_queue,odometry_queue,odometry_ready,visual_ready,processes


def wait_for_data(visual_ready,odometry_ready,pose_queue,odometry_queue):
    # Wait until both visual and odometry are ready
    print("Waiting for visual and odometry systems to initialize...")

    while True:
        with visual_ready.get_lock(), odometry_ready.get_lock():
            if visual_ready.value and odometry_ready.value:
                break
        print("Waiting for systems to be ready...")
        time.sleep(0.5)  # Adjust the sleep duration as needed

    # Check if the queues have initial data
    while pose_queue.empty() or odometry_queue.empty():
        print("Waiting for initial data in queues...")
        time.sleep(0.1)  # Small delay to avoid excessive CPU usage

    print("Initial data received. Starting control loop.")


def check_system_status(visual_ready, odometry_ready):
    """
    Check if the visual and odometry systems are operational.

    Args:
        visual_ready (Synchronized): Shared value indicating visual system status.
        odometry_ready (Synchronized): Shared value indicating odometry system status.

    Returns:
        bool: True if systems are operational, False otherwise.
    """
    with visual_ready.get_lock(), odometry_ready.get_lock():
        if not visual_ready.value or not odometry_ready.value:
            print("Visual or Odometry system failed. Stopping control loop safely.")
            return False
    return True

def retrieve_pose(queue, previous_pose, pose_name):
    """
    Retrieve the latest pose from a queue, falling back to the previous pose if necessary.

    Args:
        queue (Queue): The queue holding pose data.
        previous_pose: The previous pose to use as a fallback.
        pose_name (str): Name of the pose for logging purposes.

    Returns:
        tuple: A tuple containing the retrieved pose and the updated previous pose.
    """
    latest_pose = get_latest(queue)
    if latest_pose is None:
        print(f"Using previous pose as {pose_name} fallback.")
        return previous_pose, previous_pose
    return latest_pose, latest_pose


def loggin_stuff(visual_pose_plot,visual_pose,odometry_pose_plot,odometry_pose,dt_plot,current_loop_time,last_control_time,t_plot,current_time, start_time,i):
    # Log each pose for plotting            
    visual_pose_plot[:, i] = visual_pose
    odometry_pose_plot[:, i] = odometry_pose                                  
    
    dt_plot[i] = current_loop_time - last_control_time
    t_plot[i] = current_time - start_time