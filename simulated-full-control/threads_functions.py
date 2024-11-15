# threads_functions.py
import time
import numpy as np
import queue  # To handle queue.Empty exception

def visual_perception_loop(current_pose_shared, pose_queue):
    print("Starting visual perception loop...")
    while True:
        time.sleep(1/30.0)  # Run at approximately 30Hz

        # Get current pose from shared memory
        visual_pose = np.array([current_pose_shared[i] for i in range(3)])
        noise = np.random.normal(0, 0.001, 3)  # Adjust noise as needed
        visual_pose = visual_pose + noise

        # Put the visual pose in the queue, replacing the oldest if full
        if not pose_queue.full():
            pose_queue.put(visual_pose)
            #print("Visual Perception: Put visual_pose into queue:", visual_pose)
        else:
            try:
                pose_queue.get_nowait()  # Remove oldest if full
                pose_queue.put(visual_pose)
                #print("Visual Perception: Queue full. Replaced oldest entry with:", visual_pose)
            except queue.Empty:
                # Handle rare case where the queue was full but another process consumed an item
                pose_queue.put(visual_pose)
                #print("Visual Perception: Queue was full but became available. Put visual_pose:", visual_pose)

def odometry_loop(current_pose_shared, odometry_queue):
    print("Starting odometry loop...")
    while True:
        time.sleep(1/100.0)  # Run at approximately 100Hz

        # Get current pose from shared memory and add noise
        odometry_pose = np.array([current_pose_shared[i] for i in range(3)])
        noise = np.random.normal(0, 0.001, 3)  # Adjust noise as needed
        noisy_odometry_pose = odometry_pose + noise

        # Put the odometry pose in the queue, replacing the oldest if full
        if not odometry_queue.full():
            odometry_queue.put(noisy_odometry_pose)
            #print("Odometry: Put noisy_odometry_pose into queue:", noisy_odometry_pose)
        else:
            try:
                odometry_queue.get_nowait()  # Remove oldest if full
                odometry_queue.put(noisy_odometry_pose)
                #print("Odometry: Queue full. Replaced oldest entry with:", noisy_odometry_pose)
            except queue.Empty:
                # Handle rare case where the queue was full but another process consumed an item
                odometry_queue.put(noisy_odometry_pose)
                #print("Odometry: Queue was full but became available. Put noisy_odometry_pose:", noisy_odometry_pose)