
from shared_data import pose_queue, current_pose, odometry_queue, control_command_queue, current_velocity
import time
import queue
import numpy as np

# Visual Perception Loop (e.g., ArUco pose estimation)
def visual_perception_loop():
    global current_pose
    while True:
        # Simulate pose detection at 30Hz
        time.sleep(1/30.0)
        
        # Update the pose (here, randomly generating a new pose)
        detected_pose = np.random.rand(3)
        
        # Put latest pose in queue
        if not pose_queue.full():
            pose_queue.put(detected_pose)
        else:
            pose_queue.get_nowait()
            pose_queue.put(detected_pose)
        
        print("Visual Perception Updated Pose:", detected_pose)

# Odometry Loop (e.g., retrieving odometry data over WiFi)
def odometry_loop():
    global current_velocity
    while True:
        # Simulate receiving odometry data at 100Hz
        time.sleep(1/100.0)
        
        # Update the velocity (random data for example)
        detected_velocity = np.random.rand(2)
        
        # Put latest odometry in queue
        if not odometry_queue.full():
            odometry_queue.put(detected_velocity)
        else:
            odometry_queue.get_nowait()
            odometry_queue.put(detected_velocity)
        
        print("Odometry Updated Velocity:", detected_velocity)


# Control Loop (compute and send control commands at 100Hz)
def control_loop():
    global current_pose, current_velocity
    while True:
        # Run control logic at 100Hz
        time.sleep(1/100.0)
        
        # Fetch the latest pose and velocity data
        try:
            latest_pose = pose_queue.get_nowait()
            current_pose = latest_pose
        except queue.Empty:
            # Use the last known pose if no new data
            latest_pose = current_pose

        try:
            latest_velocity = odometry_queue.get_nowait()
            current_velocity = latest_velocity
        except queue.Empty:
            # Use the last known velocity if no new data
            latest_velocity = current_velocity

        # Simple control computation (example: PID controller)
        # Here just generating a dummy control signal for example
        control_command = current_pose[:2] * 0.1 + current_velocity * 0.05
        
        # Put control command in queue for output (or send to robot)
        if not control_command_queue.full():
            control_command_queue.put(control_command)
        else:
            control_command_queue.get_nowait()
            control_command_queue.put(control_command)
        
        print("Control Command Sent:", control_command)
