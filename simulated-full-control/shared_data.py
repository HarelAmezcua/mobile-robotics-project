# shared_data.py
import queue
import numpy as np

pose_queue = queue.Queue(maxsize=1)
odometry_queue = queue.Queue(maxsize=1)
control_command_queue = queue.Queue(maxsize=1)

current_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
current_velocity = np.array([0.0, 0.0])  # v, omega