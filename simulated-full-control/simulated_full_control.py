import threading
import time
import queue
import numpy as np
from threads_functions import visual_perception_loop
from threads_functions import odometry_loop
from threads_functions import control_loop
from shared_data import control_command_queue

# Launch threads for each loop
threads = [
    threading.Thread(target=visual_perception_loop, daemon=True),
    threading.Thread(target=odometry_loop, daemon=True),
    threading.Thread(target=control_loop, daemon=True)
]

for thread in threads:
    thread.start()

# Main program loop (for demonstration purposes, run indefinitely)
try:
    while True:
        # For example, read the latest control command sent to robot
        try:
            control_command = control_command_queue.get_nowait()
            print("Latest Control Command:", control_command)
            print("-------------")
        except queue.Empty:
            # No control command available at the moment
            pass
        
        time.sleep(0.1)  # Simulate other tasks in main loop

except KeyboardInterrupt:
    print("Shutting down...")
