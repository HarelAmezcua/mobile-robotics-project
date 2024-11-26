# main.py
import matplotlib.pyplot as plt
import multiprocessing as mp
import numpy as np
import time
import queue  # To handle queue.Empty exception
from robot_functions import (
    plotting_visual_vs_odometry, plot_against_time, plot_dt
)
from initialize_odometry_test_parameters import (initialize_parameters, initialize_states)
from aux_functions import (sensor_fusion, low_pass_filter, initialize_queues_processes,wait_for_data, check_system_status, retrieve_pose, loggin_stuff)


def main():

    # Set previous pose when queue is empty
    previous_visual_pose = np.array([0, 0, 0])
    previous_odometry_pose = np.array([0, 0, 0])

    manager,pose_queue,odometry_queue,odometry_ready,visual_ready,processes = initialize_queues_processes()
    wait_for_data(visual_ready,odometry_ready,pose_queue,odometry_queue)


    # Initialize parameters and states
    L, l, dt, Tf, N, gain_matrix = initialize_parameters()
    visual_pose, odometry_pose,visual_pose_plot,odometry_pose_plot, t_plot, dt_plot, fused_pose, previous_fused_pose = initialize_states(N)

    start_time = time.perf_counter()
    last_control_time = start_time
    i = 0

    try:
        while time.perf_counter() - start_time <= Tf:
            current_time = time.perf_counter()

            # Check if it's time to run the control logic
            if current_time - last_control_time >= dt:

                # Check if systems are still operational
                if not check_system_status(visual_ready, odometry_ready):
                    break
                
                # Retrieve actual pose
                visual_pose, previous_visual_pose = retrieve_pose(pose_queue, previous_visual_pose, "visual_pose")
                odometry_pose, previous_odometry_pose = retrieve_pose(odometry_queue, previous_odometry_pose, "odometry_pose")
                

                fused_pose = sensor_fusion(visual_pose, odometry_pose) # Sensor Fusion (balanced weighted average)   
                fused_pose = low_pass_filter(fused_pose, previous_fused_pose, alpha=0.05) # Apply Low-Pass Filter to Fused Pose
                previous_fused_pose = fused_pose      


                # Time and Logging stuff
                current_loop_time = time.perf_counter()
                loggin_stuff(visual_pose_plot,visual_pose,odometry_pose_plot,odometry_pose,dt_plot,current_loop_time,last_control_time,t_plot,current_time, start_time,i)
                last_control_time = current_loop_time

                # Print the current robot pose for debugging
                print(f"Iteration {i}: Visual Pose: {visual_pose}, Odometry Pose: {odometry_pose}")
                i += 1

    except KeyboardInterrupt:
        print("Interrupted by user. Stopping control loop.")

    finally:        
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
