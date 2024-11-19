import time
import numpy as np
import queue
import cv2
import socket


def initialize_camera_parameters():
    """Initialize camera calibration parameters and ArUco dictionary."""
    camera_matrix = np.array([
        [808.699859964307, 0, 322.814181940711],
        [0, 809.589244472111, 245.745667049578],
        [0, 0, 1]
    ], dtype="double")
    dist_coeffs = np.array([-0.0360992756163317, -0.185105812278993, 0, 0])
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    return camera_matrix, dist_coeffs, aruco_dict, aruco_params


def process_frame(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length):
    """Process a single camera frame to detect ArUco markers and estimate poses."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    poses = []
    if ids is not None:
        for i, corner in enumerate(corners):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, marker_length, camera_matrix, dist_coeffs
            )
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = tvec.flatten()
            poses.append(transform_matrix)
    return ids, poses


def handle_pose_queue(pose_queue, visual_pose):
    """Handle adding a visual pose to the queue."""
    if not pose_queue.full():
        pose_queue.put(visual_pose)
    else:
        try:
            pose_queue.get_nowait()  # Remove oldest if full
            pose_queue.put(visual_pose)
        except queue.Empty:
            pose_queue.put(visual_pose)


def low_pass_filter(current_value, previous_value, alpha=0.5):
    """Apply a low-pass filter to smooth the values."""
    return alpha * current_value + (1 - alpha) * previous_value


def visual_perception_loop(pose_queue, visual_ready):
    """Main loop for visual perception system."""    
    try:
        # Camera and ArUco initialization
        camera_matrix, dist_coeffs, aruco_dict, aruco_params = initialize_camera_parameters()
        marker_length = 0.095  # Marker length in meters
        cap = cv2.VideoCapture(0)

        # State initialization        
        detected_car = False
        global_frame_set = False
        global_transform = np.eye(4)  # Initial global transformation
        previous_pose = np.zeros(3)  # Initialize previous pose for filtering

        while True:
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame from camera.")
                break

            # Process frame for ArUco markers
            ids, poses = process_frame(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length)

            if ids is not None:
                if not detected_car:
                    with visual_ready.get_lock():
                        visual_ready.value = True
                    print("Visual perception system initialized and ready.")
                    detected_car = True

                for transform_matrix in poses:
                    # Set first marker as global frame
                    if not global_frame_set:
                        global_transform = transform_matrix
                        global_frame_set = True

                    # Compute relative transformation
                    relative_transform = np.linalg.inv(global_transform) @ transform_matrix
                    theta = np.arctan2(relative_transform[1, 0], relative_transform[0, 0])
                    current_pose = np.array([relative_transform[0, 3], relative_transform[1, 3], theta])

                    # Apply low-pass filter
                    filtered_pose = low_pass_filter(current_pose, previous_pose)
                    previous_pose = filtered_pose  # Update previous pose

                    # Add pose to queue
                    handle_pose_queue(pose_queue, filtered_pose)
            else:
                print("No markers detected in this frame.")

    except Exception as e:
        with visual_ready.get_lock():
            visual_ready.value = False
        print(f"Visual perception loop encountered an error: {e}")

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

##################################### ODOMETRY ################################
def odometry_loop(odometry_queue, odometry_ready):
    """Main loop for odometry system."""    
    try:
        # Simulate initialization delay
        time.sleep(2)
        with odometry_ready.get_lock():
            odometry_ready.value = True
        print("Odometry system initialized and ready.")

        previous_pose = np.zeros(3)  # Initialize previous pose for filtering

        while True:
            time.sleep(1 / 100.0)  # Run at approximately 100Hz

            # Generate odometry pose with noise
            odometry_pose = np.array([0, 0, 0]) # + np.random.normal(0, 0.01, 3)

            # Apply low-pass filter
            filtered_pose = low_pass_filter(odometry_pose, previous_pose)
            previous_pose = filtered_pose  # Update previous pose

            # Add odometry pose to queue
            handle_pose_queue(odometry_queue, filtered_pose)

    except Exception as e:
        with odometry_ready.get_lock():
            odometry_ready.value = False
        print(f"Odometry loop encountered an error: {e}")

"""

# Function to handle odometry data
def odometry_loop(receive_socket, odometry_queue, BUFFER_SIZE,odometry_ready):
    
    print("Waiting for data to start flowing...")
    receive_socket.settimeout(0.1)  # Set a timeout for receiving data
    data_started = False

    previous_pose = np.zeros(3)  # Initialize previous pose for filtering
    last_iteration = 0

    while True:
        try:
            # Check if data has started flowing
            if not data_started:
                try:
                    data, addr = receive_socket.recvfrom(BUFFER_SIZE)
                    if data:
                        data_started = True
                        with odometry_ready.get_lock():
                            odometry_ready.value = True
                        print("Data flow started.")
                        receive_socket.settimeout(None)  # Remove timeout after data starts
                except socket.timeout:
                    continue  # Keep waiting if no data yet
            
            
            # Process data
            data, addr = receive_socket.recvfrom(BUFFER_SIZE)
            if data:
                # Parse and update odometry data
                x, y, theta, iteration = map(float, data.decode().split(","))

                if last_iteration < iteration:
                    odometry_pose = np.array([x,y,theta])
                    # Apply low-pass filter
                    filtered_pose = low_pass_filter(odometry_pose, previous_pose)
                    previous_pose = filtered_pose  # Update previous pose
                    # Add odometry pose to queue
                    handle_pose_queue(odometry_queue, filtered_pose) 
                    last_iteration = iteration               

            # Reset the timeout
            last_message_time = time.time()

        ######################### Exception Mangement #########################
        except socket.timeout:
            # Break the loop if no data received for more than 0.1s
            if time.time() - last_message_time > 0.1:
                print("No messages received for 0.1 seconds, exiting loop.")
                with odometry_ready.get_lock():
                    odometry_ready.value = False
                break

        except socket.error as e:
            # Handle socket errors
            with odometry_ready.get_lock():
                if not odometry_ready.value:
                    break  # Exit the loop if stop_event is set
            print(f"Socket error during receiving: {e}")
            time.sleep(1)  # Add a small delay to avoid busy-waiting

        except ValueError as e:
            # Handle data parsing errors
            print(f"Data parsing error: {e}")
            continue  # Skip to the next iteration if data is invalid

            """

############################### SEND WHEEL SPEEDS #############################

"""
# Function to send wheel speed commands
def send_commands(send_socket, stop_event, wheel_speeds, lock, interval=0.01):
    next_time = time.time()
    while not stop_event.is_set():
        with lock:
            command_message = f"{wheel_speeds[0]},{wheel_speeds[1]},{wheel_speeds[2]},{wheel_speeds[3]}"
        try:
            send_socket.sendto(command_message.encode(), (UDP_IP, UDP_PORT_SEND))
            print(f"Sent wheel speeds = {wheel_speeds}")
        except socket.error as e:
            print(f"Socket error during sending: {e}")

        next_time += interval
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_time = time.time()

    # Send final zero velocities before exiting
    final_message = "0.0,0.0,0.0,0.0"
    try:
        send_socket.sendto(final_message.encode(), (UDP_IP, UDP_PORT_SEND))
        print(f"Sent final zero velocities: {final_message}")
    except socket.error as e:
        print(f"Socket error during final sending: {e}")"""