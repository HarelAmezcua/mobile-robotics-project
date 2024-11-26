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
        cap = cv2.VideoCapture(2)

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