import cv2
import numpy as np
import time

# Camera calibration parameters (replace with actual calibration data)
fx = 808.699859964307
cx = 322.814181940711
fy = 809.589244472111
cy = 245.745667049578

camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype="double")
dist_coeffs = np.array([-0.0360992756163317,-0.185105812278993,0,0])  # Use actual distortion coefficients if available

# Define the ArUco marker dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# Specify the actual marker length in meters (e.g., 0.1 m for a 10 cm marker)
marker_length = 0.095

# Open the camera (0 is usually the default camera)
cap = cv2.VideoCapture(2)
# Initialize time for the first loop and global frame
prev_time = time.time()
global_frame_set = False
global_transform = np.eye(4)  # Initial global transformation as identity matrix

# Loop to capture frames and process them
while True:
    # Calculate time passed since last loop
    current_time = time.time()
    time_passed = current_time - prev_time
    print(f"Time passed since last loop: {time_passed:.4f} seconds")
    prev_time = current_time

    # Capture a frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame from camera.")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    # Estimate pose for each detected marker
    if ids is not None:
        for i, corner in enumerate(corners):
            # Estimate the pose of the marker
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, marker_length, camera_matrix, dist_coeffs
            )

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            # Create the homogeneous transformation matrix
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = tvec.flatten()

            # Set the first marker detected as the global frame
            if not global_frame_set:
                global_transform = transform_matrix
                global_frame_set = True

            # Compute the transformation relative to the initial global frame
            relative_transform = np.linalg.inv(global_transform) @ transform_matrix

            theta = np.arctan2(relative_transform[1, 0], relative_transform[0, 0])

            # Print the homogeneous transformation matrix
            print(f"Marker ID: {ids[i][0]}")
            print("Relative Transformation Matrix:\n", relative_transform)
            print("theta: {}",format(theta))

            # Draw the detected marker and its axes on the frame
            cv2.aruco.drawDetectedMarkers(frame, corners)
            #cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)  # Axis length of 0.05 m

    else:
        print("No markers detected in this frame.")

    # Display the frame with detected markers and axes
    cv2.imshow("ArUco Markers with Axes", frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()