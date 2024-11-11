import cv2
import numpy as np

# Camera calibration parameters (replace with actual calibration data)
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype="double")
dist_coeffs = np.zeros((4, 1))  # Use actual distortion coefficients if available

# Define the ArUco marker dictionary and parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

# Load the image containing ArUco markers
image = cv2.imread('image_with_aruco.jpg')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect markers
corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

# Specify the actual marker length in meters (e.g., 0.1 m for a 10 cm marker)
marker_length = 0.1

# Estimate pose for each detected marker
if ids is not None:
    for i, corner in enumerate(corners):
        # Directly estimate the pose of the marker
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corner, marker_length, camera_matrix, dist_coeffs
        )
        
        # Display the pose information
        print(f"Marker ID: {ids[i][0]}")
        print("Rotation Vector:\n", rvec)
        print("Translation Vector:\n", tvec)
        
        # Optionally, draw the detected marker and its axes on the image
        cv2.aruco.drawDetectedMarkers(image, corners)
        cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, 0.05)  # Axis length of 0.05 m

    # Show the image with the marker axes
    cv2.imshow("ArUco Markers with Axes", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No markers detected.")
