import cv2
import apriltag
import numpy as np

# Camera calibration parameters (replace with actual calibration data)
fx = 490.346934509346
cx = 329.879215710305
fy = 491.640993779275
cy = 227.768486995022

# Camera calibration parameters
# These should be determined through calibration for your specific camera
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype="double")

dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion for simplicity

# Define the 3D coordinates of the tag's corners (assuming a tag size of 0.1 meters)
tag_size = 0.1  # Side length of the tag in meters
half_size = tag_size / 2
object_points = np.array([
    [-half_size, -half_size, 0],
    [ half_size, -half_size, 0],
    [ half_size,  half_size, 0],
    [-half_size,  half_size, 0]
])

# Load and process the image
image = cv2.imread('image_with_apriltag.jpg', cv2.IMREAD_GRAYSCALE)
detector = apriltag.Detector()
tags = detector.detect(image)

# Process each detected tag
for tag in tags:
    # The detected corners of the tag
    image_points = np.array(tag.corners, dtype="double")

    # Estimate the pose of the tag
    success, rotation_vector, translation_vector = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs
    )

    if success:
        print(f"Tag ID: {tag.tag_id}")
        print("Rotation Vector:\n", rotation_vector)
        print("Translation Vector:\n", translation_vector)

        # Optionally, convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        print("Rotation Matrix:\n", rotation_matrix)
