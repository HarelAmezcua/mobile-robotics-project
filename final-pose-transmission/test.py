import cv2
if hasattr(cv2, 'aruco'):
    print("cv2.aruco is available!")
else:
    print("cv2.aruco is not available.")
