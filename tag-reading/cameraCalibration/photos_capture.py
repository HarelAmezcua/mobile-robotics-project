import cv2
import time
import os

# Create a folder to save images if it doesn't exist
save_folder = "captured_images"
os.makedirs(save_folder, exist_ok=True)

# Open the camera
camera = cv2.VideoCapture(1)

if not camera.isOpened():
    print("Error: Camera not accessible.")
    exit()

# Set the number of shots
num_shots = 300

for i in range(1, num_shots + 1):
    # Capture a frame
    ret, frame = camera.read()
    if not ret:
        print(f"Failed to capture image {i}.")
        continue

    # Save the frame as an image file
    image_path = os.path.join(save_folder, f"image_{i:02d}.png")
    cv2.imwrite(image_path, frame)

    # Display the current frame in a window
    cv2.imshow("Current Photo", frame)

    print(f"Shot {i}: {image_path} saved.")
    time.sleep(1)  # Wait for 1 second

    # Break if the user presses the ESC key
    if cv2.waitKey(1) & 0xFF == 27:  # 27 is the ASCII code for ESC
        print("Capture interrupted by user.")
        break

# Release the camera and close OpenCV windows
camera.release()
cv2.destroyAllWindows()

print("All shots taken.")
