import socket
import time
import threading

# Set up the UDP communication
UDP_IP = "192.168.137.25"  # ESP32 IP Address
UDP_PORT_SEND = 12345  # Port to send commands
UDP_PORT_RECEIVE = 12346  # Port to receive data from ESP32
BUFFER_SIZE = 1024  # Buffer size for receiving data

# Create UDP socket for sending and receiving
send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receive_socket.bind(("", UDP_PORT_RECEIVE))

# Variables for wheel speeds
wheel_speeds = [0.0, 0.0, 0.0, 0.0]

# Function to send wheel speed commands
def send_commands():
    while True:
        # Prepare the command message as a string
        command_message = f"{wheel_speeds[0]},{wheel_speeds[1]},{wheel_speeds[2]},{wheel_speeds[3]}"
        send_socket.sendto(command_message.encode(), (UDP_IP, UDP_PORT_SEND))
        time.sleep(0.01)  # 100Hz loop rate (0.01s interval)

# Function to receive position and velocity data
def receive_data():
    while True:
        data, addr = receive_socket.recvfrom(BUFFER_SIZE)
        if data:
            # Parse the received message
            position_velocity = data.decode()
            print("Received:", position_velocity)

# Create and start threads for sending and receiving
send_thread = threading.Thread(target=send_commands)
receive_thread = threading.Thread(target=receive_data)

send_thread.start()
receive_thread.start()

# Main loop to update wheel speeds (example)
try:
    while True:
        # Example of updating wheel speeds
        wheel_speeds[0] = 1.0  # Example value
        wheel_speeds[1] = 1.0  # Example value
        wheel_speeds[2] = 1.0  # Example value
        wheel_speeds[3] = 1.0  # Example value
        time.sleep(1)  # Adjust speed every second (or as needed)
except KeyboardInterrupt:
    print("Stopping...")
