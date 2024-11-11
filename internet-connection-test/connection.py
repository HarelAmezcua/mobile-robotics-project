import socket
import time
import threading
import tkinter as tk

# Set up the UDP communication
UDP_IP = "192.168.137.235"  # ESP32 IP Address
UDP_PORT_SEND = 12345  # Port to send commands to ESP32
UDP_PORT_RECEIVE = 12346  # Port to receive data from ESP32
BUFFER_SIZE = 1024  # Buffer size for receiving data

# Create UDP sockets for sending and receiving
send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receive_socket.bind(("", UDP_PORT_RECEIVE))

# Variables for wheel speeds
wheel_speeds = [0.0, 0.0, 0.0, 0.0]
stop_event = threading.Event()
lock = threading.Lock()  # Lock for thread-safe access to wheel_speeds

# Function to send wheel speed commands
def send_commands():
    interval = 0.01  # 100Hz loop rate
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
        print(f"Socket error during final sending: {e}")

# Function to receive position and velocity data
def receive_data():
    while not stop_event.is_set():
        try:
            data, addr = receive_socket.recvfrom(BUFFER_SIZE)
            if data:
                position_velocity = data.decode()
                print(f"Received position (x, y, v, w) = {position_velocity}")
        except socket.error as e:
            if stop_event.is_set():
                break  # Exit the loop if stop_event is set
            print(f"Socket error during receiving: {e}")
            time.sleep(1)

# Control loop to compute and update wheel speeds
def control_loop():
    while not stop_event.is_set():
        with lock:
            wheel_speeds[0] = 1.0
            wheel_speeds[1] = 1.0
            wheel_speeds[2] = 1.0
            wheel_speeds[3] = 1.0
        time.sleep(0.1)

# Function to stop the program
def stop_program():
    stop_event.set()  # Signal threads to stop
    root.after(100, cleanup_resources)  # Schedule cleanup after 100 ms

def cleanup_resources():
    # Wait for threads to stop before closing sockets
    send_thread.join(timeout=1)
    receive_thread.join(timeout=1)
    control_thread.join(timeout=1)

    # Now it's safe to close the sockets
    send_socket.close()
    receive_socket.close()

    root.quit()  # Stop the tkinter main loop
    root.destroy()  # Close the UI

# Create and start threads
send_thread = threading.Thread(target=send_commands)
receive_thread = threading.Thread(target=receive_data)
control_thread = threading.Thread(target=control_loop)

send_thread.start()
receive_thread.start()
control_thread.start()

# Create the UI with tkinter
root = tk.Tk()
root.title("Robot Control")
stop_button = tk.Button(root, text="Stop", command=stop_program, bg="red", fg="white")
stop_button.pack(pady=40)
root.mainloop()
