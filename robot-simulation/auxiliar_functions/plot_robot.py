import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def animate_trajectory(trajectory, L, l, dt):
    """
    Creates an animation of the traced trajectory for an omnidirectional robot.

    Parameters:
    - trajectory: An (n x N) array, where n is the degrees of freedom (e.g., x, y, theta)
      and N is the number of time steps.
    - L: Length of the robot body.
    - l: Width of the robot body.
    - dt: Time step between each frame, in seconds.
    """
    # Setup figure and axis
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlim(-5, 5)  # Fixed x-axis limit
    ax.set_ylim(-5, 5)  # Fixed y-axis limit

    # Store initial limits for repeated use
    ax_limits = {'xlim': (-5, 5), 'ylim': (-5, 5)}

    # Animation update function
    def update(frame):
        # Clear and reset limits and properties
        ax.clear()
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlim(*ax_limits['xlim'])
        ax.set_ylim(*ax_limits['ylim'])
        
        # Get the current state from the trajectory matrix
        p = trajectory[:, frame]
        
        # Draw the robot at the current state
        dibujar_omnidireccional_4(ax, p, L, l)

    # Calculate interval in milliseconds for real-time animation
    interval_ms = int(dt)  # Convert from seconds to milliseconds

    # Create the animation
    ani = FuncAnimation(fig, update, frames=trajectory.shape[1], interval=interval_ms, repeat=False)
    plt.show()



def dibujar_omnidireccional_4(ax, p, L, l):
    x = p[0]
    y = p[1]
    theta = p[2]
    
    Lo = L * 0.3
    lo = L * 0.2
    Tob = np.array([[np.cos(theta), -np.sin(theta), x],
                    [np.sin(theta),  np.cos(theta), y],
                    [0,             0,             1]])
    
    Tblf = np.array([[1, 0, L],
                     [0, 1, l],
                     [0, 0, 1]])
    Tbrf = np.array([[1, 0, L],
                     [0, 1, -l],
                     [0, 0, 1]])
    Tblb = np.array([[1, 0, -L],
                     [0, 1, l],
                     [0, 0, 1]])
    Tbrb = np.array([[1, 0, -L],
                     [0, 1, -l],
                     [0, 0, 1]])
    
    Tolf = Tob @ Tblf
    Torf = Tob @ Tbrf
    Tolb = Tob @ Tblb
    Torb = Tob @ Tbrb
    
    # Base
    phi = np.linspace(0, 2 * np.pi, 50)
    pc = Tob @ np.array([L * 0.7, 0, 1])
    
    cx = pc[0] + L * 0.15 * np.cos(phi)
    cy = pc[1] + L * 0.15 * np.sin(phi)
    ax.plot(cx, cy, linewidth=2, markersize=10, color=[1, 0, 0])
    
    def plot_rectangle(T, Lo, lo, color):
        p1 = T @ np.array([+Lo, -lo, 1])
        p2 = T @ np.array([-Lo, -lo, 1])
        p3 = T @ np.array([+Lo, +lo, 1])
        p4 = T @ np.array([-Lo, +lo, 1])
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], linewidth=2, color=color)
        ax.plot([p1[0], p3[0]], [p1[1], p3[1]], linewidth=2, color=color)
        ax.plot([p2[0], p4[0]], [p2[1], p4[1]], linewidth=2, color=color)
        ax.plot([p3[0], p4[0]], [p3[1], p4[1]], linewidth=2, color=color)

    # Draw base frame
    plot_rectangle(Tob, L + Lo, l + lo, [0, 0, 1])
    
    # Draw wheels
    plot_rectangle(Tolf, Lo, lo, [0, 0, 1])
    plot_rectangle(Torf, Lo, lo, [0, 0, 1])
    plot_rectangle(Torb, Lo, lo, [0, 0, 1])
    plot_rectangle(Tolb, Lo, lo, [0, 0, 1])
