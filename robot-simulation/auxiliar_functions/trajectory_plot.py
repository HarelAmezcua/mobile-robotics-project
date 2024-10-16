# trajectory_plot.py

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation
import numpy as np
from matplotlib.widgets import Button

def create_trajectory_plot(x, y):
    """Create and display a static plot of the robot trajectory."""
    fig, ax = plt.subplots()
    ax.set_xlabel('x position (m)')
    ax.set_ylabel('y position (m)')
    ax.set_title('Omnidirectional Robot Trajectory')
    ax.axis('equal')
    ax.grid(True)
    
    # Plot the trajectory
    ax.plot(x, y, label='Trajectory', color='red', linestyle='--')
    ax.legend()
    
    return fig, ax

def create_robot_patch(robot_length, robot_width):
    """Create the initial robot polygon patch."""
    robot_shape = np.array([
        [robot_length / 2, 0],                         # Front point
        [-robot_length / 2, robot_width / 2],          # Rear left
        [-robot_length / 2, -robot_width / 2]          # Rear right
    ])
    robot_patch = Polygon(robot_shape, closed=True, color='blue')
    return robot_patch, robot_shape

def create_animation_plot(x, y, robot_length, robot_width):
    """Create and display the plot for the animated robot movement."""
    fig, ax = plt.subplots()
    ax.set_xlabel('x position (m)')
    ax.set_ylabel('y position (m)')
    ax.set_title('Omnidirectional Robot Movement')
    ax.axis('equal')
    ax.grid(True)
    
    # Set axis limits
    ax.set_xlim(np.min(x) - 1, np.max(x) + 1)
    ax.set_ylim(np.min(y) - 1, np.max(y) + 1)
    
    # Plot the dashed trajectory line
    ax.plot(x, y, 'r--', label='Trajectory')
    
    # Create the robot patch and add it to the axis
    robot_patch, robot_shape = create_robot_patch(robot_length, robot_width)
    ax.add_patch(robot_patch)
    
    return fig, ax, robot_patch, robot_shape

def animate_robot_movement(i, x, y, theta, t, robot_patch, robot_shape, ax, is_paused):
    """Animate the robot's movement based on its trajectory."""
    if is_paused:
        return robot_patch,
    
    xi = x[i]
    yi = y[i]
    thetai = theta[i]
    
    # Compute the rotation matrix
    R = np.array([
        [np.cos(thetai), -np.sin(thetai)],
        [np.sin(thetai), np.cos(thetai)]
    ])
    
    # Rotate and translate the robot shape
    transformed_shape = (R @ robot_shape.T).T + np.array([xi, yi])
    
    # Update the robot patch
    robot_patch.set_xy(transformed_shape)
    
    # Ensure t[i] is displaying the right time
    if i < len(t):
        ax.set_title(f'Omnidirectional Robot Movement (t = {t[i]:.2f}s)')
    else:
        ax.set_title('Omnidirectional Robot Movement')

    return robot_patch,


def setup_interaction_buttons(fig, toggle_pause, stop_animation, reset_animation):
    """Create Play/Pause, Stop, and Reset buttons for interaction."""
    button_ax_play = plt.axes([0.7, 0.025, 0.1, 0.04])  # [left, bottom, width, height]
    button_ax_stop = plt.axes([0.81, 0.025, 0.1, 0.04])
    button_ax_reset = plt.axes([0.59, 0.025, 0.1, 0.04])
    
    # Create the buttons
    button_play = Button(button_ax_play, 'Play/Pause')
    button_stop = Button(button_ax_stop, 'Stop')
    button_reset = Button(button_ax_reset, 'Reset')
    
    # Assign the functions to the buttons
    button_play.on_clicked(toggle_pause)
    button_stop.on_clicked(stop_animation)
    button_reset.on_clicked(reset_animation)

def toggle_pause(event, state):
    """Toggle the play/pause state."""
    state['is_paused'] = not state['is_paused']

def stop_animation(event, ani):
    """Stop the animation."""
    ani.event_source.stop()

def reset_animation(event, ani, state):
    """Reset the animation to the beginning."""
    ani.event_source.start()
    state['is_paused'] = False

def run_robot_animation(x, y, theta, t, dt, robot_length=0.2, robot_width=0.1):
    """Main function to set up and run the robot animation."""
    # Create trajectory plot (static)
    create_trajectory_plot(x, y)
    
    # Create animation plot
    fig, ax, robot_patch, robot_shape = create_animation_plot(x, y, robot_length, robot_width)
    
    # Control variable to track play/pause state
    state = {'is_paused': False}
    
    # Animation function with current state of pause
    def animate_func(i):
        return animate_robot_movement(i, x, y, theta, t, robot_patch, robot_shape, ax, state['is_paused'])
    
    # Create the animation
    ani = FuncAnimation(fig, animate_func, frames=len(t), interval=dt*1000, blit=True)
    
    # Set up interaction buttons, adding reset functionality
    setup_interaction_buttons(fig, lambda event: toggle_pause(event, state), 
                              lambda event: stop_animation(event, ani), 
                              lambda event: reset_animation(event, ani, state))
    
    # Show the figures
    plt.show()
