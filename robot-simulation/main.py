import numpy as np
from auxiliar_functions.trajectory_plot import run_robot_animation

# Example data to run the animation (replace with actual robot data)
x = np.linspace(0, 10, 101)
y = np.sin(x)
theta = np.linspace(0, 2*np.pi, 101)
t = np.linspace(0, 10, 100)
dt = 0.1

print (x)

# Call the function to run the animation
run_robot_animation(x, y, theta, t, dt)
