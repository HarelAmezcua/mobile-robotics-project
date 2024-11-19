import numpy as np
import matplotlib.pyplot as plt

def plotting_visual_vs_odometry(visual_pose_plot, odometry_pose_plot):
    plt.figure()
    plt.plot(visual_pose_plot[0, :], visual_pose_plot[1, :], 'b', linewidth=2)
    plt.plot(odometry_pose_plot[0, :], odometry_pose_plot[1, :], "r--", linewidth=2)
    plt.plot(0, 0, 'k+', markersize=10, linewidth=2)
    plt.xlabel('Posición X [m]')
    plt.ylabel('Posición Y [m]')
    plt.title('Trayectoria del robot')
    plt.grid()
    plt.axis('equal')
    plt.xlim([-1, 1])  # Set x-axis limits
    plt.ylim([-1, 1])  # Set y-axis limits
    plt.legend(["Visual Pose", "Odometry Pose"])


def plot_against_time(t_plot, visual_pose_plot, odometry_pose_plot):
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(t_plot, visual_pose_plot[0, :], "b", linewidth=2)
    plt.plot(t_plot, odometry_pose_plot[0, :], "r--", linewidth=2)
    plt.ylabel("m")
    plt.xlabel("Tiempo [s]")
    plt.title("Desplazamiento en X")
    plt.legend(["X visual", "X Odometry"])
    plt.ylim([-0.3, 0.3])  # Set y-axis limits
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(t_plot, visual_pose_plot[1, :], "b", linewidth=2)
    plt.plot(t_plot, odometry_pose_plot[1, :], "r--", linewidth=2)
    plt.ylabel("m")
    plt.xlabel("Tiempo [s]")
    plt.title("Desplazamiento en Y")
    plt.legend(["Y visual", "Y Odometry"])
    plt.ylim([-0.3, 0.3])  # Set y-axis limits
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(t_plot, visual_pose_plot[2, :], "b", linewidth=2)
    plt.plot(t_plot, odometry_pose_plot[2, :], "r--", linewidth=2)
    plt.ylabel("m")
    plt.xlabel("Tiempo [s]")
    plt.title("Desplazamiento en theta")
    plt.legend(["Theta visual", "Theta Odometry"])
    plt.ylim([-1, 1])  # Set y-axis limits
    plt.grid()

def plot_dt(dt_plot,t_plot):
    plt.figure()
    plt.plot(t_plot, dt_plot, "b", linewidth=2)
    plt.ylabel("dt (s)")
    plt.xlabel("Time [s]")
    plt.title("Actual Time Steps")
    plt.grid()
    plt.tight_layout()
    plt.show()