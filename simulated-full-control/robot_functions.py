import numpy as np
import matplotlib.pyplot as plt


def q_deseada(t):
    R = 1  # Radius coefficient
    k = 3  # Non-integer value for an open rose pattern

    theta = 0.1 * t
    r = R * np.sin(k * theta)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.array([x, y, 0])


def qp_deseada(t):
    R = 1  # Radius coefficient
    k = 3  # Non-integer value consistent with q_deseada

    theta = 0.1 * t
    r = R * np.sin(k * theta)
    dr_dt = R * k * np.cos(k * theta) * 0.1
    dx_dt = dr_dt * np.cos(theta) - r * np.sin(theta) * 0.1
    dy_dt = dr_dt * np.sin(theta) + r * np.cos(theta) * 0.1
    return np.array([dx_dt, dy_dt, 0])


# Define the robotMovement function before using it
def robot_movement(theta, v,L,l):
    alpha = theta + np.pi / 4
    return (1 / 4) * np.array([
        [np.sqrt(2) * np.sin(alpha), np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.cos(alpha),
         np.sqrt(2) * np.sin(alpha)],
        [-np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), np.sqrt(2) * np.sin(alpha),
         -np.sqrt(2) * np.cos(alpha)],
        [-1 / (L + l), 1 / (L + l), -1 / (L + l), 1 / (L + l)]
    ]) @ v

def plotting_xy(q_plot, q_desired):
    plt.figure()
    plt.plot(q_plot[0, :], q_plot[1, :], 'b', linewidth=2)
    plt.plot(q_desired[0, :], q_desired[1, :], "r--", linewidth=2)
    plt.plot(q_plot[0, 0], q_plot[1, 0], 'k+', markersize=10, linewidth=2)
    plt.xlabel('Posición X [m]')
    plt.ylabel('Posición Y [m]')
    plt.title('Trayectoria del robot')
    plt.grid()
    plt.axis('equal')
    plt.legend(["Real", "Deseada"])


def plot_desired_vs_actual(t_plot, q_plot, q_desired_array):
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(t_plot, q_plot[0, :], "b", linewidth=2)
    plt.plot(t_plot, q_desired_array[0, :], "r--", linewidth=2)
    plt.ylabel("m")
    plt.xlabel("Tiempo [s]")
    plt.title("Desplazamiento en X")
    plt.legend(["X real", "X deseada"])
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(t_plot, q_plot[1, :], "b", linewidth=2)
    plt.plot(t_plot, q_desired_array[1, :], "r--", linewidth=2)
    plt.ylabel("m")
    plt.xlabel("Tiempo [s]")
    plt.title("Desplazamiento en Y")
    plt.legend(["Y real", "Y deseada"])
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(t_plot, q_plot[2, :], "b", linewidth=2)
    plt.plot(t_plot, q_desired_array[2, :], "r--", linewidth=2)
    plt.ylabel("rad")
    plt.xlabel("Tiempo [s]")
    plt.title("Desplazamiento en theta")
    plt.legend(["θ real", "θ deseada"])
    plt.grid()


def plot_control_action(t_plot, control_plot):
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(t_plot, control_plot[0, :], linewidth=2)
    plt.title('Acción de control en $\dot{x}$', fontsize=10)
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(t_plot, control_plot[1, :], linewidth=2)
    plt.title('Acción de control en $\dot{y}$', fontsize=10)
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(t_plot, control_plot[2, :], linewidth=2)
    plt.title("Acción de control en $\dot{θ}$", fontsize=10)
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()


def plot_wheel_speeds(t_plot, v):
    plt.figure()
    plt.subplot(4, 1, 1)
    plt.plot(t_plot, v[0, :], linewidth=2)
    plt.title("Velocidad de rueda 1")
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.subplot(4, 1, 2)
    plt.plot(t_plot, v[1, :], linewidth=2)
    plt.title("Velocidad de rueda 2")
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.subplot(4, 1, 3)
    plt.plot(t_plot, v[2, :], linewidth=2)
    plt.title("Velocidad de rueda 3")
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.subplot(4, 1, 4)
    plt.plot(t_plot, v[3, :], linewidth=2)
    plt.title("Velocidad de rueda 4")
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()