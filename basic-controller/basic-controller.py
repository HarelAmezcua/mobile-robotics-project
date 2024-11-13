import numpy as np
import matplotlib.pyplot as plt

# Robot parameters
L = 0.25
l = 0.20

# Simulation parameters
dt = 0.01
S = 60
N = int(S / dt)

# Pre-allocate storage matrices
q_plot = np.zeros((3, N))
qp_plot = np.zeros((3, N))
t_plot = np.arange(0, N) * dt  # Fixed the size of tPlot
control_plot = np.zeros((3, N))
wheel_speed_plot = np.zeros((4, N))
q_desired_plot = np.zeros((3, N))
qp_desired_plot = np.zeros((3, N))

# Polar rose parameters
R = 5  # Radius coefficient
k = 10  # Number of petals (integer values produce closed roses)


def q_deseada(t):
    theta = 0.1 * t
    r = R * np.sin(k * theta)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.array([x, y, 0])  # theta_desired = 0


def qp_deseada(t):
    theta = 0.1 * t
    r = R * np.sin(k * theta)
    dr_dt = R * k * np.cos(k * theta) * 0.1
    dx_dt = dr_dt * np.cos(theta) - r * np.sin(theta) * 0.1
    dy_dt = dr_dt * np.sin(theta) + r * np.cos(theta) * 0.1
    return np.array([dx_dt, dy_dt, 0])


# Define the robotMovement function before using it
def robot_movement(theta, v):
    alpha = theta + np.pi / 4
    return (1 / 4) * np.array([
        [np.sqrt(2) * np.sin(alpha), np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.cos(alpha),
         np.sqrt(2) * np.sin(alpha)],
        [-np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), np.sqrt(2) * np.sin(alpha),
         -np.sqrt(2) * np.cos(alpha)],
        [-1 / (L + l), 1 / (L + l), -1 / (L + l), 1 / (L + l)]
    ]) @ v


# Initial condition
q = np.array([-5, 5, np.pi / 4])
qp = np.zeros(3)
q_plot[:, 0] = q
qp_plot[:, 0] = qp

# Gains
k_x, k_y, k_theta = 1, 1, 1
gain_matrix = np.diag([k_x, k_y, k_theta])

# Simulation
i = 1
for t in np.arange(dt, S, dt):
    q_desired_plot[:, i] = q_deseada(t)
    qp_desired_plot[:, i] = qp_deseada(t)

    # Controller (First part - [u_x, u_y, u_theta])
    q_actual = q
    u = qp_deseada(t) + gain_matrix @ (q_deseada(t) - q_actual)

    # Convert to wheel speeds
    alpha = q_actual[2] + np.pi / 4
    v = np.array([
        [np.sqrt(2) * np.sin(alpha), -np.sqrt(2) * np.cos(alpha), -(L + l)],
        [np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), (L + l)],
        [np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), -(L + l)],
        [np.sqrt(2) * np.sin(alpha), -np.sqrt(2) * np.cos(alpha), (L + l)]
    ]) @ u

    qp = robot_movement(q_actual[2], v)
    q = q + qp * dt

    q_plot[:, i] = q
    qp_plot[:, i] = qp
    control_plot[:, i] = u
    wheel_speed_plot[:, i] = v

    i += 1


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


# Call plotting functions
plotting_xy(q_plot, q_desired_plot)
plot_desired_vs_actual(t_plot, q_plot, q_desired_plot)
plot_control_action(t_plot, control_plot)
plot_wheel_speeds(t_plot, wheel_speed_plot)

# Add plt.show() at the end of the script
plt.show()
