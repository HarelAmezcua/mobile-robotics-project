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


# Trajectory generation
a = 0.1
b = 2
c = 0.1

def qDeseada(t):
    return np.array([a * t, b * np.sin(c * t), 0])

def qpDeseada(t):
    return np.array([a, b * c * np.cos(c * t), 0])

# Define the robotMovement function before using it
def robotMovement(theta, v):
    alpha = theta + np.pi / 4
    return (1 / 4) * np.array([
        [np.sqrt(2) * np.sin(alpha), np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha)],
        [-np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), np.sqrt(2) * np.sin(alpha), -np.sqrt(2) * np.cos(alpha)],
        [-1 / (L + l), 1 / (L + l), -1 / (L + l), 1 / (L + l)]
    ]) @ v

# Initial condition
q = np.array([-5, 5, np.pi / 4])
qp = np.zeros(3)
q_plot[:, 0] = q
qp_plot[:, 0] = qp

# Gains
kx, ky, ktheta = 1, 1, 1
matrizGanancia = np.diag([kx, ky, ktheta])

# Simulation
i = 1
for t in np.arange(dt, S, dt):
    q_desired_plot[:, i] = qDeseada(t)
    qp_desired_plot[:, i] = qpDeseada(t)
    
    # Controller (First part - [u_x, u_y, u_theta])
    qActual = q
    u = qpDeseada(t) + matrizGanancia @ (qDeseada(t) - qActual)

    # Convert to wheel speeds
    alpha = qActual[2] + np.pi / 4
    v = np.array([
        [np.sqrt(2) * np.sin(alpha), -np.sqrt(2) * np.cos(alpha), -(L + l)],
        [np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), (L + l)],
        [np.sqrt(2) * np.cos(alpha), np.sqrt(2) * np.sin(alpha), -(L + l)],
        [np.sqrt(2) * np.sin(alpha), -np.sqrt(2) * np.cos(alpha), (L + l)]
    ]) @ u

    qp = robotMovement(qActual[2], v)
    q = q + qp * dt

    q_plot[:, i] = q
    qp_plot[:, i] = qp
    control_plot[:, i] = u
    wheel_speed_plot[:, i] = v

    i += 1



# Remove plt.show() from each plotting function

def plottingXY(qPlot, qDeseada):
    plt.figure()
    plt.plot(qPlot[0, :], qPlot[1, :], 'b', linewidth=2)
    plt.plot(qDeseada[0, :], qDeseada[1, :], "r--", linewidth=2)
    plt.plot(qPlot[0, 0], qPlot[1, 0], 'k+', markersize=10, linewidth=2)
    plt.xlabel('Posición X [m]')
    plt.ylabel('Posición Y [m]')
    plt.title('Trayectoria del robot')
    plt.grid()
    plt.axis('equal')
    plt.legend(["Real", "Deseada"])
    # Removed plt.show()

def plottingDesiredvsActual(tPlot, qPlot, qDeseadaPlot):
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(tPlot, qPlot[0, :], "b", linewidth=2)
    plt.plot(tPlot, qDeseadaPlot[0, :], "r--", linewidth=2)
    plt.ylabel("m")
    plt.xlabel("Tiempo [s]")
    plt.title("Desplazamiento en X")
    plt.legend(["X real", "X deseada"])
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(tPlot, qPlot[1, :], "b", linewidth=2)
    plt.plot(tPlot, qDeseadaPlot[1, :], "r--", linewidth=2)
    plt.ylabel("m")
    plt.xlabel("Tiempo [s]")
    plt.title("Desplazamiento en Y")
    plt.legend(["Y real", "Y deseada"])
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(tPlot, qPlot[2, :], "b", linewidth=2)
    plt.plot(tPlot, qDeseadaPlot[2, :], "r--", linewidth=2)
    plt.ylabel("rad")
    plt.xlabel("Tiempo [s]")
    plt.title("Desplazamiento en theta")
    plt.legend(["θ real", "θ deseada"])
    plt.grid()
    # Removed plt.show()

def plottingControlAction(tPlot, controlPlot):
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(tPlot, controlPlot[0, :], linewidth=2)
    plt.title('Acción de control en $\dot{x}$', fontsize=10)
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(tPlot, controlPlot[1, :], linewidth=2)
    plt.title('Acción de control en $\dot{y}$', fontsize=10)
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(tPlot, controlPlot[2, :], linewidth=2)
    plt.title("Acción de control en $\dot{θ}$", fontsize=10)
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()
    # Removed plt.show()

def plottingWheelSpeeds(tPlot, v):
    plt.figure()
    plt.subplot(4, 1, 1)
    plt.plot(tPlot, v[0, :], linewidth=2)
    plt.title("Velocidad de rueda 1")
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.subplot(4, 1, 2)
    plt.plot(tPlot, v[1, :], linewidth=2)
    plt.title("Velocidad de rueda 2")
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.subplot(4, 1, 3)
    plt.plot(tPlot, v[2, :], linewidth=2)
    plt.title("Velocidad de rueda 3")
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()

    plt.subplot(4, 1, 4)
    plt.plot(tPlot, v[3, :], linewidth=2)
    plt.title("Velocidad de rueda 4")
    plt.ylabel("m/s")
    plt.xlabel("Tiempo [s]")
    plt.grid()
    # Removed plt.show()

# Call plotting functions
plottingXY(q_plot, q_desired_plot)
plottingDesiredvsActual(t_plot, q_plot, q_desired_plot)
plottingControlAction(t_plot, control_plot)
plottingWheelSpeeds(t_plot, wheel_speed_plot)

# Add plt.show() at the end of the script
plt.show()
