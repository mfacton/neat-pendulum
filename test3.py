import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Constants
g = 9.81  # Acceleration due to gravity (m/s^2)
L1 = 1.0  # Length of the first pendulum (m)
L2 = 1.0  # Length of the second pendulum (m)
m1 = 1.0  # Mass of the first pendulum (kg)
m2 = 1.0  # Mass of the second pendulum (kg)
dt = 0.05  # Time step (s)
t_max = 20.0  # Maximum simulation time (s)


# Function to compute the derivatives
def derivatives(state):
    theta1, theta1_dot, theta2, theta2_dot, pivot_x, pivot_y, pivot_vx, pivot_vy = state

    # Derivatives of pendulum angles
    theta1_ddot = (
        -g * (2 * m1 + m2) * np.sin(theta1)
        - m2 * g * np.sin(theta1 - 2 * theta2)
        - 2
        * np.sin(theta1 - theta2)
        * m2
        * (theta2_dot**2 * L2 + theta1_dot**2 * L1 * np.cos(theta1 - theta2))
    ) / (L1 * (2 * m1 + m2 - m2 * np.cos(2 * theta1 - 2 * theta2)))
    theta2_ddot = (
        2
        * np.sin(theta1 - theta2)
        * (
            theta1_dot**2 * L1 * (m1 + m2)
            + g * (m1 + m2) * np.cos(theta1)
            + theta2_dot**2 * L2 * m2 * np.cos(theta1 - theta2)
        )
    ) / (L2 * (2 * m1 + m2 - m2 * np.cos(2 * theta1 - 2 * theta2)))

    # Derivatives of pivot point position
    pivot_ax = 0  # Acceleration in x-direction (assumed zero for simplicity)
    pivot_ay = 0  # Acceleration in y-direction (assumed zero for simplicity)

    return [
        theta1_dot,
        theta1_ddot,
        theta2_dot,
        theta2_ddot,
        pivot_vx,
        pivot_vy,
        pivot_ax,
        pivot_ay,
    ]


# Function to perform RK4 integration
def rk4(state, derivatives, dt):
    k1 = np.array(derivatives(state))
    k2 = np.array(derivatives(state + 0.5 * dt * k1))
    k3 = np.array(derivatives(state + 0.5 * dt * k2))
    k4 = np.array(derivatives(state + dt * k3))
    state_next = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return state_next


# Initial conditions (theta1, theta1_dot, theta2, theta2_dot, pivot_x, pivot_y, pivot_vx, pivot_vy)
initial_state = [np.pi / 2, 0, np.pi / 2, 0, 0, 0, 0.1, 0.1]

# Lists to store the trajectory
trajectory = []

# Perform simulation
t = 0.0
state = initial_state
while t < t_max:
    # Store the current state
    trajectory.append(state)

    # Perform RK4 integration
    state = rk4(state, derivatives, dt)

    # Update pivot point position
    state[4] += state[6] * dt  # Update pivot_x
    state[5] += state[7] * dt  # Update pivot_y

    # Increment time
    t += dt

# Convert trajectory to numpy array for easier indexing
trajectory = np.array(trajectory)

# Extract the angles
theta1_vals = trajectory[:, 0]
theta2_vals = trajectory[:, 2]

# Convert angles to Cartesian coordinates
x1 = L1 * np.sin(theta1_vals)
y1 = -L1 * np.cos(theta1_vals)
x2 = x1 + L2 * np.sin(theta2_vals)
y2 = y1 - L2 * np.cos(theta2_vals)

# Plot the double pendulum
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
(line,) = ax.plot([], [], "o-", lw=2)


# Function to update the animation
def update(frame):
    pivot_x, pivot_y = trajectory[frame, 4:6]
    line.set_data([pivot_x, x1[frame], x2[frame]], [pivot_y, y1[frame], y2[frame]])
    return (line,)


# Create the animation
ani = FuncAnimation(fig, update, frames=len(trajectory), blit=True, interval=dt * 1000)
plt.show()
