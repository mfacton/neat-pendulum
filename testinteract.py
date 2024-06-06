import math
import time

import cv2
import numpy as np

x = 256
y = 256
# Global variables to store previous mouse position and time
mouse_x = None
mouse_y = None
prev_x = None
prev_y = None
prev_velocity_x = None
prev_velocity_y = None

acceleration_x = 0
acceleration_y = 0

g = 9.81  # Acceleration due to gravity (m/s^2)
L1 = 100.0  # Length of the first pendulum (m)
L2 = 100.0  # Length of the second pendulum (m)
m1 = 100.0  # Mass of the first pendulum (kg)
m2 = 100.0  # Mass of the second pendulum (kg)

state = [math.pi / 2, 0, math.pi / 2, 0]

img = np.zeros((512, 512, 3), np.uint8)


def derivatives(state, accel):
    theta1, theta1_dot, theta2, theta2_dot = state

    # Derivatives of pendulum angles
    theta1_ddot = (
        -accel * (2 * m1 + m2) * np.sin(theta1)
        - m2 * accel * np.sin(theta1 - 2 * theta2)
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
            + accel * (m1 + m2) * np.cos(theta1)
            + theta2_dot**2 * L2 * m2 * np.cos(theta1 - theta2)
        )
    ) / (L2 * (2 * m1 + m2 - m2 * np.cos(2 * theta1 - 2 * theta2)))

    return [
        theta1_dot,
        theta1_ddot,
        theta2_dot,
        theta2_ddot,
    ]


def rk4(state, derivatives, accel, dt):
    k1 = np.array(derivatives(state, accel))
    k2 = np.array(derivatives(state + 0.5 * dt * k1, accel))
    k3 = np.array(derivatives(state + 0.5 * dt * k2, accel))
    k4 = np.array(derivatives(state + dt * k3, accel))
    state_next = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return state_next


# Callback function to capture mouse events
def mouse_callback(event, mx, my, flags, param):
    global mouse_x, mouse_y

    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x = mx
        mouse_y = my


cv2.imshow("Double Pendulum", img)
cv2.setMouseCallback("Double Pendulum", mouse_callback)

last_time = time.time() * 8
time.sleep(0.001)

draw = 0
while cv2.waitKey(1):
    img = np.zeros((512, 512, 3), np.uint8)
    # draw state
    x1 = x + L1 * math.sin(state[0])
    y1 = y + L1 * math.cos(state[0])

    x2 = x1 + L2 * math.sin(state[2])
    y2 = y1 + L2 * math.cos(state[2])

    cv2.circle(img, (int(x), int(y)), 5, (255, 255, 255), -1)
    cv2.line(img, (int(x), int(y)), (int(x1), int(y1)), (255, 255, 255), 1)
    cv2.circle(img, (int(x1), int(y1)), 5, (255, 255, 255), -1)
    cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 255), 1)
    cv2.circle(img, (int(x2), int(y2)), 5, (255, 255, 255), -1)

    cv2.imshow("Double Pendulum", img)

    current_time = time.time() * 8
    delta_time = current_time - last_time
    last_time = current_time

    # ax = math.sin(current_time) * 50
    # ay = math.cos(current_time) * 50 + g
    ax = 0
    ay = g

    x = 256  # + 50 * math.sin(current_time)
    y = 256  # + 50 * math.cos(current_time)
    twist = math.pi / 2 - math.atan2(ay, ax)

    # state[0] -= twist
    # state[2] -= twist

    # Perform RK4 integration
    state = rk4(state, derivatives, math.sqrt(ax * ax + ay * ay), delta_time)

    # state[0] += twist
    # state[2] += twist

    # calculate mouse accel
    if prev_x is not None:
        delta_x = mouse_x - prev_x
        delta_y = mouse_y - prev_y
        if abs(delta_x) > 1:
            velocity_x = delta_x / delta_time
            if prev_velocity_x is not None:
                acceleration_x = (velocity_x - prev_velocity_x) / delta_time
                prev_velocity_x = velocity_x
            prev_velocity_x = velocity_x
        else:
            prev_velocity_x = 0
            acceleration_x = 0

        if abs(delta_y) > 1:
            velocity_y = delta_y / delta_time
            if prev_velocity_y is not None:
                acceleration_y = (velocity_y - prev_velocity_y) / delta_time
                prev_velocity_y = velocity_y
            prev_velocity_y = velocity_y
        else:
            prev_velocity_y = 0
            acceleration_y = 0

    prev_x = mouse_x
    prev_y = mouse_y

cv2.destroyAllWindows()
