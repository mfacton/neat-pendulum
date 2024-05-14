import math

import cv2
import numpy as np


class Simulation:
    """Simulation that runs double pendulum"""

    def __init__(self) -> None:
        self.screen_size = 600
        self.ball_size = 10

        self.r1 = 125
        self.r2 = 125
        self.m1 = 10
        self.m2 = 10
        self.theta1 = math.pi / 2
        self.theta2 = math.pi / 2
        self.vtheta1 = 0
        self.vtheta2 = 0
        self.g = 1

    def step(self, dtime):
        msum = self.m1 + self.m2
        thetadiff = self.theta1 - self.theta2
        sindif = math.sin(thetadiff)
        cosdif = math.cos(thetadiff)

        denominator = self.r1 * (self.m1 + msum - self.m2 * math.cos(2 * thetadiff))

        atheta1 = (
            -self.g * (msum + self.m1) * math.sin(self.theta1)
            - self.m2 * self.g * math.sin(thetadiff - self.theta2)
            - 2
            * sindif
            * self.m2
            * (
                self.vtheta2 * self.vtheta2 * self.r2
                + self.vtheta1 * self.vtheta1 * self.r1 * cosdif
            )
        ) / denominator
        atheta2 = (
            2
            * sindif
            * (
                self.vtheta1 * self.vtheta1 * self.r1 * msum
                + self.g * msum * math.cos(self.theta1)
                + self.vtheta2 * self.vtheta2 * self.r2 * self.m2 * cosdif
            )
        ) / denominator

        self.vtheta1 += atheta1 * dtime
        self.vtheta2 += atheta2 * dtime
        self.theta1 += self.vtheta1 * dtime
        self.theta2 += self.vtheta2 * dtime

    def draw(self):
        img = np.zeros((self.screen_size, self.screen_size, 3), dtype=np.uint8)

        centerx = self.screen_size / 2
        centery = self.screen_size / 2

        x1 = centerx + self.r1 * math.sin(self.theta1)
        y1 = centery + self.r1 * math.cos(self.theta1)

        x2 = x1 + self.r2 * math.sin(self.theta2)
        y2 = y1 + self.r2 * math.cos(self.theta2)

        cv2.line(
            img, (int(centerx), int(centery)), (int(x1), int(y1)), (255, 255, 255), 1
        )
        cv2.circle(img, (int(x1), int(y1)), self.ball_size, (255, 255, 255), -1)
        cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 255), 1)
        cv2.circle(img, (int(x2), int(y2)), self.ball_size, (255, 255, 255), -1)

        cv2.imshow("Simulation", img)
        cv2.waitKey(1)
