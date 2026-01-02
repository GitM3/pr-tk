import numpy as np


class InvertedPendulumCart:
    def __init__(self, M, m, L, g=9.81, B_M=0.05, B_m=0.01):
        """
        M   : cart mass
        m   : pendulum mass
        L   : pendulum length
        g   : gravity
        B_M : cart damping
        B_m : pendulum damping
        """
        self.M = M
        self.m = m
        self.L = L
        self.g = g
        self.B_M = B_M
        self.B_m = B_m

    def dynamics(self, state, u):
        """
        Continuous-time dynamics

        state = [x, x_dot, theta, theta_dot]
        u     = force applied to cart
        """
        x, x_dot, theta, theta_dot = state

        M, m, L, g = self.M, self.m, self.L, self.g
        B_M, B_m = self.B_M, self.B_m

        s = np.sin(theta)
        c = np.cos(theta)
        denom = L * (M + m - m * c**2)

        x_ddot = (
            L * u
            + B_m * theta_dot * c
            - m * L * g * s * c
            + m * L**2 * theta_dot**2 * s
            - B_M * L * x_dot
        ) / denom

        theta_ddot = (
            -m * L * c * u
            - m**2 * L**2 * theta_dot**2 * s * c
            + B_M * x_dot * m * L * c
            - (M + m) * B_m * theta_dot
            + (M + m) * m * g * L * s
        ) / (m * L**2 * (M + m - m * c**2))

        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

    def tip_kinematics(self, state, x_ddot, theta_ddot):
        _, x_dot, theta, theta_dot = state
        L = self.L
        v_tip = np.array(
            [x_dot + L * np.cos(theta) * theta_dot, -L * np.sin(theta) * theta_dot]
        )
        a_tip = np.array(
            [
                x_ddot
                - L * np.sin(theta) * theta_dot**2
                + L * np.cos(theta) * theta_ddot,
                -L * np.cos(theta) * theta_dot**2 - L * np.sin(theta) * theta_ddot,
            ]
        )

        return v_tip, a_tip
