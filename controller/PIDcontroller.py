"""
Description: The current PID controller is a PD-controller.
It is possible to implement a I-part for the cart as the cart 
will be the only part of the system where the drift and intergrator is 
suitable.

Sources for improvement:
1. https://www.youtube.com/watch?v=hRnofMxEf3Q
2. Defined tuning method: Ziegler, or alike.

"""

import numpy as np

class PIDcontroller:
    def __init__(self, env):
        """
        Implements the env and initializes the PID controller. 
        """
        self.theta_ref = 0.0
        self.x_ref = 0.0
        
        # Parameters from (env, dynamics)
        g = env.g
        m_pole = env.m_p
        m_total = env.m_t
        l = env.l
        d_theta = env.mu_p
        b_x = env.mu_c

        alpha = 4.0/3.0 - m_pole/m_total

        # Lagrange derived solutions with inclusion of counteractive forces. 
        # Linearized around 0.
        self.A = np.array([[0, 1, 0, 0],
                           [0, -b_x/m_total, -m_pole*g/(m_total*alpha), -m_pole*d_theta/(m_total*alpha)],
                           [0, 0, 0, 1],
                           [0, 0, -g/(l*alpha), -d_theta/(l*alpha)]])
        self.B = np.array([
                            [0],
                            [(1/m_total) + (m_pole / (m_total**2 * alpha))],
                            [0],
                            [-(1/(m_total * l * alpha))]
                            ])
        # PID gains
        self.Kp = 30
        self.Kd = 0.1
        
        self.Kp_cart = 0.9
        self.Kd_cart = 0.5

        self.Ki = 0.0
        self.Ki_cart = 0.0

        # PD - Closed loop solution
        K_pid = np.array([[self.Kp_cart, self.Kd_cart, self.Kp, self.Kd]])
        A_cl = self.A - self.B @ K_pid
        poles = np.linalg.eigvals(A_cl)
        print("Closed-loop poles:", poles)

    def get_force(self, state, dt):
        x, x_dot, theta, theta_dot = state

        # Wrapped angle error
        theta_error = self.theta_ref - theta
        theta_error = (theta_error + np.pi) % (2*np.pi) - np.pi

        # Errors
        x_error = self.x_ref - x

        # PD control:
        # Pole stabilization
        F_pole = (
            self.Kp * theta_error
            + self.Kd * theta_dot
        )

        # Cart regulation
        F_cart = (
            self.Kp_cart * x_error
            + self.Kd_cart * x_dot
        )

        F = F_pole + F_cart

        return -F
