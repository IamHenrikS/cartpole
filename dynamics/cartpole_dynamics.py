"""
The equations of motion of the cartpole.
Based on the followning derivation of the dynamics:
Source 1: Cart-Pole Optimal Control
https://openmdao.github.io/dymos/examples/cart_pole/cart_pole.html
Source 2: Cart-Pole System: Equations of motion.
https://courses.ece.ucsb.edu/ECE594/594D_W10Byl/hw/cartpole_eom.pdf 
"""
import numpy as np

class CartPoleDynamics:
    def __init__(self, dt=0.02):
        # Parameters:
        self.g = 9.82
        self.m_cart = 1.0
        self.m_pole = 0.1
        self.m_total = self.m_cart + self.m_pole
        self.l = 0.5
        self.dt = dt

        # Dampning of the system
        self.d_theta = 0.6
        self.b_x = 0.1

        # State
        self.reset()

    def reset(self):
        """
        Docstring for reset
        
        :param self: self.theta is the angle of the
        pole when it is hanging down vertically. The upright position 
        would thus be pi (180 deg)
        """
        self.x = 0.0
        self.x_dot = 0.0
        self.theta = np.deg2rad(175)
        self.theta_dot = 0.0
    
    @property
    def state(self):
        return np.array([self.x, self.x_dot, self.theta, self.theta_dot], dtype=float)
    
    
    def step(self, force):
        g = self.g
        m_p = self.m_pole
        m_t = self.m_total
        l = self.l
        d_theta = self.d_theta
        b_x = self.b_x
        dt = self.dt

        x, x_dot, theta, theta_dot = self.x, self.x_dot, self.theta, self.theta_dot

        sin_t = np.sin(theta)
        cos_t = np.cos(theta)

        # Angular acceleration (explicit form)
        theta_acc = (
            g * sin_t
            - cos_t * (force + m_p * l * theta_dot**2 * sin_t) / m_t
            - d_theta * theta_dot
        ) / (
            l * (4.0 / 3.0 - (m_p * cos_t**2) / m_t)
        )

        # Horizontal acceleration (explicit form)
        x_acc = (
            (force + m_p * l * theta_dot**2 * sin_t) / m_t
            - (m_p * l * theta_acc * cos_t) / (m_t - b_x * x_dot)
        )

        # Integrate
        x += dt * x_dot
        x_dot += dt * x_acc
        theta += dt * theta_dot
        theta_dot += dt * theta_acc

        self.x = x
        self.x_dot = x_dot
        self.theta = theta
        self.theta_dot = theta_dot


    def poleposition(self):
        """
        Docstring for poleposition
        
        :param self: Return the position (x_pole, y_pole)
        for the pole to be used in plotting.
        """
        L = self.l*2
        x_pole = self.x + L * np.sin(self.theta)
        y_pole = -L * np.cos(self.theta)
        return x_pole, y_pole
    
    def angleposition(self):
        """Return a point representing the angle from vertical.
        Y > 0 for upwards, X displacement proportional to sin(theta) at fixed radius.
        """
        radius = 1.0  # arbitrary length for visualization
        x_angle = self.x + radius * np.sin(self.theta)
        y_angle = radius * np.cos(self.theta)
        return x_angle, y_angle
                


