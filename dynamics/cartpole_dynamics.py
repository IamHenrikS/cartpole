"""
The equations of motion of the cartpole.
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
        self.x = 0.0
        self.x_dot = 0.0
        self.theta = 0.05
        self.theta_dot = 0.0
    
    @property
    def state(self):
        return np.array([self.x, self.x_dot, self.theta, self.theta_dot], dtype=float)
    
    
    def step(self, force):
        g = self.g
        m_total = self.m_total
        m_cart = self.m_cart
        m_pole = self.m_pole
        l = self.l
        d_theta = self.d_theta
        b_x = self.b_x
        
        x, x_dot, theta, theta_dot = self.x, self.x_dot, self.theta, self.theta_dot
        
        costh = np.cos(theta)
        sinth = np.sin(theta)

        temp = (force + m_pole * l * theta_dot**2 * sinth) / m_total
        
        theta_acc = (g * sinth - costh * temp - d_theta * theta_dot) / (
            l * (4/3 - (m_pole * costh**2) / m_total)
        )
        
        x_acc = temp - (m_pole * l * theta_acc * costh) / (m_total - b_x * x_dot)

        # Integrate
        x += self.dt * x_dot
        x_dot += self.dt * x_acc
        theta += self.dt * theta_dot
        theta_dot += self.dt * theta_acc

        self.x, self.x_dot, self.theta, self.theta_dot = x, x_dot, theta, theta_dot

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
                


