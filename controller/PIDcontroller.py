import numpy as np

class PIDcontroller:
    def __init__(self, Kp=2, Kp_cart=0.05, Ki=0, Ki_cart=0.0, Kd=0.1, Kd_cart=0.2):
        # Angle PID
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Cart PID
        self.Kp_cart = Kp_cart
        self.Ki_cart = Ki_cart
        self.Kd_cart = Kd_cart

        # References
        self.theta_ref = np.pi
        self.x_ref = 0.0

        # Integrators
        self.integral_theta = 0.0
        self.integral_x = 0.0

    def get_force(self, state, dt):
        x, x_dot, theta, theta_dot = state

        # Wrapped angle error
        theta_error = self.theta_ref - theta
        theta_error = (theta_error + np.pi) % (2*np.pi) - np.pi

        # Errors
        x_error = self.x_ref - x

        # Integrate
        self.integral_theta += theta_error * dt
        self.integral_x += x_error * dt

        # PID control
        F = (
            self.Kp * theta_error
            + self.Ki * self.integral_theta
            + self.Kd * theta_dot
            + self.Kp_cart * x_error
            + self.Ki_cart * self.integral_x
            + self.Kd_cart * x_dot
        )

        return F
