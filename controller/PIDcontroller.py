import matplotlib.pyplot as plt
import numpy as np

class PIDcontroller:
    def __init__(self, Kp=2, Kp_cart=0.05, Ki = 0, Ki_cart = 0.0, Kd=0.1, Kd_cart=0.2):
        # Angle PID
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        # Cart PID
        self.Kp_cart = Kp_cart
        self.Ki_cart = Ki_cart
        self.Kd_cart = Kd_cart

        # Other
        self.theta_ref = np.pi  # Upright
        self.error_history = []
        self.integral = 0.0
        self.integral_cart = 0.0
        self.max_points = 500

        # --- Initialize live plot ---
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("PID Theta Error (deg)")
        self.ax.set_xlabel("Time step")
        self.ax.set_ylabel("Theta Error (deg)")
        self.ax.grid(True)
        self.ax.set_ylim(-20, 20)  # fixed Y-axis
        self.ax.invert_yaxis()       # inverted Y-axis
        self.line, = self.ax.plot([], [], 'r-')

    def get_force(self, state):
        """
        Docstring for get_force
        
        :param self: Initial values of the system
        :param state: Contains the state variables used for the control system
        """        
        x, x_dot, theta, theta_dot = state

        # Compute normalized error in degrees
        theta_error = self.theta_ref - theta
        theta_error = (theta_error + np.pi) % (2*np.pi) - np.pi
        theta_error_deg = np.rad2deg(theta_error)

        # The I-term of the PID (lowered the dt for finer and less numerical error)
        dt = 0.001
        self.integral += theta_error*dt
        self.integral_cart += x*dt
        
        
        # PID control (Angle PID) + (position PID)
        F = (self.Kp * theta_error + self.Kd * theta_dot) \
        - (self.Kp_cart*x + self.Kd_cart*x_dot + self.Ki_cart*self.integral_cart)

        # --- Update live plot ---
        self.error_history.append(theta_error_deg)
        if len(self.error_history) > self.max_points:
            self.error_history.pop(0)

        self.line.set_data(np.arange(len(self.error_history)), self.error_history)
        self.ax.set_xlim(0, self.max_points)  # fixed X-axis window

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        return F
