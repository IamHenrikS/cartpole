import numpy as np
import matplotlib.pyplot as plt

class OfflinePlotter:
    def __init__(self, theta_ref=np.pi, x_ref=0.0, title="CartPole Tracking Errors"):
        self.theta_ref = theta_ref
        self.x_ref = x_ref

        self.theta_err = []
        self.x_err = []
        self.t = []

        self.title = title

    def log(self, state, t):
        """
        Docstring for log
        
        :param self: self
        :param state: contains the state variables.
        :param t: contians the time stored. 
        """
        x, _, theta, _ = state

        # Theta error (wrapped)
        theta_err = self.theta_ref - theta
        theta_err = (theta_err + np.pi) % (2*np.pi) - np.pi
        theta_err_deg = np.degrees(theta_err)

        # Cart error
        x_err = self.x_ref - x

        self.theta_err.append(theta_err_deg)
        self.x_err.append(x_err)
        self.t.append(t)

    def plot(self):
        """
        Docstring for plot
        
        # Setting 2 plots with the error of theta from theta_ref
        and from the x_ref to x 
        """
        fig, (ax_theta, ax_x) = plt.subplots(2, 1, figsize=(6, 6), sharex=True)
        fig.suptitle(self.title)

        ax_theta.plot(self.t, self.theta_err, 'r-', label="θ error")
        ax_theta.set_ylabel("Theta Error (deg)")
        ax_theta.set_ylim(-20, 20)
        ax_theta.invert_yaxis()
        ax_theta.grid(True)
        ax_theta.legend()

        ax_x.plot(self.t, self.x_err, 'b-', label="x error")
        ax_x.set_ylabel("x Error (m)")
        ax_x.set_xlabel("Time (s)")
        ax_x.grid(True)
        ax_x.legend()

        plt.show()
