"""
Description: The file utilizes matplotlib for evaluation of the systems
generated in order to see the values.

# Ideas:
1. I want to loop through all the controllers and paramterize from the linearization.
2. I want to plot the 4 states: x, x_dot, theta, theta_dot.
3. Evaluate the time and learning precision.??
4. Implement noise into the systems and evaluate the stability margins of the cart-pole.
"""

import numpy as np
import matplotlib.pyplot as plt

class OfflinePlotter:
    def __init__(self, theta_ref=np.pi, x_ref=0.0, title="CartPole Tracking Errors"):
        self.theta_ref = theta_ref
        self.x_ref = x_ref

        self.theta_err = []
        self.x_err = []
        self.t = []
        self.states = []

        self.title = title

    def log(self, state, t):
        """
        Docstring for log
        
        :param self: self
        :param state: contains the state variables.
        :param t: contians the time stored. 
        """
        x, x_dot, theta, theta_dot = state

        # Theta error (wrapped)
        theta_err = self.theta_ref - theta
        theta_err = (theta_err + np.pi) % (2*np.pi) - np.pi
        theta_err_deg = np.degrees(theta_err)

        # Cart error
        x_err = self.x_ref - x

        self.theta_err.append(theta_err_deg)
        self.x_err.append(x_err)
        self.t.append(t)

        self.states.append(state)


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

    def StatePlotter(self):
        """
        Plot the 4 states in a 2x2 grid.
        """ 
        states_array = np.array(self.states)

        fig, axes = plt.subplots(2, 2, figsize=(10, 8), sharex=True)
        fig.suptitle("CIP States Over Time", fontsize=16, fontname='Garamond')
        
        # POSITION
        axes[0, 0].plot(self.t, states_array[:,0], 'b-', label='position (m)')
        axes[0, 0].set_ylabel('x (m)', fontsize=14, fontname='Garamond')
        axes[0, 0].grid(True)
        axes[0, 0].legend(fontsize=12)
        
        # VELOCITY
        axes[0, 1].plot(self.t, states_array[:,1], 'g-', label='velocity (m/s)')
        axes[0, 1].set_ylabel(r'$\dot{x}$ (m/s)', fontsize=14, fontname='Garamond')
        axes[0, 1].grid(True)
        axes[0, 1].legend(fontsize=12)

        # ANGLE
        axes[1, 0].plot(self.t, np.degrees(states_array[:,2]), 'r-', label='Angle (deg)')
        axes[1, 0].set_ylabel(r'$\theta$ (deg)', fontsize=14, fontname='Garamond')
        axes[1, 0].set_xlabel('Time (s)', fontsize=14, fontname='Garamond')
        axes[1, 0].grid(True)
        axes[1, 0].legend(fontsize=12)

        # ANGULAR VELOCITY
        axes[1, 1].plot(self.t, np.degrees(states_array[:,3]), 'm-', label='Angular velocity (deg/s)')
        axes[1, 1].set_ylabel(r'$\dot{\theta}$ (deg/s)', fontsize=14, fontname='Garamond')
        axes[1, 1].set_xlabel('Time (s)', fontsize=14, fontname='Garamond')
        axes[1, 1].grid(True)
        axes[1, 1].legend(fontsize=12)

        plt.tight_layout(rect=[0,0,1,0.96])
        plt.show()