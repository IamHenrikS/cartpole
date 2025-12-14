import matplotlib.pyplot as plt
import numpy as np

class LivePlotter:
    def __init__(self, theta_ref=np.pi, x_ref=0.0, title="CartPole Errors"):
        plt.ion()

        self.theta_ref = theta_ref
        self.x_ref = x_ref

        self.fig, (self.ax_theta, self.ax_x) = plt.subplots(2, 1, figsize=(6, 6), sharex=True)
        self.fig.suptitle(title)

        # --- Theta error plot ---
        self.ax_theta.set_ylabel("Theta Error (deg)")
        self.ax_theta.grid(True)
        self.ax_theta.set_ylim(-20, 20)
        self.ax_theta.invert_yaxis()
        self.theta_line, = self.ax_theta.plot([], [], 'r-', label="θ error")

        # --- Cart position error plot ---
        self.ax_x.set_xlabel("Time step")
        self.ax_x.set_ylabel("x Error (m)")
        self.ax_x.grid(True)
        self.x_line, = self.ax_x.plot([], [], 'b-', label="x error")

        self.theta_err = []
        self.x_err = []
        self.steps = []
        self.step_counter = 0

        self.ax_theta.legend()
        self.ax_x.legend()

    def update(self, state):
        """
        state: [x, x_dot, theta, theta_dot]
        """
        x, _, theta, _ = state

        # Theta error (wrapped)
        theta_err = self.theta_ref - theta
        theta_err = (theta_err + np.pi) % (2*np.pi) - np.pi
        theta_err_deg = np.degrees(theta_err)

        # Cart error
        x_err = self.x_ref - x

        # Store
        self.step_counter += 1
        self.steps.append(self.step_counter)
        self.theta_err.append(theta_err_deg)
        self.x_err.append(x_err)

        # Update plots
        self.theta_line.set_data(self.steps, self.theta_err)
        self.x_line.set_data(self.steps, self.x_err)

        self.ax_theta.set_xlim(0, max(100, self.step_counter))
        self.ax_x.set_xlim(0, max(100, self.step_counter))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
