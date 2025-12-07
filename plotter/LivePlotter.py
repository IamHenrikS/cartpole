import matplotlib.pyplot as plt
import numpy as np

class LivePlotter:
    def __init__(self, title="CartPole Live Plot"):
        plt.ion()
        self.fig, (self.ax_tip, self.ax_angle) = plt.subplots(2, 1, figsize=(6, 8))
        self.fig.suptitle(title)

        # --- Tip trajectory plot ---
        self.ax_tip.set_xlabel("X (m)")
        self.ax_tip.set_ylabel("Y (m)")
        self.ax_tip.grid(True)
        self.tip_line, = self.ax_tip.plot([], [], 'r-', label="Pole Tip")
        self.tip_x, self.tip_y = [], []

        # --- Angle plot ---
        self.ax_angle.set_xlabel("Time step")
        self.ax_angle.set_ylabel("Angle (°)")
        self.ax_angle.grid(True)
        self.angle_line, = self.ax_angle.plot([], [], 'b-', label="Pole Angle")
        self.angle_y = []
        self.steps = []

        self.step_counter = 0

        self.ax_tip.legend()
        self.ax_angle.legend()

    def update(self, tip_pos, angle_rad):
        """Update both subplots.

        tip_pos: (x_tip, y_tip)
        angle_rad: theta in radians
        """
        # Convert angle to degrees and wrap to [-180, 180]
        angle_deg = np.degrees(angle_rad)
        angle_deg = ((angle_deg + 180) % 360) - 180  # wrap

        # Update tip trajectory
        x_tip, y_tip = tip_pos
        self.tip_x.append(x_tip)
        self.tip_y.append(y_tip)
        self.tip_line.set_xdata(self.tip_x)
        self.tip_line.set_ydata(self.tip_y)

        # Update angle plot
        self.step_counter += 1
        self.steps.append(self.step_counter)
        self.angle_y.append(angle_deg)
        self.angle_line.set_xdata(self.steps)
        self.angle_line.set_ydata(self.angle_y)

        # Rescale axes
        self.ax_tip.relim()
        self.ax_tip.autoscale_view()
        self.ax_angle.relim()
        self.ax_angle.autoscale_view()

        # Redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
