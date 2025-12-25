"""
Main file for the CartPole project.

Modules and Components:

1. Controllers:
    - PIDController: Standard PID controller for stabilizing the cart-pole.
    - LQRController: Linear Quadratic Regulator for optimal control.
    - NonLinearLQR: Non-linear extension of LQR for improved performance.
    - MLController: Machine learning-based controller (optional).

2. Dynamics:
    - cartpole_dynamics(): Simulates the dynamics of the cart and pole system.

3. Plotting:
    - LivePlotter(): Real-time plotting of system errors and states. Used in conjunction with Pygame for visualization.
    - OfflinePlotter(): Post-simulation plotting, optimized for analytics and batch simulations.

4. Rendering:
    - PygameRenderer(): Enables visualization using Pygame, showing the cart, pole, and applied forces.

5. User Interface:
    - UIManager(): Handles user interface elements and interactions.

Operation Modes:

1. Online Mode:
    - Activates PygameRenderer and LivePlotter.
    - Runs the simulation in real-time until the window is closed.

2. Offline Mode:
    - Uses OfflinePlotter for faster simulation.
    - Stores simulation results for analysis.
    - Requires specifying a simulation time `SIM_TIME`.

Additional Notes:
- Verify the implementations and update the ULM.dio with how the code is structured.
- The system uses getters and setters for cleaner logic and easier access to environment parameters.
- The modular design allows swapping controllers, plotting methods, and renderers as needed.
"""

import sys
import pygame
import numpy as np

from dynamics.cartpole_dynamics import CartPoleDynamics
from controller.KeyboardController import KeyboardController
from controller.PIDcontroller import PIDcontroller
from controller.LQRcontroller import LQRcontroller
from controller.ModelPredictiveController import ModelPredictiveController

# =================
# Config
# =================
MODE = "online"    # "online" or "offline"
SIM_TIME = 10.0    # seconds
FPS = 50           # Framerate for online mode

class simState:
    menu = "menu"
    running = "running"
    stopped = "stopped"

class simulation():
    def __init__(self, mode="online"):
        self.mode = mode
        self.state = simState.menu
        self.running = True

        self.env = CartPoleDynamics()
        self.controller = None
        
        # Store pending initial conditions.
        self.init_x = 0.0
        self.init_theta = 10.0

        if self.mode == "online":
            pygame.init()

            # Import the classes
            from render.PygameRenderer import PygameRenderer
            from UI.UImanager import UImanager

            self.renderer = PygameRenderer(self.env)
            self.ui = UImanager(self) # If set to "None" we ignore ui. 

        if self.mode == "offline":
            from plotter.OfflinePlotter import OfflinePlotter

            self.plotter = OfflinePlotter()

    # =================
    # Controller setup
    # =================
    def set_controller(self, name):
        """
        Docstring for set_controller
        
        :param self: Description
        :param name: Description
        """
        if self.state != simState.menu:
            return
        
        if name == "KEY":
            self.controller = KeyboardController(self.env)
        elif name == "PID":
            self.controller = PIDcontroller(self.env)
        elif name == "LQR": 
            self.controller = LQRcontroller(self.env)
        elif name == "NLLQR":
            self.controller = ModelPredictiveController(self.env)
        else:
            raise ValueError("Unknown controller")
        
        print(f"Controller selected: {name}")

    # =================
    # Sim control
    # ================= 
    def start(self):
        """
        Docstring for start
        
        :param self: Description
        """
        if self.controller is None:
            print("Select a controller")
            return
        
        self.init_theta = self.ui.angle_input.value(default=10.0)
        self.init_x = self.ui.x_input.value(default=0.0)

        self.env.reset(
            x0 = self.init_x,
            theta0 = self.init_theta
        )

        self.state = simState.running
        print("Simulation started (def start(self))")

    def reset(self):
        """
        Docstring for reset
        
        :param self: Description
        """
        self.env.reset(
            x0 = self.init_x,
            theta0 = self.init_theta
        )
        self.state = simState.menu
        print("Simulation reset (def reset(self))")

    # =================
    # Online loop
    # ================= 
    def handle_events(self):
        """
        Docstring for handle_events
        
        :param self: Description
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            self.ui.handle_event(event)

    def update(self):
        """
        Docstring for update
        
        :param self: Description
        """
        if self.state != simState.running:
            return
        
        force = self.controller.get_force(self.env.state, self.env.dt)
        self.env.step(force)

    def render(self):
        """
        Docstring for render
        
        :param self: Description
        """
        self.renderer.render()
        self.ui.draw(self.renderer.screen)
        pygame.display.flip()
        self.renderer.tick(FPS)

    def run_online(self):
        while self.running:
            self.handle_events()
            self.update()
            self.render()

        pygame.quit()
        sys.exit()

    # =================
    # Offline loop
    # ================= 
    def run_offline(self):
        if self.controller is None:
            raise ValueError("Offline mode requires a controller")
        
        t = 0.0
        dt = self.env.dt

        while t < SIM_TIME:
            force = self.controller.get_force(self.env.state, dt)
            self.env.step(force)
            self.plotter.log(self.env.state, t)
            t += dt
        self.plotter.plot()

# =================
# Main init
# ================= 
if __name__ == "__main__":
    sim = simulation(mode=MODE)
    if MODE == "online":
        sim.run_online()
    else:
        sim.set_controller("LQR") # Offline preset
        sim.run_offline()