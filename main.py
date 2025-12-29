"""
Entry point for the CartPole simulation.

This module is responsible for:
- Selecting simulation mode (online / offline)
- Wiring together controllers, environment, renderer, and UI
- Running the main simulation loop

All domain logic (dynamics, control, rendering) lives in separate modules.

# Note: AI cleanup with Clean Code in mind
"""

from enum import Enum
import sys
import pygame
import numpy as np

from dynamics.cartpole_dynamics import CartPoleDynamics
from controller.KeyboardController import KeyboardController
from controller.PIDcontroller import PIDcontroller
from controller.LQRcontroller import LQRcontroller
from controller.ModelPredictiveController import ModelPredictiveController


# ======================
# Configuration
# ======================

MODE = "offline"          # "online" or "offline"
SIM_TIME = 10.0          # seconds (offline mode)
FPS = 50                 # frames per second (online mode)

# ======================
# Only works Offline: Movement in x-dir during sim
# ======================
OFFLINE_SWEEP = False     # False: Disregard: NEED TO VERIFY THIS

# ======================
# Simulation state
# ======================

class SimState(Enum):
    MENU = "menu"
    RUNNING = "running"


# ======================
# Simulation
# ======================

class Simulation:
    """
    High-level simulation orchestrator.

    Responsibilities:
    - Manage simulation state
    - Connect controller, environment, renderer, and UI
    - Run online or offline simulation loops
    """
    
    def __init__(self, mode: str = "online"):
        self.mode = mode
        self.running = True
        self.state = SimState.MENU

        self.env = CartPoleDynamics()
        self.controller = None

        # Initial conditions (offline mode)
        self.init_x = 0.0
        self.init_theta = 30.0

        if self.mode == "online":
            self._init_online_mode()
        else:
            self._init_offline_mode()

    # ------------------
    # Initialization
    # ------------------

    def _init_online_mode(self):
        from render.PygameRenderer import PygameRenderer
        from UI.UImanager import UImanager

        pygame.init()

        self.renderer = PygameRenderer(self.env)
        self.ui = UImanager(
            on_start=self.start,
            on_reset=self.reset,
            on_set_controller=self.set_controller,
            get_state=lambda: self.state.value,
        )

    def _init_offline_mode(self):
        from plotter.OfflinePlotter import OfflinePlotter
        self.plotter = OfflinePlotter()

    # ------------------
    # Controller setup
    # ------------------

    def set_controller(self, name: str):
        """Select controller by name (UI or offline preset)."""
        if self.state != SimState.MENU:
            return

        controllers = {
            "KEY": KeyboardController,
            "PID": PIDcontroller,
            "LQR": LQRcontroller,
            "NMPC": ModelPredictiveController,
        }

        self.controller = controllers[name](self.env)

    # ------------------
    # Simulation control
    # ------------------

    def start(self):
        """Start simulation from UI-selected initial conditions."""
        if self.controller is None:
            print("Select a controller before starting.")
            return

        self.init_x = self.ui.x_input.value(default=0.0)
        self.init_theta = self.ui.angle_input.value(default=10.0)

        self.env.reset(x0=self.init_x, theta0=self.init_theta)
        self.state = SimState.RUNNING

    def reset(self):
        """Reset simulation to initial conditions."""
        self.env.reset(x0=self.init_x, theta0=self.init_theta)
        self.state = SimState.MENU

    # ------------------
    # Online simulation
    # ------------------

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            self.ui.handle_event(event)

    def step_simulation(self):
        dt = self.env.dt
        force = self.controller.get_force(self.env.state, dt)
        self.env.step(force)

    def update(self):
        if self.state != SimState.RUNNING:
            return
        self.step_simulation()

    def render(self):
        if self.state == SimState.MENU:
            x = self.ui.x_input.value(default=0.0)
            theta = np.deg2rad(self.ui.angle_input.value(default=0.0))
            self.renderer.render(preview=(x, theta))
        else:
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

    # ------------------
    # Offline simulation
    # ------------------

    def run_offline(self):
        if self.controller is None:
            raise ValueError("Offline mode requires a controller.")

        self.env.reset(x0=self.init_x, theta0=self.init_theta)

        t = 0.0
        dt = self.env.dt

        while t < SIM_TIME:

            # --- minimal sweep logic ---
            if OFFLINE_SWEEP:
                if t < 0.3 * SIM_TIME:
                    x_ref = -2.0
                elif t < 0.7 * SIM_TIME:
                    x_ref = 1.0
                else:
                    x_ref = self.init_x
            else:
                x_ref = self.init_x
            
            self.env.x_ref = x_ref
            force = self.controller.get_force(
                self.env.state,
                dt,
            )

            self.env.step(force)
            self.plotter.log(self.env.state, t)

            t += dt

        self.plotter.StatePlotter()


# ======================
# Entry point
# ======================

if __name__ == "__main__":
    sim = Simulation(mode=MODE)

    if MODE == "online":
        sim.run_online()
    else:
        sim.set_controller("LQR")  # Offline preset
        sim.run_offline()
