"""
Main file to be used as the python run file. 
The main utilizes the following files:

Controller: Can be switched with different controllers.
- PIDcontroller: 
- LQRcontroller:
- ML
Dynamics
- cartpole_dynamics(): Dynamics of the cart and the pole.
plotter
- LivePlotter(): Is plotting the errors in real-time. Used for showcasing with pygame.
- OfflinePlotter(): Quicker simulations with focus on analytics of the system.
render:
- PygameRenderer(): Enables pygame and visual addition.

The file operates in two options: offline and online.
The online activates the PygameRenderer() and LivePlotter whilst
offline deactivates those and instead uses OfflinePlotter
whilst storing the values of each run.

Mode:
- offline: Needs to set a simulation time SIM_TIME
- online: runs the enviornment until closed.
"""
import sys
import pygame

from dynamics.cartpole_dynamics import CartPoleDynamics
from controller.PIDcontroller import PIDcontroller

MODE = "offline"    # "online" or "offline"
SIM_TIME = 10.0    # seconds

env = CartPoleDynamics()
controller = PIDcontroller()

# ============================
# ONLINE MODE
# ============================
if MODE == "online":
    from render.PygameRenderer import PygameRenderer
    from plotter.LivePlotter import LivePlotter

    renderer = PygameRenderer(env)

    plotter = LivePlotter(
        theta_ref=controller.theta_ref,
        x_ref=controller.x_ref,
        title="CartPole Tracking Errors"
    )

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        force = controller.get_force(env.state, env.dt)
        env.step(force)

        # Calling LivePlotter() with state variables.
        plotter.update(env.state)

        # Rendition to pygame
        renderer.render()
        renderer.tick(50)

# ============================
# OFFLINE MODE
# ============================
elif MODE == "offline":
    from plotter.OfflinePlotter import OfflinePlotter

    plotter = OfflinePlotter(
        theta_ref=controller.theta_ref,
        x_ref=controller.x_ref
    )

    t = 0.0
    dt = env.dt

    while t < SIM_TIME:
        # Iniate the controller get-func()
        force = controller.get_force(env.state, dt)
        env.step(force)

        plotter.log(env.state, t)
        t += dt

    plotter.plot()

    # Post-processing plots
    #plotter.plot_time_response()
    #plotter.plot_poles_zeros(env)
    #plotter.plot_nyquist(env)