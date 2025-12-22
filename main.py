"""
The main utilizes the following files:

Controller: Can be switched with different controllers.
- PIDcontroller: 
- LQRcontroller:
- Non Linear LQR
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
offline instead uses OfflinePlotter whilst storing the values of each run.

Mode:
- offline: Needs to set a simulation time SIM_TIME
- online: runs the enviornment until closed.
"""
import sys
import pygame
import numpy as np

from dynamics.cartpole_dynamics import CartPoleDynamics
from controller.PIDcontroller import PIDcontroller
from controller.LQRcontroller import LQRcontroller
from controller.nonlinearLQRcontroller import nonlinearLQRcontroller

##### CONFIG #####
MODE = "online"    # "online" or "offline"
SIM_TIME = 10.0    # seconds
FPS = 50           # Framerate for online mode

##### Initialization of the environment #####
env = CartPoleDynamics()

##### Selection of Controller #####
#controller = PIDcontroller(env)            # PD-controller
#controller = LQRcontroller(env)            # LQR-controller
controller = nonlinearLQRcontroller(env)    # LQR-controller
#controller = None                          # Placeholder

##### MAIN PROGRAM #####
if MODE == "online":
    from render.PygameRenderer import PygameRenderer
    from plotter.LivePlotter import LivePlotter
    
    # Activates the renderer with environment
    renderer = PygameRenderer(env)

    # Main whil loop for pygame
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        force = 0 if controller is None else controller.get_force(env.state, env.dt)
        env.step(force)
        
        renderer.render()
        renderer.tick(50)
        #plotter.update(env.state) # Optional

elif MODE == "offline":
    from plotter.OfflinePlotter import OfflinePlotter

    if controller is None:
        raise ValueError("Offline mode requires a controller to be initialized.")

    plotter = OfflinePlotter(
        theta_ref=controller.theta_ref,
        x_ref=controller.x_ref
    )

    t = 0.0
    dt = env.dt

    while t < SIM_TIME:
        force = controller.get_force(env.state, dt)
        env.step(force)
        plotter.log(env.state, t)
        t += dt

    plotter.plot()
else:
    raise ValueError("The mode has to be decided 'online' or 'offline'.")
