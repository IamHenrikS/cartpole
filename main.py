"""
Docstring for main

Main block for the cartpole project.
"""
import pygame
import sys

from dynamics.cartpole_dynamics import CartPoleDynamics
from render.PygameRenderer import PygameRenderer
from controller.PIDcontroller import PIDcontroller
from plotter.LivePlotter import LivePlotter

env = CartPoleDynamics()
renderer = PygameRenderer(env)
controller = PIDcontroller()
plotter = LivePlotter("CartPole: Tip & Angle")
plotting = "no"

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Get force
    force = controller.get_force(env.state)
    env.step(force)

    if plotting == "yes": 
        # Get tip and angle
        tip_pos = env.poleposition()
        angle = env.theta  # angle in radians
        plotter.update(tip_pos, angle)
    elif plotting == "no":
        pass


    # Render pygame
    renderer.render()
    renderer.tick(50)
