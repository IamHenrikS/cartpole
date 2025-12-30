"""
Description: Renderer for seeing the cart-pole problem in pygame enviornment
utilizes the import of pygame and simple geometries to have a visual inspection
of the dynamics of the system.

# Improvements to perform, my thoughts:
1. Split the code into working subclasses for easier modification and integration
2. Add a visual force arrow to the cart.
3. I believe that the draw function should be a class that then can be called on with shapes and 
positions to be implemented? 

# Note: The current code is not refractored and does alot which makes it hard to follow, thus 
a decision is to refractor the full renderer to easier and more readable code.
"""
import pygame
import render.constants as C 

from render.Viewport import Viewport
from render.CartPoleView import CartPoleView
from render.ForceRenderer import ForceRenderer
from render.HudRenderer import HudRenderer

class PygameRenderer:
    """
    High-level renderer coordinating all visual subsystems.
    """

    def __init__(self, env,
                 width=C.DEFAULT_WIDTH,
                 height=C.DEFAULT_HEIGHT,
                 left_margin=C.PANEL_MARGIN,
                 ppm=C.PIXELS_PER_METER):

        pygame.init()
        self.env = env
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()

        self.ground_y = height - C.GROUND_OFFSET_Y

        self.viewport = Viewport(left_margin, width, height, ppm)
        self.cartpole = CartPoleView(env, self.viewport, self.ground_y)
        self.force_renderer = ForceRenderer()
        self.hud = HudRenderer()

    def render(self, preview=None):
        self.screen.fill(C.WHITE)

        x, theta = preview if preview else (self.env.x, self.env.theta)

        self.cartpole.draw(self.screen, x, theta)

        if preview is None:
            self.force_renderer.draw(self.screen, self.env, self.viewport, self.ground_y)
            self.hud.draw(self.screen, self.env)

    def tick(self, fps=50):
        self.clock.tick(fps)
