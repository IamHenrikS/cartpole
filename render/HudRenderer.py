import pygame
import render.constants as C 
import numpy as np

class HudRenderer:
    """Displays state information."""

    def __init__(self):
        self.font = pygame.font.SysFont("Garamond", 14)

    def draw(self, screen, env):
        x = env.x
        theta_deg = np.rad2deg(env.theta)

        lines = [
            f"x = {x:+.3f} m",
            f"θ = {theta_deg:+.2f} deg",
        ]

        for i, text in enumerate(lines):
            surf = self.font.render(text, True, C.BLACK)
            screen.blit(surf, (15, 15 + i * 18))
