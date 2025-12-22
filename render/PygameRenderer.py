"""
Description: Renderer for seeing the cart-pole problem in pygame enviornment
utilizes the import of pygame and simple geometries to have a visual inspection
of the dynamics of the system.

"""

import pygame
import numpy as np

class PygameRenderer:
    def __init__(self, env, width=1280, height=600, ppm=100):
        pygame.init()
        self.env = env
        self.width = width
        self.height = height
        self.center = width // 2          # zero position in the middle
        self.pixels_per_meter = ppm
        self.cart_half_width = 40         # half width of cart in pixels
        self.ground_y = self.height - 250 # Y-coordinate of ground

        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
    
    def draw_coordinate_system(self):
        screen = self.screen

        # Place coordinate system in bottom-left corner
        origin_x = 50                # small offset from left
        origin_y = self.height - 50  # small offset from bottom
        axis_len = 50                # shorter arrows

        # X-axis
        pygame.draw.line(screen, (0, 0, 255),
                        (origin_x, origin_y),
                        (origin_x + axis_len, origin_y), 3)
        # Y-axis
        pygame.draw.line(screen, (255, 0, 0),
                        (origin_x, origin_y),
                        (origin_x, origin_y - axis_len), 3)

        # Arrowheads on x,y-axis
        pygame.draw.polygon(screen, (0, 0, 255),
                            [(origin_x + axis_len, origin_y),
                            (origin_x + axis_len - 5, origin_y - 3),
                            (origin_x + axis_len - 5, origin_y + 3)])
        pygame.draw.polygon(screen, (255, 0, 0),
                            [(origin_x, origin_y - axis_len),
                            (origin_x - 3, origin_y - axis_len + 5),
                            (origin_x + 3, origin_y - axis_len + 5)])

        # Labels
        font = pygame.font.SysFont("Garamond", 14)
        screen.blit(font.render("+x", True, (0, 0, 255)), (origin_x + axis_len + 5, origin_y - 10))
        screen.blit(font.render("+y", True, (255, 0, 0)), (origin_x - 15, origin_y - axis_len - 20))

    def world_to_screen_x(self, x_world):
        """Convert physics world x-coordinate to screen pixels."""
        return int(self.center + x_world * self.pixels_per_meter)

    def draw_boundaries(self):
        """Draw hard boundary walls at x_min and x_max."""
        width = 10
        wall_color = (255, 0, 0)

        x_left = self.world_to_screen_x(self.env.x_min) - self.cart_half_width - width/2
        x_right = self.world_to_screen_x(self.env.x_max) + self.cart_half_width + width/2

        # Walls
        pygame.draw.rect(self.screen, wall_color, (x_left - width // 2, self.ground_y-width, width, 2*width))
        pygame.draw.rect(self.screen, wall_color, (x_right - width // 2, self.ground_y-width, width, 2*width))

    def render(self):
        screen = self.screen
        screen.fill((255, 255, 255))

        # Ground line
        pygame.draw.line(screen, (0, 0, 0), (0, self.ground_y), (self.width, self.ground_y), 4)

        # Draw boundaries first
        self.draw_boundaries()

        # Cart position, world -> pygame
        x_cart = self.world_to_screen_x(self.env.x)
        y_cart = self.ground_y

        # Draw cart
        pygame.draw.rect(screen, (0, 0, 0),
                         (x_cart - self.cart_half_width, y_cart - 20,
                          self.cart_half_width * 2, 40))

        # Draw pole
        l_p = int(self.env.l * self.pixels_per_meter)  # pixels for length l
        theta = self.env.theta

        x_pole = x_cart + int(l_p * np.sin(theta))
        y_pole = y_cart - int(l_p * np.cos(theta))

        self.draw_coordinate_system()

        pygame.draw.line(screen, (255, 0, 0), (x_cart, y_cart), (x_pole, y_pole), 6)
        pygame.display.flip()

    # --- Framerate control ---
    def tick(self, fps=50):
        self.clock.tick(fps)
