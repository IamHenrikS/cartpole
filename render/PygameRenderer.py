"""
Description: Renderer for seeing the cart-pole problem in pygame enviornment
utilizes the import of pygame and simple geometries to have a visual inspection
of the dynamics of the system.

# Improvements to perform:
1. Split the code into working subclasses for easier modification and integration
2. Add a visual force arrow to the cart.
"""
import pygame
import numpy as np

class PygameRenderer:
    def __init__(self, env, width=1280, height=600, ppm=100):
        pygame.init()
        self.env = env
        self.width = width
        self.height = height
        self.center = width // 2                         # zero position in the middle
        self.pixels_per_meter = ppm
        self.cart_half_width = 40                        # half width of cart in pixels
        self.cart_height = 40                            # full cart height
        self.ground_y = self.height - 250                # Y-coordinate of ground
        self.hud_font = pygame.font.SysFont("Garamond", 14)

        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()

    def world_to_screen_x(self, x_world):
        """Convert physics world x-coordinate to screen pixels."""
        return int(self.center + x_world * self.pixels_per_meter)

    def draw_force_arrow(self):
        """Draw a fixed-length blue arrow outside the cart pointing toward it."""
        force = getattr(self.env, "applied_force", 0.0)
        if force == 0:
            return

        x_cart = self.world_to_screen_x(self.env.x)
        y_cart = self.ground_y - 20  # top of cart
        start_y = end_y = y_cart + self.cart_height // 2  # vertical center of cart

        arrow_length = 40
        arrow_size = 8
        visual_buffer = 3

        if force > 0:  # rightward force
            start_x = x_cart - self.cart_half_width - arrow_length - visual_buffer
            end_x = x_cart - self.cart_half_width
            direction = 1
            arrow_tip_x = end_x + visual_buffer
        else:          # leftward force
            start_x = x_cart + self.cart_half_width + arrow_length + visual_buffer
            end_x = x_cart + self.cart_half_width
            direction = -1
            arrow_tip_x = end_x - visual_buffer

        # Draw line
        pygame.draw.line(self.screen, (0, 0, 255), (start_x, start_y), (end_x, end_y), 5)

        # Draw arrowhead
        pygame.draw.polygon(
            self.screen, (0, 0, 255),
            [
                (arrow_tip_x, end_y),
                (arrow_tip_x - direction * arrow_size, end_y - arrow_size // 2),
                (arrow_tip_x - direction * arrow_size, end_y + arrow_size // 2)
            ]
        )

    def draw_hud(self):
        """Render cart position and pole angle on the top-left HUD."""
        x = self.env.x
        theta_deg = np.rad2deg(self.env.theta)
        lines = [f"x = {x:+.3f} m", f"θ = {theta_deg:+.2f} deg"]

        for i, text in enumerate(lines):
            surf = self.hud_font.render(text, True, (0, 0, 0))
            self.screen.blit(surf, (15, 15 + i * 18))

    def draw_coordinate_system(self):
        """Draw a small X-Y axis in the bottom-left corner."""
        origin_x, origin_y = 50, self.height - 50
        axis_len = 50

        # X-axis
        pygame.draw.line(self.screen, (0, 0, 255), (origin_x, origin_y), (origin_x + axis_len, origin_y), 3)
        # Y-axis
        pygame.draw.line(self.screen, (255, 0, 0), (origin_x, origin_y), (origin_x, origin_y - axis_len), 3)

        # Arrowheads
        pygame.draw.polygon(self.screen, (0, 0, 255),
                            [(origin_x + axis_len, origin_y),
                             (origin_x + axis_len - 5, origin_y - 3),
                             (origin_x + axis_len - 5, origin_y + 3)])
        pygame.draw.polygon(self.screen, (255, 0, 0),
                            [(origin_x, origin_y - axis_len),
                             (origin_x - 3, origin_y - axis_len + 5),
                             (origin_x + 3, origin_y - axis_len + 5)])

        # Labels
        self.screen.blit(self.hud_font.render("+x", True, (0, 0, 255)), (origin_x + axis_len + 5, origin_y - 10))
        self.screen.blit(self.hud_font.render("+y", True, (255, 0, 0)), (origin_x - 15, origin_y - axis_len - 20))

    def draw_boundaries(self):
        """Draw hard walls at x_min and x_max."""
        width = 10
        wall_color = (255, 0, 0)

        x_left = self.world_to_screen_x(self.env.x_min) - self.cart_half_width - width / 2
        x_right = self.world_to_screen_x(self.env.x_max) + self.cart_half_width + width / 2

        pygame.draw.rect(self.screen, wall_color, (x_left - width // 2, self.ground_y - width, width, 2 * width))
        pygame.draw.rect(self.screen, wall_color, (x_right - width // 2, self.ground_y - width, width, 2 * width))

    def render(self):
        """Render the entire scene: ground, cart, pole, HUD, coordinate system, and force arrow."""
        self.screen.fill((255, 255, 255))

        # Ground
        pygame.draw.line(self.screen, (0, 0, 0), (0, self.ground_y), (self.width, self.ground_y), 4)

        # Boundaries
        self.draw_boundaries()

        # Cart
        x_cart = self.world_to_screen_x(self.env.x)
        y_cart = self.ground_y
        pygame.draw.rect(self.screen, (0, 0, 0),
                         (x_cart - self.cart_half_width, y_cart - self.cart_height // 2,
                          self.cart_half_width * 2, self.cart_height))

        # Pole
        l_p = int(self.env.l * self.pixels_per_meter)
        theta = self.env.theta
        x_pole = x_cart + int(l_p * np.sin(theta))
        y_pole = y_cart - int(l_p * np.cos(theta))

        # HUD, coordinate system, and force arrow
        self.draw_hud()
        self.draw_coordinate_system()
        self.draw_force_arrow()

        # Pole line
        pygame.draw.line(self.screen, (255, 0, 0), (x_cart, y_cart), (x_pole, y_pole), 6)

    def tick(self, fps=50):
        """Control framerate."""
        self.clock.tick(fps)
