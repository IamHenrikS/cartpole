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
import numpy as np

class PygameRenderer:
    def __init__(self, env, *, left_margin=260, width=1280, height=600, ppm=100):
        pygame.init()
        self.env = env
        self.width = width
        self.height = height
        self.left_margin = left_margin

        # World viewport
        self.world_width = width - left_margin
        self.world_origin_x = left_margin + self.world_width//2
        
        self.pixels_per_meter = ppm
        self.cart_half_width = 40                        # half width of cart in pixels
        self.cart_height = 40                            # full cart height
        self.ground_y = self.height - 250                # Y-coordinate of ground
        self.y_range = 2.5   # meters up AND down from ground

        self.hud_font = pygame.font.SysFont("Garamond", 14)

        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()

    def world_to_screen_x(self, x_world):
        """Convert physics world x-coordinate to screen pixels."""
        return int(self.world_origin_x + x_world * self.pixels_per_meter)

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
        """Draw X-Y axes indicator in bottom-right of world viewport."""
        margin = 40
        axis_len = 50

        origin_x = self.width - margin
        origin_y = self.ground_y - margin

        # X-axis
        pygame.draw.line(
            self.screen, (0, 0, 255),
            (origin_x - axis_len, origin_y),
            (origin_x, origin_y), 3
        )

        # Y-axis
        pygame.draw.line(
            self.screen, (255, 0, 0),
            (origin_x, origin_y),
            (origin_x, origin_y - axis_len), 3
        )

        # Arrowheads
        pygame.draw.polygon(
            self.screen, (0, 0, 255),
            [
                (origin_x, origin_y),
                (origin_x - 6, origin_y - 3),
                (origin_x - 6, origin_y + 3)
            ]
        )

        pygame.draw.polygon(
            self.screen, (255, 0, 0),
            [
                (origin_x, origin_y - axis_len),
                (origin_x - 3, origin_y - axis_len + 6),
                (origin_x + 3, origin_y - axis_len + 6)
            ]
        )

        # Labels
        self.screen.blit(
            self.hud_font.render("+x", True, (0, 0, 255)),
            (origin_x - axis_len - 15, origin_y - 12)
        )
        self.screen.blit(
            self.hud_font.render("+y", True, (255, 0, 0)),
            (origin_x + 5, origin_y - axis_len - 10)
        )

    def draw_boundaries(self):
        """Draw hard walls at x_min and x_max."""
        width = 10
        wall_color = (255, 0, 0)

        x_left = self.world_to_screen_x(self.env.x_min) - self.cart_half_width - width / 2
        x_right = self.world_to_screen_x(self.env.x_max) + self.cart_half_width + width / 2

        pygame.draw.rect(self.screen, wall_color, (x_left - width // 2, self.ground_y - width, width, 2 * width))
        pygame.draw.rect(self.screen, wall_color, (x_right - width // 2, self.ground_y - width, width, 2 * width))

    def draw_grid(self, x_spacing=1.0, y_spacing=0.5):
        """
        Draw a matplotlib-style grid in world coordinates.
        X-grid: cart position
        Y-grid: height relative to ground
        """
        # ---- Vertical grid lines (X) ----
        x_min = self.env.x_min
        x_max = self.env.x_max

        x = np.ceil(x_min / x_spacing) * x_spacing
        while x <= x_max:
            x_screen = self.world_to_screen_x(x)

            pygame.draw.line(
                self.screen, (220, 220, 220),
                (x_screen, 0),
                (x_screen, self.height), 1
            )

            # Tick label
            label = self.hud_font.render(f"{x:.1f}", True, (120, 120, 120))
            self.screen.blit(label, (x_screen - 10, self.ground_y + 6))

            x += x_spacing

        # ---- Horizontal grid lines (Y) ----
        # World y = 0 at ground
        # ---- Horizontal grid lines (Y, symmetric) ----
        y_min = -self.y_range
        y_max =  self.y_range

        y = np.ceil(y_min / y_spacing) * y_spacing
        while y <= y_max:
            y_screen = self.ground_y - int(y * self.pixels_per_meter)

            pygame.draw.line(
                self.screen, (230, 230, 230),
                (self.left_margin, y_screen),
                (self.width, y_screen), 1
            )

            # Label (skip labeling y=0 twice)
            label = self.hud_font.render(f"{y:.1f}", True, (120, 120, 120))
            self.screen.blit(label, (self.left_margin + 5, y_screen - 8))

            y += y_spacing

    def render(self, *, preview=None):
        self.screen.fill((255, 255, 255))

        world_rect = pygame.Rect(self.left_margin, 0, self.world_width, self.height)
        self.screen.set_clip(world_rect)

        self.draw_grid()
        self.draw_coordinate_system()

        if preview is not None:
            x, theta = preview
            self._render_cartpole(x, theta)
        else:
            self._render_cartpole(self.env.x, self.env.theta)
            self.draw_force_arrow()
            self.draw_hud()

        self.screen.set_clip(None)


    #----#
    def render_preview(self, x, theta):
        self._render_cartpole(x, theta)

    def _render_cartpole(self, x, theta):
        # Ground
        pygame.draw.line(
            self.screen, (0, 0, 0),
            (self.left_margin, self.ground_y),
            (self.width, self.ground_y), 4
        )

        # Cart
        x_cart = self.world_to_screen_x(x)
        y_cart = self.ground_y

        pygame.draw.rect(
            self.screen, (0, 0, 0),
            (x_cart - self.cart_half_width,
            y_cart - self.cart_height // 2,
            self.cart_half_width * 2,
            self.cart_height)
        )

        # Pole
        l_p = int(self.env.l * self.pixels_per_meter)
        x_pole = x_cart + int(l_p * np.sin(theta))
        y_pole = y_cart - int(l_p * np.cos(theta))

        pygame.draw.line(
            self.screen, (255, 0, 0),
            (x_cart, y_cart),
            (x_pole, y_pole), 6
        )


    def tick(self, fps=50):
        """Control framerate."""
        self.clock.tick(fps)
