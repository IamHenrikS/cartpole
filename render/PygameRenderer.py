import pygame
import numpy as np

class PygameRenderer:
    def __init__(self, env, width=800, height=600, ppm=100):
        pygame.init()
        self.env = env
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        self.center = width // 2          # zero position in the middle
        self.pixels_per_meter = ppm
        self.cart_half_width = 40         # half width of cart in pixels
        self.ground_y = self.height - 100 # Y-coordinate of ground

    def render(self):
        screen = self.screen
        screen.fill((255, 255, 255))

        # Ground line
        pygame.draw.line(screen, (0,0,0), (0, self.ground_y), (self.width, self.ground_y), 4)

        # Cart position in pixels
        cart_x = self.center + int(self.env.x * self.pixels_per_meter)

        # Clamp cart inside screen (soft clamp)
        min_x = self.cart_half_width
        max_x = self.width - self.cart_half_width
        if cart_x < min_x:
            cart_x = min_x
            self.env.x = (cart_x - self.center) / self.pixels_per_meter
            self.env.x_dot = 0  # stop velocity at edge
        elif cart_x > max_x:
            cart_x = max_x
            self.env.x = (cart_x - self.center) / self.pixels_per_meter
            self.env.x_dot = 0  # stop velocity at edge

        cart_y = self.ground_y

        # Draw cart
        pygame.draw.rect(screen, (0,0,0), (cart_x - self.cart_half_width, cart_y - 20,
                                           self.cart_half_width*2, 40))

        # Draw pole
        pole_length_px = int(self.env.l * 2 * self.pixels_per_meter)
        theta = self.env.theta
        pole_x = cart_x + int(pole_length_px * np.sin(theta))
        pole_y = cart_y + int(pole_length_px * np.cos(theta))
        pygame.draw.line(screen, (255,0,0), (cart_x, cart_y), (pole_x, pole_y), 6)

        # Draw the ball ontop of the pole
        ball_radius = 10 #pixels
        pygame.draw.circle(screen, (255,0,0), (pole_x, pole_y), ball_radius)
        pygame.display.flip()

    # --- Framerate control method ---
    def tick(self, fps=50):
        self.clock.tick(fps)
