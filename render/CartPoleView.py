import pygame
import numpy as np
import render.constants as C 

class CartPoleView:
    def __init__(self, env, viewport, y_ground):
        self.env = env
        self.viewport = viewport
        self.y_ground = y_ground
    
    # Public
    def draw(self, screen, x, theta):
        # Implement to the callable func.
        self._draw_grid(screen)
        self._draw_ground(screen)
        self._draw_axis_labels(screen)
        self._draw_ground(screen)
        self._draw_cart(screen, x)
        self._draw_pole(screen, x, theta)
    
    # _: private
    def _draw_ground(self, screen):
        pygame.draw.line(
            screen, C.BLACK, 
            (self.viewport.left_margin, self.y_ground),
            (self.viewport.width, self.y_ground), 4)
    
    def _draw_cart(self, screen, x):
        x_center = self.viewport.world_x_to_screen(x)

        pygame.draw.rect(
            screen, C.BLACK,
            (
                x_center - C.CART_WIDTH_PX//2,
                self.y_ground - C.CART_HEIGHT_PX//2,
                C.CART_WIDTH_PX,
                C.CART_HEIGHT_PX,
            ),
        )

    def _draw_pole(self, screen, x, theta):
        x_base = self.viewport.world_x_to_screen(x)
        l_px = int(self.env.l * self.viewport.ppm)

        x_tip = x_base + int(l_px * np.sin(theta))
        y_tip = self.y_ground-int(l_px * np.cos(theta))

        pygame.draw.line(
            screen, C.RED,
            (x_base, self.y_ground),
            (x_tip, y_tip),
            6
        )

    # The following two methods were solely generated through AI.
    def _draw_grid(self, screen):
        spacing_px = int(self.viewport.ppm * C.GRID_SPACING_M)

        left = self.viewport.left_margin
        right = self.viewport.width
        top = 0
        bottom = self.viewport.height

        x0 = self.viewport.origin_x
        y0 = self.y_ground

        # Vertical grid lines
        i = 0
        x = x0
        while x < right:
            color = C.GRID_MAJOR_COLOR if i % C.GRID_MAJOR_EVERY == 0 else C.GRID_COLOR
            pygame.draw.line(screen, color, (x, top), (x, bottom), 1)
            x += spacing_px
            i += 1

        i = 1
        x = x0 - spacing_px
        while x > left:
            color = C.GRID_MAJOR_COLOR if i % C.GRID_MAJOR_EVERY == 0 else C.GRID_COLOR
            pygame.draw.line(screen, color, (x, top), (x, bottom), 1)
            x -= spacing_px
            i += 1

        # Horizontal grid lines
        i = 0
        y = y0
        while y > top:
            color = C.GRID_MAJOR_COLOR if i % C.GRID_MAJOR_EVERY == 0 else C.GRID_COLOR
            pygame.draw.line(screen, color, (left, y), (right, y), 1)
            y -= spacing_px
            i += 1

        i = 1
        y = y0 + spacing_px
        while y < bottom:
            color = C.GRID_MAJOR_COLOR if i % C.GRID_MAJOR_EVERY == 0 else C.GRID_COLOR
            pygame.draw.line(screen, color, (left, y), (right, y), 1)
            y += spacing_px
            i += 1

    def _draw_axis_labels(self, screen):
        font = pygame.font.SysFont("Garamond", C.AXIS_FONT_SIZE)

        spacing_m = C.GRID_SPACING_M
        spacing_px = int(self.viewport.ppm * spacing_m)

        x0 = self.viewport.origin_x
        y0 = self.y_ground

        bounds = {
            "x+": (x0, self.viewport.width),
            "x-": (x0, self.viewport.left_margin),
            "y+": (y0, 0),
            "y-": (y0, self.viewport.height),
        }

        def draw_labels(axis, sign, start, end):
            i = 1
            pos = start + sign * spacing_px

            while (pos < end if sign > 0 else pos > end):
                if i % C.GRID_MAJOR_EVERY == 0:
                    value = sign * i * spacing_m
                    label = font.render(f"{value:.1f}", True, C.AXIS_LABEL_COLOR)

                    if axis == "x":
                        rect = label.get_rect(center=(pos, y0 + 14))
                    else:
                        rect = label.get_rect(midright=(x0 - 6, pos))

                    screen.blit(label, rect)

                pos += sign * spacing_px
                i += 1

        # X-axis labels
        draw_labels("x", +1, x0, bounds["x+"][1])
        draw_labels("x", -1, x0, bounds["x-"][1])

        # Y-axis labels
        draw_labels("y", -1, y0, bounds["y+"][1])  # up
        draw_labels("y", +1, y0, bounds["y-"][1])  # down
