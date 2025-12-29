
import pygame
import render.constants as C 

class ForceRenderer:
    """ Description: """
    def draw(self, screen, env, viewport, y_ground):
        force = getattr(env, "applied_force", 0.0)
        if force == 0:
            return
        
        direction = 1 if force > 0 else -1
        length = 40
        arrow_size = 8
        
        x_cart = viewport.world_x_to_screen(env.x)
        y = y_ground

        start_x = x_cart - direction * (C.CART_WIDTH_PX // 2 + length)
        end_x = x_cart - direction * (C.CART_WIDTH_PX // 2)

        pygame.draw.line(screen, C.BLUE, (start_x, y), (end_x, y), 5)

        pygame.draw.polygon(
            screen, C.BLUE,
            [
                (end_x, y),
                (end_x - direction * arrow_size, y - arrow_size // 2),
                (end_x - direction * arrow_size, y + arrow_size // 2),
            ]
        )