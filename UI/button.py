import pygame

class Button:
    def __init__(self, rect, text, callback, font=None):
        self.text = text
        self.callback = callback
        self.hovered = False

        self.rect = pygame.Rect(rect)
        self.font = font or pygame.font.SysFont("Garamond", 14)

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.callback()

    def draw(self, screen):
        color = (180, 180, 180) if self.hovered else (220, 220, 220)
        pygame.draw.rect(screen, color, self.rect)
        pygame.draw.rect(screen, (0, 0, 0), self.rect, 2)

        text_surf = self.font.render(self.text, True, (0, 0, 0))
        text_rect = text_surf.get_rect(center=self.rect.center)
        screen.blit(text_surf, text_rect)
