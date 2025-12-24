import pygame

class initInput:
    def __init__(self, rect, label, initial="0.0"):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.text = initial
        self.active = False
        self.font = pygame.font.SysFont("Garamond", 14)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.active = self.rect.collidepoint(event.pos)

        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            elif event.key == pygame.K_RETURN:
                self.active = False
            else:
                if event.unicode in "0123456789.-":
                    self.text += event.unicode

    def value(self, default=0.0):
        try:
            return float(self.text)
        except ValueError:
            return default

    def draw(self, screen):
        color = (255, 255, 255) if self.active else (230, 230, 230)
        pygame.draw.rect(screen, color, self.rect)
        pygame.draw.rect(screen, (0, 0, 0), self.rect, 2)

        label_surf = self.font.render(self.label, True, (0, 0, 0))
        screen.blit(label_surf, (self.rect.x, self.rect.y - 16))

        text_surf = self.font.render(self.text, True, (0, 0, 0))
        screen.blit(text_surf, (self.rect.x + 6, self.rect.y + 8))