import pygame

class KeyboardController:
    def __init__(self, env, force_mag=10.0):
        self.force_mag = force_mag
        self.env = env

    def get_force(self, state, dt):
        keys = pygame.key.get_pressed()
        force = 0.0

        if keys[pygame.K_LEFT]:
            force = -self.force_mag
        if keys[pygame.K_RIGHT]:
            force = self.force_mag
        return force
