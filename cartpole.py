"""
Solver for the first 
"""
import numpy as np
import pygame
import sys
import matplotlib.pyplot as plt

# --- CartPole Physical Parameters ---
g = 9.8           # gravity
m_cart = 1.0   # cart mass
m_pole = 0.1   # pole mass
m_total = m_cart + m_pole
l_pole = 0.5      # half the pole length
ml_pole = m_pole * l_pole
f_mag = 10.0  # magnitude of force

dt = 0.02         # simulation timestep

# --- Initial State ---
x = 0.0            # cart position
x_dot = 0.0         # cart velocity
theta = 0.00        # pole angle (radians)
theta_dot = 0.0     # pole angular velocity

# --- Pygame Setup ---
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

# Convert physics coordinates to screen
center = width // 2
pixels_per_meter = 100

def dynamics(force):
    global x, x_dot, theta, theta_dot
    
    # Equations of motion
    costheta = np.cos(theta)
    sintheta = np.sin(theta)

    # Due to the system being underdamped. 
    d_theta = 0.6 #d=damp
    b_x = 0.1     # linear-damping 
 
    # Forces and accelerations
    temp = (force + ml_pole * theta_dot**2 * sintheta) / m_total
    theta_acc = (g * sintheta - costheta * temp - d_theta * theta_dot) / (l_pole * (4.0/3.0 - m_pole * costheta**2 / m_total))
    x_acc = temp - ml_pole * theta_acc * costheta / (m_total - b_x*x_dot)

    # Integrate
    x += dt * x_dot
    x_dot += dt * x_acc
    theta += dt * theta_dot
    theta_dot += dt * theta_acc

    # Clamp the movement between the frame.
    x_max = (width/2-50)/pixels_per_meter
    x_min = -(width/2-50)/pixels_per_meter
    if x > x_max:
        x = x_max
        x_dot = 0
    elif x < x_min:
        x = x_min
        x_dot = 0

def draw():
    screen.fill((255, 255, 255))

    # Draw the ground line
    pygame.draw.line(screen, (0, 0, 0), (0, height - 100), (width, height - 100), 4)

    # Cart
    cart_x = center + int(x * pixels_per_meter)
    cart_y = height - 100
    pygame.draw.rect(screen, (0, 0, 0), (cart_x - 40, cart_y - 20, 80, 40))

    # Pole end
    pole_x = cart_x + int(l_pole * 2 * pixels_per_meter * np.sin(theta))
    pole_y = cart_y - int(l_pole * 2 * pixels_per_meter * np.cos(theta))
    pygame.draw.line(screen, (200, 0, 0), (cart_x, cart_y), (pole_x, pole_y), 6)


    pygame.display.flip()

# --- Control Theory --- #





# --- Main Loop ---
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    keys = pygame.key.get_pressed()
    force = 0
    if keys[pygame.K_LEFT]:
        force = -f_mag
    if keys[pygame.K_RIGHT]:
        force = f_mag

    dynamics(force)
    draw()
    clock.tick(50)
