"""
Docstring for UI.UImanager
- Allow the UImanager to handle the different buttons,
text fields and so on whilst the individual subclasses handles the actions.

# To-Do:
1. Finish the documentation and correct the positions of the buttons
2. Finish the docstrings
"""
import pygame
import numpy as np
from UI.button import Button
from UI.initInput import initInput
from main import simState

class UImanager:
    def __init__(self, sim):
        """
        Docstring for __init__:
        To modify the UI with inputs this is done through
        the list widgets which stores the new objects and is
        responsible for placing on the pygame renderer.
        
        :param self: Description
        :param sim: Description
        """
        self.sim = sim

        # Setting the UIs
        self.buttons = []
        self.widgets = []
        self.angle_input = initInput((10, 70, 70, 30), 
                                     "Theta [deg]", 
                                     f"{np.rad2deg(sim.env.theta):.2f}")
        
        self.x_input = initInput((110, 70, 70, 30), 
                                 "X [m]", 
                                 f"{sim.env.x:.2f}")

        self._create_menu_buttons()

        self.widgets.extend(self.buttons)
        self.widgets.append(self.angle_input)
        self.widgets.append(self.x_input)

    def _create_menu_buttons(self):
        screen = pygame.display.get_surface()
        width, height = screen.get_size()

        # Width, Heigh and Gap between buttons
        w = 120
        h = 40
        gap = 12

        # Placement on the screen
        y = height - h - 20    # bottom margin
        x_start = 250          # left margin

        labels = [
            ("Keyboard", "KEY"),
            ("PID", "PID"),
            ("LQR", "LQR"),
            ("NMPC", "NMPC"),
        ]

        self.buttons = []
        
        # Loop through tuple (index, value) through labels and key
        for i, (label, key) in enumerate(labels):
            x = x_start + i * (w + gap)
            self.buttons.append(
                Button(
                    (x, y, w, h),
                    label,
                    lambda k=key: self.sim.set_controller(k),
                )
            )

        # Start (seperate button due to starting sim.)
        self.buttons.append(
            Button(
                (x_start + len(labels) * (w + gap) + 30, y, w, h),
                "START",
                self.sim.start,
            )
        )

        self.reset_button = Button(
            (800, y, 120, 40),      # Might need to tweak the x-value, not super aligned. 
            "RESET",
            self.sim.reset,
        )

    def handle_event(self,event):
        
        if self.sim.state == simState.menu:
            for i in self.widgets:
                i.handle_event(event)
        elif self.sim.state == simState.running:
            self.reset_button.handle_event(event)
        
    def draw(self,screen):

        if self.sim.state == simState.menu:    
            for i in self.widgets:
                i.draw(screen)
        elif self.sim.state == simState.running:
            self.reset_button.draw(screen)

        

