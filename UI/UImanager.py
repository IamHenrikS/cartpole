"""
UI Manager Module

Responsible for creating, rendering, and handling all UI components.
The UImanager does not contain application logic; instead, it delegates
actions via injected callbacks.

# Methodology of coding following: Clean Code (Robert C. Martin)
"""
import pygame
from UI.button import Button
from UI.initInput import initInput

# --- UI Layout Constants ---
PANEL_WIDTH = 260
PANEL_PADDING = 20
BUTTON_HEIGHT = 36
BUTTON_GAP = 10
INPUT_WIDTH_MARGIN = 40

ANGLE_INPUT_Y = 90
X_INPUT_Y = 140
BUTTONS_START_Y = 200

class UImanager:
    """
    Manages all UI widgets (buttons and inputs) and routes user interaction
    events to the correct components based on the application state.
    """

    def __init__(self, *, on_start, on_reset, on_set_controller, get_state):
        """
        Initialize the UI manager.

        Parameters
        ----------
        on_start : callable
            Callback executed when the START button is pressed.
        on_reset : callable
            Callback executed when the RESET button is pressed.
        on_set_controller : callable
            Callback executed when a controller button is selected.
        get_state : callable
            Returns the current application state ("menu" or "running").
        """
        self.on_start = on_start
        self.on_reset = on_reset
        self.on_set_controller = on_set_controller
        self.get_state = get_state

        self.panel = self._create_panel()
        self.buttons = []
        self.widgets = []

        self.angle_input = self._create_angle_input()
        self.x_input = self._create_x_input()

        self._create_menu_buttons()
        self._assemble_widgets()

    # --- helpers ---
    def _create_panel(self):
        """Create the left-side UI panel."""
        screen = pygame.display.get_surface()
        _, height = screen.get_size()
        return pygame.Rect(0, 0, PANEL_WIDTH, height)

    def _create_angle_input(self):
        """Create the angle input field."""
        return initInput(
            (PANEL_PADDING,
             ANGLE_INPUT_Y,
             self.panel.width - INPUT_WIDTH_MARGIN,
             30),
            "Theta [deg]",
            "0.0",
        )

    def _create_x_input(self):
        """Create the X-position input field."""
        return initInput(
            (PANEL_PADDING,
             X_INPUT_Y,
             self.panel.width - INPUT_WIDTH_MARGIN,
             30),
            "X [m]",
            "0.0",
        )

    def _assemble_widgets(self):
        """Assemble all widgets visible in the menu state."""
        self.widgets.extend(self.buttons)
        self.widgets.append(self.angle_input)
        self.widgets.append(self.x_input)

    # --- create buttons ---
    def _create_menu_buttons(self):
        """Create controller selection and action buttons."""
        self._create_controller_buttons()
        self._create_start_button()
        self._create_reset_button()

    def _create_controller_buttons(self):
        """Create buttons for controller selection."""
        button_width = self.panel.width - 2 * PANEL_PADDING
        y_offset = BUTTONS_START_Y

        controllers = [
            ("Keyboard", "KEY"),
            ("PID", "PID"),
            ("LQR", "LQR"),
            ("NMPC", "NMPC"),
        ]

        for label, controller_key in controllers:
            self.buttons.append(
                Button(
                    (PANEL_PADDING, y_offset, button_width, BUTTON_HEIGHT),
                    label,
                    lambda key=controller_key: self.on_set_controller(key),
                )
            )
            y_offset += BUTTON_HEIGHT + BUTTON_GAP

        self._next_button_y = y_offset + 10

    def _create_start_button(self):
        """Create the START button."""
        button_width = self.panel.width - 2 * PANEL_PADDING

        self.buttons.append(
            Button(
                (PANEL_PADDING, self._next_button_y, button_width, BUTTON_HEIGHT),
                "START",
                self.on_start,
            )
        )

    def _create_reset_button(self):
        """
        Create the RESET button.

        The reset button is only active while the application is running.
        """
        button_width = self.panel.width - 2 * PANEL_PADDING

        self.reset_button = Button(
            (PANEL_PADDING, self._next_button_y, button_width, BUTTON_HEIGHT),
            "RESET",
            self.on_reset,
        )

    # --- Rendering ---
    def draw_panel(self, screen):
        """
        Draw the UI background panel and its separating border.
        """
        pygame.draw.rect(screen, (240, 240, 240), self.panel)
        pygame.draw.line(
            screen,
            (0, 0, 0),
            (self.panel.right, 0),
            (self.panel.right, self.panel.height),
            2,
        )

    def draw(self, screen):
        """
        Draw visible UI components based on the current state.
        """
        if self.get_state() == "menu":
            for widget in self.widgets:
                widget.draw(screen)
        elif self.get_state() == "running":
            self.reset_button.draw(screen)

    # --- event ---
    def handle_event(self, event):
        """
        Route pygame events to active UI elements.

        Mouse events outside the panel are ignored.
        """
        if self._is_mouse_event(event) and not self.panel.collidepoint(event.pos):
            return

        if self.get_state() == "menu":
            self._handle_menu_events(event)
        elif self.get_state() == "running":
            self.reset_button.handle_event(event)

    def _handle_menu_events(self, event):
        """Handle events for all menu widgets."""
        for widget in self.widgets:
            widget.handle_event(event)

    @staticmethod
    def _is_mouse_event(event):
        """Return True if the event is a mouse interaction."""
        return event.type in (
            pygame.MOUSEBUTTONDOWN,
            pygame.MOUSEMOTION,
        )
