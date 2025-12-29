"""
Description: Separate the transform for pygame metrics to int metrics.
"""
class Viewport:
    """ World to screen coordinate transforms """
    def __init__(self, left_margin, width, height, ppm):
        self.left_margin = left_margin
        self.width = width
        self.height = height
        self.ppm = ppm  # Pixels Per Meter

        self.world_width = width-left_margin
        self.origin_x = left_margin + self.world_width//2

    def world_x_to_screen(self, x_world):
        return int(self.origin_x + x_world * self.ppm)
    
    def world_y_to_screen(self, y_world, y_ground):
        return int(y_ground - y_world * self.ppm)