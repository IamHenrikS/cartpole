"""

"""

class PIDcontroller:
    def __init__(self, Kp=10.0, Kd=0.1, Kx=1.0, Kdx=2.0):
        self.Kp = Kp      # for theta
        self.Kd = Kd      # for theta_dot
        self.Kx = Kx      # for cart position
        self.Kdx = Kdx    # for cart velocity

    def get_force(self, state):
        """
        Docstring for get_force
        
        :param self: tuning params
        :param state: [x, x_dot, theta, theta_dot]
        """
        x, x_dot, theta, theta_dot = state
        F = -(self.Kp * theta + self.Kd * theta_dot) - (self.Kx * x + self.Kdx * x_dot)
        return F