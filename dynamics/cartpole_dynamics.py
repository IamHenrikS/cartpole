"""
Description: This class sets the enviornment (env) for the 
cart pole problem. The following main structure is that the
numerical integrator, Equations of Motion and state updates.

The following sources has been used: 
Source 1: Cart-Pole Optimal Control
https://openmdao.github.io/dymos/examples/cart_pole/cart_pole.html
Source 2: Cart-Pole System: Equations of motion.
https://courses.ece.ucsb.edu/ECE594/594D_W10Byl/hw/cartpole_eom.pdf 
Source 3: Main source for the EoM
https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html

# TO-DO Verify this in comparission with the backup files:
1. Check so that the dynamics is the same.
2. Update the descriptions and show the flow-chart of the system.
"""
import numpy as np

class CartPoleDynamics:
    def __init__(self, dt=0.030):
        """
        Description: Initializes the parameters of the cart-pole. It also ensures
        that the state is resetted().

        Source: https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html
        """
        # Parameters:
        self.g = 9.82
        self.m_c = 1.0
        self.m_p = 0.1
        self.m_t = self.m_c + self.m_p
        self.l = 1
        self.dt = dt
        
        # Friction
        self.mu_c = 0.2 # Cart-track friction coeff
        self.mu_p = 0.2 # Cart-pole friction coeff

        # Limits (meters)
        self.x_min = -5.5
        self.x_max =  5.5

        # State
        self.reset()

    def reset(self, x0=0.0, theta0=-180):
        """
        Description: State initialization.

        Theta = 0: Upright, Theta < 0: Pole leaning left, Theta > 0: Pole leaning right.
        The initial values of the systems.
        """
        self.x = float(x0)
        self.x_dot = 0.0
        self.theta = np.deg2rad(theta0) # I want to move this out to the system renderer. 
        self.theta_dot = np.deg2rad(0)
    
    @property
    def state(self):
        """ Description: Creates new Numpy Array every call. """
        return np.array([self.x, self.x_dot, self.theta, self.theta_dot], dtype=float)
 
    def step(self, force):
        """
        Description: Generates the steps itreative progressions of the dynamics.
        Advance the cart-pole state by one timestep using 4th-order Runge-Kutta integration.
        Improvements of the small steps in the system. This is based on the works of
        https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html where the
        showcase of how RK4 and RK2 is more effective than Euler approximations of the dynamics.  
        """
        # Allow the last applied force be singulary stored.
        # Used in PygameRenderer
        self.applied_force = force
        dt = self.dt

        def derivatives(state, force):
            """
            Description: Contains the derivative states of the dynamics
            """
            x, x_dot, theta, theta_dot = state
            g, m_p, m_t, l, mu_p, mu_c = self.g, self.m_p, self.m_t, self.l, self.mu_p, self.mu_c

            sin_t = np.sin(theta)
            cos_t = np.cos(theta)
            alpha = 4.0/3.0 - (m_p * cos_t**2)/m_t

            # Equations of Motion (ddot(theta), ddot(x))
            # Lagrangian theory with non-conservative terms. 
            theta_ddot = (
                g * sin_t 
                - cos_t * (force + m_p * l * theta_dot**2 * sin_t) / m_t 
                - (mu_p * theta_dot) / (l*alpha)
            ) 

            x_ddot = (
                (force + m_p * l * theta_dot**2 * sin_t)/m_t 
                - (m_p * l * theta_ddot * cos_t / m_t)
                - mu_c*x_dot
            )

            return np.array([x_dot, x_ddot, theta_dot, theta_ddot], dtype=float)

        state = np.array([self.x, self.x_dot, self.theta, self.theta_dot], dtype=float)

        k1 = derivatives(state, force)
        k2 = derivatives(state + dt/2 * k1, force)
        k3 = derivatives(state + dt/2 * k2, force)
        k4 = derivatives(state + dt * k3, force)

        # RK4 and defining next state.
        state_next = state + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        self.x, self.x_dot, self.theta, self.theta_dot = state_next

        # Defining the limits for the hardstop
        # Implement a spring damp mass for the solution 
        # This way we avoid the cart just losing full momentum breaking the simulation.
        if self.x < self.x_min:
            self.x = self.x_min
            if self.x_dot < 0:
                self.x_dot = 0.0
        elif self.x > self.x_max:
            self.x = self.x_max
            if self.x_dot > 0:
                self.x_dot = 0.0

    def pole_tip_position(self):
        """
        Returns the Cartesian position of the pole tip.

        Coordinate frame:
        - x: horizontal (right positive)
        - y: vertical (up positive)
        - theta = 0 corresponds to pole upright

        Assumptions:
        - self.l is the distance from pivot to pole center of mass
        - Full pole length = 2 * self.l
        """
        pole_length = 2.0 * self.l

        x_tip = self.x + pole_length * np.sin(self.theta)
        y_tip = pole_length * np.cos(self.theta)

        return x_tip, y_tip

    
    def angle_indicator_position(self, radius=1.0):
        """
        Returns a point used to visualize the pole angle.

        This is NOT a physical point on the pole.
        It is intended only for plotting the angle direction.

        Parameters
        ----------
        radius : float
            Length of the indicator vector.
        """
        x = self.x + radius * np.sin(self.theta)
        y = radius * np.cos(self.theta)

        return x, y

                


