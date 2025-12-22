"""
Description: A non-linear LQR is implemented. The reasoning
is that the linear LQR cannot account for the drift of the cart due
to not being able to account for the moving dynamics.

# The non-linear system linearizes aswell but experiences the same
struggle as the LQR. Since the system is underactuated the system cannot 
control well enough

# WIP: Need to evaluate the structure, the dynamics and also if we can 
get a smoother filtration by using:

1. Integrator
2. Kalmanfilter
3. Trajectory Optimization for swing-up
4. 

Sources:
2. Kalmanfilter for tracking movement: https://www.youtube.com/watch?v=zSE5YMMTC7s
3. https://www.youtube.com/watch?v=wlkRYMVUZTs
"""
import numpy as np
from scipy.signal import cont2discrete     
from scipy.linalg import solve_discrete_are, solve_continuous_are

class nonlinearLQRcontroller:
    def __init__(self, env):
        # init conditions (references)
        self.env = env
        self.theta_ref = 0.0
        self.x_ref = 0.1
        
        # physical units
        self.g = env.g
        self.dt = env.dt

        # Dimensions
        self.m_p = env.m_p
        self.m_t = env.m_t
        self.l = env.l

        # Friction and resistance
        self.mu_p = env.mu_p
        self.mu_c = env.mu_c

        # Weigths of the system        
        self.Q = np.diag([10, 10, 10e6, 10])
        self.R = np.array([[0.1]])

    def linearize(self, state):
        """
        Description: 
        """
        x, x_dot, theta, theta_dot = state
        g, m_p, m_t, l, mu_p, mu_c = (
            self.g, self.m_p, self.m_t, self.l, self.mu_p, self.mu_c
        )

        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        alpha = 4.0/3.0 - (m_p / m_t)

        A = np.zeros((4, 4))
        A[0,1] = 1
        A[1,1] = -mu_c/m_t
        A[1,2] = -m_p*g/(m_t*alpha)
        A[1,3] = -m_p*mu_p/(m_t*alpha)
        A[2,3] = 1
        A[3,2] = -g/(l*alpha)
        A[3,3] = -mu_p/(l*alpha)

        B = np.zeros((4,1))
        B[1,0] = 1/m_t
        B[3,0] = -1/(l*alpha*m_t)

        return A, B

    def get_force(self, state, dt=None):
        """
        Compute adaptive nonlinear LQR force for any state
        """
        x, x_dot, theta, theta_dot = state

        # Linearize dynamics around current state
        A, B = self.linearize(state)

        # Solve continuous-time ARE
        try:
            S = solve_continuous_are(A, B, self.Q, self.R)
        except np.linalg.LinAlgError:
            # In rare cases, CARE fails; fallback to zeros
            S = np.zeros((4,4))

        # Compute LQR gain
        K = np.linalg.inv(self.R) @ B.T @ S

        # State error
        x_err = np.array([x - self.x_ref, x_dot, theta - self.theta_ref, theta_dot])

        # Compute force
        u = -K @ x_err

        # Saturate to prevent huge forces
        max_force = 50.0
        u = np.clip(u, -max_force, max_force)
        return float(u)