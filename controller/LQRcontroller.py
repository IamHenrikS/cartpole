"""
The following controller is to simulated the LQR solution 
for linearization 

The LQR source 1:
https://blog.sackarias.se/controlling-the-cart-pole-using-lqr

# Thoughts for improvement:
1. Increase the damping of the system for theta, theta_dot in order to
damp the arm motion for 

"""
import numpy as np
from scipy.signal import cont2discrete     
from scipy.linalg import solve_discrete_are, solve_continuous_are

class LQRcontroller:
    def __init__(self, env):
        # init conditions (references)
        self.theta_ref = 0.0
        self.x_ref = 0.1
        
        # physical units
        g = env.g
        dt = env.dt

        # Dimensions
        m_p = env.m_p
        m_t = env.m_t
        l = env.l

        # Friction and resistance
        mu_p = env.mu_p
        mu_c = env.mu_c

        alpha = 4.0/3.0 - m_p/m_t
        self.A = np.array([[0, 1, 0, 0],
                           [0, -mu_c/m_t, -m_p*g/(m_t*alpha), -m_p*mu_p/(m_t*alpha)],
                           [0, 0, 0, 1],
                           [0, 0, -g/(l*alpha), -mu_p/(l*alpha)]])

        self.B = np.array([
                            [0],
                            [(1/m_t)],
                            [0],
                            [-1/(m_t * l * alpha)]
                            ])

        # Defining the solution of the penalizes.
        self.Q = np.diag([10, 10, 10e6, 10])
        self.R = np.array([[0.1]])

        # The system is remodeled from continous to discrete 
        Ad, Bd, _, _, _ = cont2discrete((self.A, self.B, np.eye(4), np.zeros((4,1))), dt)
        self.Ad = Ad
        self.Bd = Bd

        # The Riccati is solved using DARE
        Pd = solve_discrete_are(self.Ad, self.Bd, self.Q, self.R)
        self.Kd = np.linalg.inv(self.R + Bd.T @ Pd @ Bd) @ (Bd.T @ Pd @ Ad)
        A_cl_d = self.Ad - self.Bd @ self.Kd

        # Evaluation of eigenvalues
        poles_d = np.linalg.eigvals(A_cl_d)
        print("Discrete-time closed-loop poles:", poles_d)

        # Compute feedforward for theta reference
        C = np.array([[1, 0, 0, 0], # tracking x
                      [0, 0, 1, 0]] # tracking theta
                      )
        D = np.zeros((2,1))

        # Solve CARE in order for the Kr
        S = solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.inv(self.R) @ self.B.T @ S
        self.K_r = np.linalg.pinv(D + C @ np.linalg.inv(-self.A + self.B @ self.K) @ self.B)
        print(self.K_r)

    def get_force(self, state, dt=None):
        """
        Description:


        """
        x, x_dot, theta, theta_dot = state
        x_lin = np.array([x - self.x_ref, x_dot, theta - self.theta_ref, theta_dot])

        # u = -K(x-x_ref)+K_r*r (feedforward term)
        # 
        r = np.array([self.x_ref, self.theta_ref])  # reference
        force = - self.Kd @ x_lin + self.K_r @ r
        return float(force)
