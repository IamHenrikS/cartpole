"""
The following controller is to simulated the LQR solution 
for linearization 

The LQR source 1:
https://blog.sackarias.se/controlling-the-cart-pole-using-lqr

Source 2: Good youtube video of the solution
https://www.youtube.com/watch?v=hAI8Ag3bzeE

Sources 3: Diagrams of LQR and PID
https://iopscience.iop.org/article/10.1088/1757-899X/263/5/052007/pdf

# Thoughts for improvement:
1. Increase the damping of the system for theta, theta_dot in order to
damp the arm motion for 
2. 

"""
import numpy as np
from scipy.signal import cont2discrete     
from scipy.linalg import solve_discrete_are, solve_continuous_are
from scipy.signal import StateSpace, TransferFunction

class LQRcontroller:
    def __init__(self, env):
        # init conditions (references)
        self.theta_ref = 0.0
        self.x_ref = 0.0
        
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

        # LQR: 
        # Disregarding states: x, x_dot due to inheret drift
        self.Q = np.diag([10e-6, 10e-6, 3e3, 300])
        self.R = np.array([[0.1]])

        # The system is remodeled from continous to discrete
        Ad, Bd, Cd, Dd, dtd = cont2discrete((self.A, self.B, np.eye(4), np.zeros((4,1))), dt)
        self.Ad = Ad
        self.Bd = Bd
        self.Cd = Cd
        self.Dd = Dd
        self.dtd = dtd

        # The Riccati is solved using DARE
        Pd = solve_discrete_are(self.Ad, self.Bd, self.Q, self.R)
        self.Kd = np.linalg.inv(self.R + Bd.T @ Pd @ Bd) @ (Bd.T @ Pd @ Ad)
        A_cl_d = self.Ad - self.Bd @ self.Kd

        # Evaluation of eigenvalues
        poles_d = np.linalg.eigvals(A_cl_d)
        print("Discrete-time closed-loop poles:", poles_d)

        # Note:
        # The linear systems transfer function can be written in the S-domain
        # G(s) = C(sI-A)^-1 B + D

    def get_force(self, state, dt=None):
        """
        Description: Returns the force as we know
        u = -Kd*state[0] (where u = Force [N])
        
        :params:

        :state:
        """
        x, x_dot, theta, theta_dot = state

        #kx ≈ 0.3 – 0.8
        #kd ≈ 0.1 – 0.4
        # This was the final solution we reduce the solution to not care about the system
        # regading x in the LQR and solely handles it with the outside loop with adjusted values
        # This stabilized the solution at the expected place.
        # Implemented as linear state feedback (PD-controller)
        # Remodeled for saturation
        kx=0.3
        kd=0.25
        theta_ref = kx * (self.x_ref - x) - kd * x_dot
        theta_ref = np.clip(theta_ref, -np.deg2rad(5), np.deg2rad(5))

        # --- Inner loop (fast LQR) ---
        x_lin = np.array([
            x,                # do NOT over-penalize x
            x_dot,
            theta - theta_ref,
            theta_dot
        ])

        force = -self.Kd @ x_lin

        return float(force)
