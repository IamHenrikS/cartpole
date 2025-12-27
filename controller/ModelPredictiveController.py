"""
Description: Since the system is a nonlinear, underactuated system 
the linearization and optimal control such as LQR solely works on small intervals, 
it is also true that there is a need to adjust for the cart-position. These limitations
can be evaluated through the MPC (model predictive controller).

# Decision: CasADi, a open-source tool for nonlinear optimization. Which is what i will try
and implement.


Sources:
2. Kalmanfilter for tracking movement: https://www.youtube.com/watch?v=zSE5YMMTC7s
3. https://www.youtube.com/watch?v=wlkRYMVUZTs
4. MPC: https://www.mdpi.com/1424-8220/22/1/243
5. https://pmc.ncbi.nlm.nih.gov/articles/PMC8749679/ 
6. https://www.youtube.com/watch?v=NkeqFtKH4Yo
7. https://skoge.folk.ntnu.no/prost/proceedings/adconip-2014/pdf/SUBS61TO80/0068/0068_FI.pdf

"""
import numpy as np
import casadi

class ModelPredictiveController:
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
        pass