"""
Description: Since the system is a nonlinear, underactuated system 
the linearization and optimal control such as LQR solely works on small intervals, 
it is also true that there is a need to adjust for the cart-position. These limitations
can be evaluated through the MPC (model predictive controller).

# Decision: CasADi, a open-source tool for nonlinear optimization. Which is what i will try
and implement.


# Notes: Various ways to solve the NLMPC (nonlinear MPC) but the one i want to try and imålement
is the 

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
        self.x_ref = 0.0
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

        # MPC parameters
        self.N = 40     # Time horizon (steps)
        self.u_max = 50.0 # For limit (evaluate if this is needed)

        # Weights 
        self.w_E = 30      # Energy
        self.w_theta = 40  # Angle
        self.w_x = 15     # Pos
        self.w_u = 0.1    # Input force

        # "Warm start of MPC"
        self.init_MPC = None

        # Initialize the definitions.
        self.build_solver()

    def derivatives(self, state, force):
            """
            Description: Contains the derivative states of the dynamics.

            :param state: The state variables.
            :param force: u = Force (input).
            """
            x = state[0]
            x_dot = state[1]
            theta = state[2]
            theta_dot = state[3]

            # Rework for casADi
            sin_t = casadi.sin(theta)
            cos_t = casadi.cos(theta)
            alpha = 4.0/3.0 - (self.m_p * cos_t**2)/self.m_t

            # Equations of Motion (ddot(theta), ddot(x))
            # Lagrangian theory with non-conservative terms. 
            theta_ddot = (
                self.g * sin_t 
                - cos_t * (force + self.m_p * self.l * theta_dot**2 * sin_t) / self.m_t 
                - (self.mu_p * theta_dot) / (self.l*alpha)
            ) 

            x_ddot = (
                (force + self.m_p * self.l * theta_dot**2 * sin_t)/self.m_t 
                - (self.m_p * self.l * theta_ddot * cos_t / self.m_t)
                - self.mu_c*x_dot
            )

            return casadi.vertcat(
                 x_dot,
                 x_ddot,
                 theta_dot,
                 theta_ddot
            )


    def energy(self, state):
        """
        Function for returning the systems energy.
        
        :param self: The classes import from dynamics and self
        contained values.
        :param state: Contains the state vector.
        """

        theta = state[2]
        theta_dot = state[3]

        # Define the energy as a function for CasADi (and not numpy)
        E = 0.5*self.m_p*self.l**2*theta_dot**2 + self.m_p*self.g*self.l*(1-casadi.cos(theta)) 

        return E
      
    def build_solver(self):
        """
        Description: NMPC solver using CasADi where energy is a global
        variable and the states are to be minimized. This is NLP inside casADi.
        
        The dynamics of RK4 is implemented in the solver due to the need for NMPC
        and casadi to have the full horizon implemented and possibility for differention.

        # Improvements:
        1. Remodel using a method of collocation instead of RK4

        :params self: Class and enviornment specific variables and functions. 
        """
        nx = 4 # No. of states
        nu = 1 # No. of inputs

        # Decision variables (can switch to MX for NMPC)
        X = casadi.SX.sym('X', nx, self.N + 1)
        U = casadi.SX.sym('U', nu, self.N)

        # Initial state
        X0 = casadi.SX.sym('X0', nx)
        #Xref = casadi.SX.sym('Xref', self.N+1)

        # Initialize the cost function
        J = 0 # Cost
        constraints = [] 

        """
        First constraint:
        Enforces condition x=X0 and that the MPC follows 
        the actual measured first state of the system.
        """ 
        constraints.append(X[:, 0]-X0)
        E_ref = self.m_p*self.g*self.l # All conds = 0, reference energy.

        for k in range(self.N):
            # Iteratives over the horizon and solves the cost function
            xk = X[:, k]
            uk = U[:, k]
            E = self.energy(xk) 

            # Cost function:
            J += (self.w_E * (E-E_ref)**2
                + self.w_theta*(1 - casadi.cos(xk[2]))
                + self.w_x * (casadi.cos(xk[2])**2)*(xk[0] - self.x_ref)**2
                + self.w_u * uk[0]**2
            )

            # RK4 from dynamics implemented into solver.
            k1 = self.derivatives(xk, uk)
            k2 = self.derivatives(xk + self.dt/2 * k1, uk)
            k3 = self.derivatives(xk + self.dt/2 * k2, uk)
            k4 = self.derivatives(xk + self.dt * k3, uk) 

            # Asserting next states:
            x_next = xk + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4)
            
            """
            Constraint 2:

            """
            constraints.append(X[:, k+1]-x_next)

        xN = X[:, -1] # Final n=N
        
        # Terminal equilibrium constraints (HARD CODE - not sought after)
       # constraints.append(xN[0] - self.x_ref)   # cart position
       # constraints.append(xN[1])                # cart velocity = 0
       # constraints.append(xN[2])                # pole angle = 0
       # constraints.append(xN[3])                # pole angular velocity = 0
        # Add terminal cost (once)
        JN = (
            50.0 * (xN[0] - self.x_ref)**2                    # Position
            + 1.5 * xN[1]**2                  # Velocity cart    
            + 20 * (1 - casadi.cos(xN[2]))    # Angle
            + 1.0 * xN[3]**2                  # Angle Vel
        )
        
        J += JN

        # NLP (Non Linear Programming)
        # Indexsation for casADI
        nlp = {
            'x': casadi.vertcat(
                casadi.reshape(X, -1, 1),
                casadi.reshape(U, -1, 1)
            ),
            'f': J,
            'g': casadi.vertcat(*constraints),
            'p': X0
        }

        """
        Opts: 
        Interior Point Optimizer which is a software package for solving
        large scale non linear optimization problems. / systems

        Source: https://coin-or.github.io/Ipopt/
        """
        opts = {
                'ipopt.print_level': 0,
                'print_time': 0,
                # --- SPEED LIMITERS ---
                'ipopt.max_iter': 100,            # hard stop on iterations
                'ipopt.tol': 1e-3,                # convergence tolerance
                'ipopt.acceptable_tol': 1e-2,     # early acceptable solution
                'ipopt.acceptable_iter': 5        # accept if good enough for 5 iters
        }

        self.nlp_solver = casadi.nlpsol('solver', 'ipopt', nlp, opts)
        self.n_constraints = sum(c.size1() for c in constraints)

    def get_force(self, state, dt=None):
        """
        Docstring for get_force
        
        :param self: Description
        :param state: Description
        :param dt: Description
        """
        x0 = np.asarray(state).flatten()
        #Xref = np.linspace(x0[0], self.x_ref, self.N+1)

        # Initial
        nx = 4
        nu = 1

        if self.init_MPC is None:
            x_init = np.zeros((nx*(self.N + 1) + nu * self.N, 1))
        else:
            x_init = self.init_MPC
        
        """ Bounds on decision variables """
        # g(X,U)=0 and lbx =< (X,U) =< ubx
        lbx = []
        ubx = []

        # State bounds
        for k in range(self.N + 1):
            # x position
            lbx.append(-5.5)
            ubx.append( 5.5)

            # x_dot
            lbx.append(-np.inf)
            ubx.append( np.inf)

            # theta
            lbx.append(-np.inf)
            ubx.append( np.inf)

            # theta_dot
            lbx.append(-np.inf)
            ubx.append( np.inf)

        # --- Input bounds ---
        for k in range(self.N):
            lbx.append(-self.u_max)
            ubx.append( self.u_max)

        sol = self.nlp_solver(
             x0 = x_init,
             p = x0, #np.concatenate([x0, Xref]),
             lbg = np.zeros(self.n_constraints),
             ubg = np.zeros(self.n_constraints),
             lbx = lbx,
             ubx = ubx
        )

        w_opts = sol['x'].full().flatten()

        """ Shift the trajectory of the MPC """
        X_opt = w_opts[:nx*(self.N+1)].reshape((self.N+1, nx))
        U_opt = w_opts[nx*(self.N+1):]

        X_shift = np.vstack([X_opt[1:], X_opt[-1]])
        U_shift = np.hstack([U_opt[1:], U_opt[-1]])

        self.init_MPC = np.concatenate([
            X_shift.flatten(),
            U_shift.flatten()
        ]).reshape((-1,1))

        offset = nx*(self.N + 1) 
        u0 = w_opts[offset]
        
        return float(u0)