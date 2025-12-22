"""
Description: The purpose is to validate the dynamics, parameterization
and ensure the logic behind the structure is correct before moving into
non-linear development. The following is tested:

- Gravity signage
- Symmetry
- Small angle frequency (compared to inverted pendulum)
- Finite differece between RK4 and normal cond.

# The code is runned by the following command:
pytest --collect-only: Verifies if the tests are gathered properly.
pytest -v tests/tests_dynamics.py: Runs the pytest
pytest -v tests.py
"""

import numpy as np
import pytest
from dynamics.cartpole_dynamics import CartPoleDynamics

def test_gravity_sign():
    """
    Description: Verify that the pole is following the given logic:
    theta > 0: Pole falls to the right from upward pos.
    theta < 0: Pole falls to the left from upward pos.
    theta = 0: Pole is upright at start pos.
    """
    env = CartPoleDynamics(dt=0.030)
    env.theta = 0.05
    env.theta_dot = 0.0
    env.step(force=0.0)

    assert env.theta_dot > 0, "Pole should fall away from upright"

def test_symmetry():
    """
    Description: Verify that the dynamics is symmetric around the positive y-axis
    and that the system has working logic.
    """
    env1 = CartPoleDynamics()
    env2 = CartPoleDynamics()

    env1.theta = 0.1
    env2.theta = -0.1

    env1.step(force=1.0)
    env2.step(force=-1.0)

    assert np.isclose(env1.theta, -env2.theta, atol=1e-6)
    assert np.isclose(env1.x, -env2.x, atol=1e-6)


theta0_deg = [1, 5, 10, 20, 25, 30]
theta0_rad = np.deg2rad(theta0_deg)

@pytest.mark.parametrize("theta0", theta0_rad)
def test_inverted_pendulum(theta0):
    """
    Description: Ensure that the equilibrium is stable and that
    the dynamics of the pendulum is as made.
    """
    env = CartPoleDynamics(dt=0.03)
    env.mu_c = 0.0
    env.mu_p = 0.0

    env.theta = theta0
    env.theta_dot = 0.0
    
    omega = np.sqrt(env.g/env.l)
    max_err = 0.0
    t = 0.0

    for _ in range(2000):
        env.step(force=0.0)
        t += env.dt
        # cosh: Inverted pendulum 
        # cos: Normal pendulum
        # Equation: comes from dual integration of ddot(theta)
        theta_lin = theta0*np.cosh(omega*t)
        max_err = max(max_err, abs(env.theta-theta_lin))
        
    assert max_err < 0.1*theta0
