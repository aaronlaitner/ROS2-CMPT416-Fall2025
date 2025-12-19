"""
    Author: Aaron Laitner

    Description:
        - A lightweight numerical Dubins vehicle model designed for runtime use
        - "dummy" model removes need for HeteroCL-based symbolic dynamics
        - state is assumed to be [px, py, theta]
        - Action is a 1D angular velocity command u in [-wMax, wMax]
        - Speed is constant and positive
"""
import os
import numpy as np
from redexp.config.dubins_3d import (
    SPEED,
    RADIUS,
    OMEGA_BAD_MODEL_MISMATCH,
    OMEGA_NO_MODEL_MISMATCH,
    OMEGA_GOOD_MODEL_MISMATCH,
    OBSTACLE_RADIUS,
)

# Numerical Dubins dynamics used at runtime
class DummyDubinsCar:
    def __init__(self, wMax, speed, r):
        self.speed = speed # Constant forward speed v (m/s)
        self.wMax = wMax # Maximum angular speed (rad/s). Action bounds are [-wMax, wMax]
        self.r = r  # radius, used for collision checks in environments

    def dynamics_non_hcl(self, t, state, action):
        # Numerical equivalent of the symbolic dynamics used in the HJ reachability computation.
        # Action is converted to a scalar
        # This function doesnt apply dt integration, that is handled by env 

        state = np.asarray(state, dtype=float) # state: [px, py, theta] (3,)
        u = float(np.asarray(action).reshape(()))  # scalar action, scalar or (1,)

        x, y, th = state
        v = self.speed

        x_dot = v * np.cos(th)
        y_dot = v * np.sin(th)
        th_dot = u

        return np.array([x_dot, y_dot, th_dot], dtype=np.float32)

    def opt_ctrl_non_hcl(self, t, state, nabla_V):

        # Optimal control for Dubins car in the direction of the gradient
        # nabla_V: (3,) = [dV/dx, dV/dy, dV/dtheta]

        # For Dubins dynamics, control u only affects theta_dot, so the
        # Hamiltonian contribution from control is ~ nabla_V[2] * u.

        # We pick u at the bounds +/-wMax depending on sign of dV/dtheta.
        # Returned as a 1-D array to match cvxpy parameter shape (action_dim = 1), 
        # did so to avoid "Invalid dimensions" as before returned a scalar
        nabla_V = np.asarray(nabla_V, dtype=float)
        p_theta = nabla_V[2]

        if p_theta > 0:
            u = -self.wMax
        elif p_theta < 0:
            u = self.wMax
        else:
            u = 0.0
            
        return np.array([u], dtype=np.float32)


class GridAdapter:
    # Simple grid adapter for value lookup
    # Used in order to utilize BRT arrays without depending on HeteroCL grid objects at runtime
    def __init__(self, lower, upper, shape):
        lower = np.asarray(lower, dtype=float) # Minimum bounds per dimension 
        upper = np.asarray(upper, dtype=float) # Maximum bounds per dimension
        shape = np.asarray(shape, dtype=int)   # Number of grid points per dimension

        self.min = lower 
        self.max = upper 
        self.pts_each_dim = shape 

        # Grid spacing per dimension: (max - min) / (N - 1)
        self.dx = (self.max - self.min) / (self.pts_each_dim - 1)

    def get_index(self, state):
        # Maps a continuous state to the nearest grid index (rounded)
        # Uses nearest-neighbor rounding rather than interpolation
        state = np.asarray(state, dtype=float)
        ratios = (state - self.min) / (self.max - self.min)
        idx_float = ratios * (self.pts_each_dim - 1)
        # Indices are clipped into a valid range
        idx = np.clip(np.round(idx_float), 0, self.pts_each_dim - 1).astype(int)
        return idx

    def get_value(self, vf, state):
        # Returns the BRT value V(state) using nerest-neighbor indexing
        idx = self.get_index(state)
        return float(vf[tuple(idx)])


# Load precomputed BRT tables
BRT_DIR = os.path.dirname(__file__)

dubins_3d_omega_0_25 = np.load(os.path.join(BRT_DIR, "dubins_3d_omega_0_25_brt.npy"))
dubins_3d_omega_0_5  = np.load(os.path.join(BRT_DIR, "dubins_3d_omega_0_5_brt.npy"))
dubins_3d_omega_0_75 = np.load(os.path.join(BRT_DIR, "dubins_3d_omega_0_75_brt.npy"))

# Grid metadata (matches original)
grid = GridAdapter(
    lower=np.array([-4.0, -4.0, -np.pi]),
    upper=np.array([ 4.0,  4.0,  np.pi]),
    shape=np.array([201, 201, 201]),
)

# Dummy car objects to match the three BRT variants
car_omega_0_25 = DummyDubinsCar(
    wMax=OMEGA_BAD_MODEL_MISMATCH,
    speed=SPEED,
    r=RADIUS,
)

car_omega_0_5 = DummyDubinsCar(
    wMax=OMEGA_NO_MODEL_MISMATCH,
    speed=SPEED,
    r=RADIUS,
)

car_omega_0_75 = DummyDubinsCar(
    wMax=OMEGA_GOOD_MODEL_MISMATCH,
    speed=SPEED,
    r=RADIUS,
)