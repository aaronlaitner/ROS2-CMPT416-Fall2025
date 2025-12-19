import numpy as np


def normalize_angle(theta):
    return ((-theta + np.pi) % (2.0 * np.pi) - np.pi) * -1.0


def spa_deriv(index, V, g):
    spa_derivatives = []
    for dim, idx in enumerate(index):
        if dim == 0:
            left_index = []
        else:
            left_index = list(index[:dim])

        if dim == len(index) - 1:
            right_index = []
        else:
            right_index = list(index[dim + 1 :])

        next_index = tuple(left_index + [index[dim] + 1] + right_index)
        prev_index = tuple(left_index + [index[dim] - 1] + right_index)

        if idx == 0:
            if dim in g.pDim:
                left_periodic_boundary_index = tuple(
                    left_index + [V.shape[dim] - 1] + right_index
                )
                left_boundary = V[left_periodic_boundary_index]
            else:
                left_boundary = V[index] + np.abs(V[next_index] - V[index]) * np.sign(
                    V[index]
                )
            left_deriv = (V[index] - left_boundary) / g.dx[dim]
            right_deriv = (V[next_index] - V[index]) / g.dx[dim]
        elif idx == V.shape[dim] - 1:
            if dim in g.pDim:
                right_periodic_boundary_index = tuple(left_index + [0] + right_index)
                right_boundary = V[right_periodic_boundary_index]
            else:
                right_boundary = V[index] + np.abs(V[index] - V[prev_index]) * np.sign(
                    V[index]
                )
            left_deriv = (V[index] - V[prev_index]) / g.dx[dim]
            right_deriv = (right_boundary - V[index]) / g.dx[dim]
        else:
            left_deriv = (V[index] - V[prev_index]) / g.dx[dim]
            right_deriv = (V[next_index] - V[index]) / g.dx[dim]

        spa_derivatives.append((left_deriv + right_deriv) / 2)

    return np.array(spa_derivatives)


def V_spa_deriv_at_state(state, V, g):
    """
        Author: Aaron Laitner

        Description:
            - Function bridges offline HJ reachability and online safety filtering
            - Value function V(x) tells how close we are to the unsafe set.
            - Spatial gradient of V tells which control direction increases safety.
            - Since V is only known at grid points, grad(V(X)) must be approximated.
            - This is accomplished by snapping the continuous state to its nearest grid cell index, then using finite differences across neighboring grid cells to approximate partial derivatives.
            - Returns V(x) and grad(V(x)) at the nearest grid cell so HJ-CBF QP can evaluate the Hamiltonian constraint online. (Need to later calc grad(V(x))*f(x,u) for hamiltonian/CBF constraint)
    """

    # nearest grid point index, ensures state aligns with same discretization used to compute the precomputed BRT table
    idx = g.get_index(state) # shape (3,)
    idx = np.asarray(idx, dtype=int)
    # Ensures idx is in bounds
    idx = np.clip(idx, 0, g.pts_each_dim - 1)

    # central finite differences
    grad = np.zeros(3) # stores dv/dx, dv/dy, dv/dtheta

    # Loop over each state
    for dim in range(3):

        # Creates a step direction vector e
        e = np.zeros(3, dtype=int)
        # Turns e into a unit step in the current dimension
        # Aka "move one grid cell forward/backward along the dimension"
        e[dim] = 1

        # For each dimension find two neighbors on either side of the current cell

        # Computes plus neighbor index, idx + e means move one step forward along dim
        # np.clip clamps each index so it stays inside valid bounds
        idx_p = np.clip(idx + e, 0, g.pts_each_dim - 1)
        
        # Computes minus neighbor index, idk - e means move one step backward along dim
        # also clipped to be inside the grid
        idx_m = np.clip(idx - e, 0, g.pts_each_dim - 1)

        # Look up the value function at the neighbor grid cells
        f_p = V[tuple(idx_p)] # Value at plus neighbor
        f_m = V[tuple(idx_m)] # Value at minus neighbor

        # Central difference formula for a derivative
        # Used to compute smooth, unbiased numerical approx of the HJ value func gradient from discretized BRT grids
        # This enables stable and accurate safety filtering
        grad[dim] = (f_p - f_m) / (2.0 * g.dx[dim])

    # Value at nearest grid cell
    val = V[tuple(idx)]
    return float(val), grad
