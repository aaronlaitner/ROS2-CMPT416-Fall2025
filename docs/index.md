<div align="center">
    <summary>
      <h1>Safe Learning in the Real World via Adaptive Shielding with Hamilton-Jacobi Reachability (Simulation + ROS2 Preperation)</h1>
      <h2> Aaron Laitner - Dr. Mo Chen </h2>
      <br>
      <h3> Simon Fraser University, Burnaby, BC, Canada </h3>
      <br>
    </summary>
</div>

### [Code](https://github.com/aaronlaitner/ROS2-CMPT416-Fall2025) · [Video Demo](https://youtu.be/VIDEO_ID)


## Approach

This project extends the safe reinforcement learning using Hamilton-Jacobi (HJ) reachability based control barrier functions (HJ-CBFs).
The project extends the original by leveraging precomputed Backward Reachable Tubes and preparing for ROS2 implementation of turtle. 

### Description

- Integrated the Dubins3d dynamics into the HJ-CBF filter
- Imported the precalculated BRT arrays removing need for original HeteroCL dependency
- Fixed grid indexing and spatial derivative matching
- Hooked V_spa_deriv_at_state into solver
- Got QP to return safe controls
- Patched dynamics to support experiment, including:
  - Opt_ctrl_non_hcl
  - Dynamics_non_hcl
  - Corrected wMax, grid, and BRT dimensions
- Enabled running training simulations with precomputed BRT values

## Why a numerical ("Dummy") Dubins car was added

The original implementation computes HJ reachability sets using OptimizedDP, which relies on HeteroCL for symbolic dynamics. However
HeteroCL dynamics cant be executed inside Gym environments. As such to address this introduced a numerical Dubins Dynamics model ("dummy car") which exactly matches the dynamics used during BRT computation. This allows offline computation of BRTs using HeteroCL, eliminating runtime HeteroCL dependencies. 

## Dubins Vehicle Dynamics

Consider a Dubins vehicle with state:

$$
x = (p_x, p_y, \theta)
$$

and dynamics:


$$
\begin{aligned}
\dot{p}_x &= v \cos \theta \\
\dot{p}_y &= v \sin \theta \\
\dot{\theta} &= u
\end{aligned}
$$

where $\ v$ is constant and 

$$
\ u \in [-\omega_{\max}, \omega_{\max}] \.
$$


## HJ Reachability and Safety Filtering

Backward Reachable Tubes are computed offline via the OptimizedDP toolbox. The resulting value function represents 
the signed distance to the unsafe set. Afterwards real-time safety can occur without requiring HeteroCL during execution.

### Experiments

Ran a 10,000 step SAC-Lagrangian training end to end with the HJ-CBF shield on using precomputed BRT value grids

## Implementation Notes

This implementation adapts the original robust HJ-CBF framework (https://github.com/sudo-michael/robust-hj-cbf-safe-rl) by 
introducing a numerical Dubins dynamics model allowing precomputed BRTs to be used inside Gymnasium and JAX-based reinforcement
learning pipelines.

The project is also designed to support future ROS2 turtle.


