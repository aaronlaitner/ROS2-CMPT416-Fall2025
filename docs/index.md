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
This project extends prior work from Michael Lu et al., *Safe Learning in the Real World via Adaptive Shielding with Hamilton-Jacobi Reachability*. 
This project enabled robust safety filtering during reinforcement learning using precomputed Backward Reachable Tubes (BRTs), while preparing the codebase for future deployment on real robotic platforms using ROS2.

The current implementation focuses on **simulation-based validation** using a Dubins vehicle model and SAC-Lagrangian (SAC-LAG) reinforcement learning.

### Description

- Integrated the Dubins3d dynamics into the HJ-CBF filter
- Imported the precalculated BRT arrays removing need for original HeteroCL dependency
- Fixed grid indexing and spatial derivative matching
- Hooked V_spa_deriv_at_state into solver
- Got QP to return safe controls
- Patched system dynamics to support simulation-based experiments, including:
  - Opt_ctrl_non_hcl
  - Dynamics_non_hcl
  - Corrected alignment of wMax, grid, and BRT dimensions
- Enabled running training simulations with precomputed BRT values

## Why a Numerical ("Dummy") Dubins Car was Added

The original implementation computes Backward Reachable Tubes using OptimizedDP library, which relies on HeteroCL to symbollically define system dynamics. 

While suitable for offline reachability computation, HeteroCL-based dynamics cannot be executed directly inside Gymnasium environments or JAX-based reinforcement learning pipelines.

To address this limitation, a numerical Dubins dynamics model ("dummy car") was introduced. This model:

- Exactly matches the dynamics assumptions used during BRT computation
- Supports forward simulation and control evaluation
- Exposes only the functions required for safety filtering and QP-based control projection

This design allows BRTs to be computed **offline** using OptimizedDP and then reused **online** during learning without requiring HeteroCL at runtime.

## Dubins Vehicle Dynamics

The Dubins vehicle is a simplified model of a ground robot that:

- Moves at a constant forward speed
- Can only change direction by applying an angular velocity
- Cannot move sideways or stop

This model ensures consistency between the system dynamics used for reachability analysis and those used during reinforcement learning. The numerical Dubins model implemented here mirrors the original reachability dynamics and enables safe policy execution using precomputed BRT value functions.


## HJ Reachability and Safety Filtering

Backward Reachable Tubes are computed offline via the OptimizedDP toolbox. The resulting value function represents 
the signed distance to the unsafe set. Afterwards real-time safety can occur without requiring HeteroCL during execution.

### Experiments

- Successfully ran a **10,000-step SAC-Lagrangian training run** with the HJ-CBF safety filter enabled
- Verified stable integration of:
  - Precomputed BRTs
  - Numerical Dubins dynamics
  - QP-based safety filtering

## Implementation Notes

This implementation adapts the original robust HJ-CBF framework (https://github.com/sudo-michael/robust-hj-cbf-safe-rl) by 
introducing a numerical Dubins dynamics model enabling use of precomputed BRTs to be used within Gymnasium and JAX-based reinforcement
learning pipelines.

The project is also designed to support future ROS2-based Turtlebot integration.
