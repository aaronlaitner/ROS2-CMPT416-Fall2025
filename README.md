<div align="center">
    <summary>
      <h1>Safe Learning in the Real World via Adaptive Shielding with Hamilton-Jacobi Reachability (Simulation + ROS2 Prep) </h1>
      <h2>CMPT416 - Fall 2025</h2>
      <br>
    </summary>
</div>

# Overview

This repository provides a simulation framework for safe reinforcement learning using Hamilton-Jacobi (HJ) reachability-based control barrier functions (HJ-CBFs)

Current implementation focuses on:
- Dubins3D navigation in simulation
- SAC-LAG reinforcement learning
- Online safety filtering using precomputed Backward Reachable Tubes (BRTs)
- Structured to support future ROS2 Turtlebot integration

The system supports a single Dubins3D regime and demonstrates stable integration of SAC-LAG with HJ-CBF safety filtering in simulation.

# Safety Filtering Formulation

A learned policy is filtered online using the HJ value function:
```math
\begin{align*}
    \phi(x, \pi_\theta(x)) :=
    \begin{cases}
        \pi_\theta(x) & V(x) \geq \epsilon \\
        \pi_{\text{safe}}(x) & \text{otherwise}
    \end{cases} \quad \text{(Least-Restrictive Safety Filter)}
\end{align*}
```

# Installation
```
conda env create -f environment.yml 
conda activate rhj
pip install -e .
```
## Computing Backward Reachable Tubes (BRTs)
BRT Computations requires the OptimizedDP library (https://github.com/SFU-MARS/optimized_dp)

```
conda activate odp
pip install -e .
pip install gymnasium
python3 redexp/brts/dubins_3d.py
python3 redexp/brts/turtlebot_brt.py
```

# Simulation Training (Dubins3D)
To run SAC-LAG training with optional HJ-CBF safety filtering:
```
python train/train_sac_lag.py \
    --config train/droq_config.py \
    --env_name=Safe-Dubins3d-NoModelMismatch-v1 \
    --cbf \
    --cbf_gamma=1.0 \
    --max_steps=10000 \
    --seed=0
```
