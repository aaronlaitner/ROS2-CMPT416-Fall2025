# CMPT416-fa25 

### Aaron Laitner - Dr. Mo Chen

Simon Fraser University

### [Code](https://github.com/aaronlaitner/ROS2-CMPT416-Fall2025) · [Video Demo](https://youtu.be/VIDEO_ID)

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
- Enabled WandB logging which allows for recording return, safety, violations, brt values, slack, training loss

### Experiments

- Ran a 10,000 step SAC-Lagrangian training end to end with the HJ-CBF shield on using precomputed BRT value grids
