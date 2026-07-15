# Path Planning & Trajectory Optimization Modules

This directory contains the core path-finding algorithms and optimization solvers that generate kinodynamically feasible trajectories for the UAV.

## Module Breakdown

### 1. Kinodynamic RRT* Planner (`kinodynamic.py`)
* **Purpose**: Generates a dynamically feasible sequence of state transitions utilizing OMPL.
* **State Space**: 4D State vector $x = [x, y, v_x, v_y]^T$ modeling physical displacement and linear velocities.
* **Controls**: 2D Acceleration control vector $u = [a_x, a_y]^T$ constrained within physical thresholds.
* **Integrator**: Standard Euler propagation method updating kinematics over discretized intervals ($\Delta t$).

### 2. Perception-Aware Kinodynamic Planner (`perception_aware.py`)
* **Purpose**: Extends basic pathfinding to actively favor routes that facilitate better state estimation (localization).
* **Cost Function**: Evaluates paths based on a combined objective
  $$\text{Total Cost} = \text{Motion Cost} + w_{\text{perc}} \times (1 - \text{Perception Quality})$$
  This penalizes paths traveling through zones with poor visibility or weak texture (low feature count).

### 3. Trajectory Optimizer (`optimization.py`)
* **Purpose**: Refines paths into smooth, obstacle-free, and high-perception trajectories using a gradient-based non-linear optimization solver (L-BFGS-B).
* **Objective Formulation**: Minimizes a composite loss:
  $$J = w_{\text{col}} J_{\text{collision}} + w_{\text{smooth}} J_{\text{smoothness}} + w_{\text{perc}} J_{\text{perception}}$$
  * **Collision Cost**: Approximated using ESDF distance fields and outwards-facing gradients.
  * **Smoothness Cost**: Implements central finite-difference approximations to penalize sudden acceleration spikes (jerk).
  * **Perception Cost**: Keeps the camera centered on visually rich environments.
