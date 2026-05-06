# Spacecraft Attitude Control with Pole Placement

This project simulates the rotational dynamics of a rigid spacecraft and designs a state-feedback controller to stabilize angular velocity around selected equilibrium axes.

The model is based on Euler's rigid-body equations. Because the dynamics are nonlinear, the system is linearized around candidate equilibrium angular velocities:

- we1 = [1, 0, 0]^T
- we2 = [0, 1, 0]^T
- we3 = [0, 0, 1]^T

A controllability matrix is computed for each linearized system. The first two equilibria are fully controllable, while the third is not controllable with the available torque inputs.

## Methods

- Nonlinear rigid-body dynamics
- Linearization around equilibrium angular velocities
- State-space modeling
- Controllability analysis
- Pole-placement control
- MATLAB `place()` function
- ODE45 simulation
- Torque saturation analysis

## Files

- `simulate_spacecraft_attitude.m`: Main spacecraft simulation environment
- `spacecraft_pole_placement_controller.m`: Pole-placement state-feedback controller
- `generate_spacecraft_report_results.m`: Script for controllability analysis, gain computation, and report simulations

## Results

The controller assigns closed-loop poles at:

```matlab
[-2.5, -3.0, -3.5]
