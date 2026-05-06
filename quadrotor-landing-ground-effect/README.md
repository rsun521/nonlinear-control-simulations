# Quadrotor Landing Control with Nonlinear Ground Effect

This project simulates the vertical landing dynamics of a quadrotor under nonlinear ground effect. The goal is to design a controller that brings the quadrotor to a stable hover 1 mm above the ground without crashing.

The controller uses a state-feedback law based on altitude error and vertical velocity:

u = -k1(z - z0) - k2*zdot

The control input is added to the equilibrium thrust needed to hover at the target height. The final thrust command is also clamped to the physical actuator limit.

## Methods

- Nonlinear ODE modeling
- Linearization around an equilibrium point
- State-space representation
- PD / state-feedback control
- MATLAB ODE45 simulation
- Monte Carlo-style randomized testing
- Settling-time and success-rate evaluation

## Files

- `simulate_quadrotor_landing.m`: Main simulation environment
- `quadrotor_pd_controller.m`: State-feedback landing controller
- `evaluate_quadrotor_controller.m`: Runs 100 trials and computes success rate and average settling time

## Results

The best tested controller used:

- k1 = 30
- k2 = 8.995

This achieved a 95% success rate with an average settling time of about 1.395 seconds. More conservative gains achieved 100% success but with longer settling times, showing a trade-off between speed and robustness.
