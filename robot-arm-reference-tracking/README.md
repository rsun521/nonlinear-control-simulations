# Gravity-Assisted Robot Arm Reference Tracking

This project simulates a nonlinear two-link robot arm with only one actuated joint. The control objective is to make the second joint angle track a piecewise-constant reference trajectory while the arm is affected by gravity and torque saturation.

The robot dynamics are modeled as:

M(q)qddot + C(q,qdot)qdot + N(q,qdot) = T

where the torque vector is:

T = [tau1, 0]^T

The actuator torque is limited to ±100 Nm.

## Methods

- Nonlinear robot dynamics
- Symbolic equation-of-motion generation
- Linearization around equilibrium
- State-space control
- Pole placement
- Reference tracking
- Integral action
- Anti-windup under torque saturation
- MATLAB ODE45 simulation

## Controller

The controller uses the structure:

u = -Kx + k_ref r - k_int ∫(q2 - r)dt

The feedback gain was selected using MATLAB's `place()` function with poles:

```matlab
[-3, -4, -1, -20]