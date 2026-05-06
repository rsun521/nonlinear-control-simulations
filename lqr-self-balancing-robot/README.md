# LQR Control and State Estimation for a Self-Balancing Two-Wheeled Robot

This project simulates and controls a nonlinear two-wheeled self-balancing robot navigating along procedurally generated roads. The robot dynamics are modeled using nonlinear equations of motion, then linearized around an equilibrium operating point for controller and observer design.

The system has six state variables:

- body tilt angle
- tilt angular velocity
- forward velocity
- turning rate
- lateral road error
- heading error

Only partial measurements are available, so a full-state observer is used to estimate the unmeasured states.

The controller combines:

- infinite-horizon LQR optimal control
- observer-based state estimation
- nonlinear simulation
- road-following dynamics
- actuator saturation

The project also includes procedural road generation with collision detection and real-time MATLAB visualization.