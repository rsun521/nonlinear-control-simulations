# Nonlinear Control Simulations

This repository contains a collection of advanced MATLAB projects in nonlinear dynamical systems and state-space control completed in an upper-level applied mathematics course on linear control systems.

The projects focus on designing controllers for nonlinear mechanical and robotic systems using techniques such as:

- state-space linearization
- pole placement
- observer design
- reference tracking
- integral control
- infinite-horizon LQR
- controllability analysis
- nonlinear simulation with ODE45

For each project, nonlinear equations of motion were linearized around equilibrium points using Jacobians with respect to the system states and control inputs. Controllers were then designed using linear control theory and tested on the full nonlinear dynamics through simulation and animation.

---

# Projects

## 1. Quadrotor Landing Control with Ground Effect

Designed a state-feedback landing controller for a quadrotor under nonlinear aerodynamic ground effects. The controller regulates altitude and vertical velocity to achieve a stable hover approximately 1 mm above the ground.

### Methods
- Nonlinear ODE modeling
- Linearization around hover equilibrium
- PD/state-feedback control
- ODE45 simulation
- Monte Carlo evaluation

---

## 2. Spacecraft Attitude Control

Modeled rigid-body spacecraft rotational dynamics and designed a state-feedback controller using pole placement. Performed controllability analysis for different equilibrium angular velocities and stabilized rotation about controllable axes.

### Methods
- Euler rigid-body dynamics
- Linearization about equilibrium rotations
- Controllability matrices
- Pole placement
- Torque saturation analysis

---

## 3. Gravity-Assisted Robot Arm Reference Tracking

Designed a controller for a nonlinear underactuated two-link robot arm with only one actuated joint. Implemented reference tracking with integral action and anti-windup logic to eliminate steady-state error caused by gravity.

### Methods
- Symbolic equation-of-motion generation
- State-space feedback control
- Reference tracking
- Integral control
- Anti-windup under actuator saturation

---

## 4. LQR Self-Balancing Robot

Developed an observer-based infinite-horizon LQR controller for a nonlinear two-wheeled self-balancing robot navigating procedurally generated roads.

### Methods
- Symbolic linearization
- Infinite-horizon LQR
- Observer/state estimation
- Road-following dynamics
- Nonlinear simulation and visualization

---

# Technical Skills

- MATLAB
- State-Space Control
- LQR
- Observer Design
- Nonlinear Dynamics
- ODE45
- Robotics
- Aerospace Dynamics
- Linear Algebra
- Optimization
- Scientific Computing

---

# Author

Ruize Sun  
Applied Mathematics & Physics  
Vassar College
