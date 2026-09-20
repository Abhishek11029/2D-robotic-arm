# 2-DOF Robotic Arm — MIMO PID Control

## Overview

This project implements a **2-link, 2-degree-of-freedom (2-DOF) robotic arm** controlled using a **MIMO PID controller** in MATLAB.

The robotic manipulator consists of two revolute joints connected in series. Each joint is actuated independently through a motor torque input, while the dynamics of the two joints remain coupled through the inertia, Coriolis/centrifugal, and gravitational terms.

The mathematical model is based on the nonlinear robotic manipulator equation:

$$
M(\theta)\ddot{\theta}+C(\theta,\dot{\theta})+G(\theta)=F
$$

where:

* `M(theta)` — Inertia matrix
* `C(theta,dtheta)` — Coriolis and centrifugal terms
* `G(theta)` — Gravity vector
* `F = [tau1; tau2]` — Applied joint torque vector

## Control Strategy

The controller uses a decoupling approach to simplify the coupled nonlinear system.

The physical torque is related to the decoupled control input by:

$$
F=M(\theta)\hat{F}
$$

where

$$
\hat{F}=
\begin{bmatrix}
f_1\\
f_2
\end{bmatrix}
$$

Each joint is controlled using an independent PID controller:

$$
f_i=K_Pe_i+K_D\dot e_i+K_I\int e_i\,dt
$$

The tracking errors are:

$$
e_1=\theta_{1,ref}-\theta_1
$$

$$
e_2=\theta_{2,ref}-\theta_2
$$

The objective is for both joint angles to converge to their respective reference angles.

## System Architecture

```text
       θ1 Reference ─────┐
                         │
                         ▼
                    ┌─────────┐
                    │ PID 1   │─── f1 ──┐
                    └─────────┘         │
                                        │
                                        ▼
                                  ┌───────────┐
                                  │ Dynamic   │
                                  │ Decoupling│
                                  │    M(θ)   │
                                  └───────────┘
                                        │
                                        ▼
                                   τ1 , τ2
                                        │
                                        ▼
                              ┌─────────────────┐
                              │ 2-DOF Robot Arm │
                              └─────────────────┘
                                  │         │
                                  ▼         ▼
                                 θ1        θ2
                                  │         │
                                  └────┬────┘
                                       │
                                    Feedback

       θ2 Reference ────► PID 2
```

## Files

### `robotDynamics.m`

Contains the nonlinear dynamic model of the 2-DOF robotic manipulator.

It calculates:

* Inertia matrix `M`
* Coriolis/centrifugal vector `C`
* Gravity vector `G`
* Physical motor torque `tau`
* Joint accelerations

### `main.m`

Runs the simulation and:

* Defines robot parameters
* Defines PID gains
* Defines initial conditions
* Defines target joint angles
* Runs the MATLAB ODE solver
* Plots joint-angle responses

## Model Assumptions

The model follows the assumptions specified for the project:

* Links are considered rigid.
* Link masses are modeled as concentrated point masses.
* Joint friction is neglected.
* The system has two revolute joints.
* The initial state determines the starting configuration of the manipulator.

## Inputs

The primary control inputs are the desired joint angles:

```matlab
theta_ref = [theta1_ref;
             theta2_ref];
```

For example:

```matlab
theta_ref = [pi/4;
             pi/3];
```

corresponds to:

* Joint 1 target = 45°
* Joint 2 target = 60°

## Outputs

The simulation produces:

* Joint 1 angle `theta1`
* Joint 2 angle `theta2`
* Tracking response of both joints
* Comparison between actual and desired joint positions

The main objective is:

$$
\theta_1 \rightarrow \theta_{1,ref}
$$

$$
\theta_2 \rightarrow \theta_{2,ref}
$$

with the tracking error approaching zero.

## How to Run

1. Open MATLAB.
2. Place `main.m` and `robotDynamics.m` in the same folder.
3. Open `main.m`.
4. Set the robot parameters.
5. Set the PID gains.
6. Set the desired joint angles.
7. Run:

```matlab
main
```

The simulation will generate the joint-angle response plot.

## PID Tuning

The PID parameters can be modified in `main.m`:

```matlab
Kp = [50 0;
      0 50];

Kd = [10 0;
      0 10];

Ki = [5 0;
      0 5];
```

The gains determine the tracking behavior of the two joints.

Higher proportional gain generally increases the response to tracking error, while derivative gain provides damping and integral gain reduces steady-state error.

The final gains should be tuned according to the actual robot parameters and desired response.

## System Classification

The robotic manipulator is classified as:

* **MIMO** — Multiple Input, Multiple Output
* **Nonlinear** — Dynamic matrices depend on joint states
* **Dynamical** — Joint states evolve according to differential equations
* **Causal** — Current output depends on current and previous system states/inputs

## Future Improvements

Possible extensions include:

* Automatic PID gain tuning
* Friction modeling
* Motor dynamics
* Encoder feedback
* Workspace trajectory tracking
* Joint-limit constraints
* Disturbance rejection
* Simulink implementation
* Hardware implementation using motors and rotary encoders

## Reference

The mathematical formulation and control architecture are based on:

**"Modeling and PID Control of a Two-Link Two-DOF Robot Manipulator"**

The project document defines the nonlinear dynamic model, decoupling scheme, PID control law, MIMO classification, hardware elements, assumptions, and control objectives.
