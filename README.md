# 🎯 Optimal Control of a Cart-Pendulum System
This project models and stabilizes a cart-pendulum system where a cart supports two coupled inverted pendulums. The system is modeled in state-space and controlled using optimal techniques including LQR, LQG, and Luenberger observer-based feedback. The project also evaluates the system's controllability and observability, simulates dynamic response, and visualizes system performance over time.

All simulations and designs are implemented in MATLAB using both symbolic and numerical workflows.

## .

🧠 Core Concepts
* State-space modeling of 6-state system (position, velocities, and two pendulum angles)

* Linear Quadratic Regulator (LQR) control design for optimal stabilization

* Luenberger Observer design for partially observable systems

* Linear Quadratic Gaussian (LQG) for full-state estimation + optimal control

* Rank-based Controllability and Observability analysis

* Nonlinear simulation using ```ode45```

## 📐 System Model
* States:
```x, ẋ, θ₁, θ̇₁, θ₂, θ̇₂```

* Inputs:
Force u applied to the cart

* Dynamics:
Modeled with symbolic matrices using system masses and pendulum lengths

##  🔧 Simulation Highlights
✅ LQR Controller
* Stabilizes the cart and both pendulums

* Custom Q and R weighting matrices

* Shows step response of controlled system

* Displays closed-loop poles

✅ Luenberger Observers
* Designed for 4 different output vector sets

* Observers built using pole placement

* Simulated system response using initial() and step()

✅ LQG Controller
* Combines LQR controller with Kalman-based observer

* Uses lqr() for both control and estimation gain K and L

* Simulates full nonlinear system with observer-in-the-loop via ode45

* Plots state evolution over 600 seconds

✅ Controllability & Observability
* Verifies full system controllability using rank of [B, AB, ..., A⁵B]

* Tests observability for multiple sensor configurations (e.g., x, θ₁, θ₂)

* Outputs matrix ranks and observability status directly to console

## 📊 Sample Outputs
* Step Response of LQR controlled system

* Observer state estimation accuracy

* State trajectories over time

* Reconstructed states from LQG observer

* Eigenvalue locations of ```A - BK``` and ```A - LC```
