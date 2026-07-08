# Simulation Mathematical Foundations

!!! abstract "Overview"
    This document outlines the numerical integration technique used to discretize the continuous-time physical models inside the NMPC framework.

---

## 1. The Discretization Problem

The vehicle or dynamic system (Plant) is defined by a continuous-time Ordinary Differential Equation (ODE):
$$ \dot{x}(t) = f(x(t), u(t)) $$

To implement digital control schemes like NMPC and Moving Horizon Estimation, this continuous model must be discretized into a sequence of steps defined by a constant time interval $\Delta t$:
$$ x_{k+1} = F(x_k, u_k) $$

While the simplest approach is Euler Integration ($x_{k+1} = x_k + f(x_k, u_k) \Delta t$), it accumulates massive truncation errors and leads to instability when dealing with highly non-linear dynamics such as the Pacjeka tire model. Thus, a more robust integrator is mandatory.

---

## 2. Runge-Kutta 4th Order (RK4)

The RK4 method is considered the standard for numerical integration due to its ideal balance between computational complexity and accuracy. It estimates the trajectory over the interval $\Delta t$ by calculating four different slopes ($\mathbf{k}_1, \dots, \mathbf{k}_4$) and computing their weighted average.

### 2.1 Slope Formulation

Given the current state $x_k$ and the control input $u_k$ (which is assumed constant over the interval $\Delta t$ due to the Zero-Order Hold property of digital controllers):

1. **First Slope ($\mathbf{k}_1$)**: The derivative at the beginning of the interval.
   $$ \mathbf{k}_1 = f(x_k, u_k) $$

2. **Second Slope ($\mathbf{k}_2$)**: The derivative at the midpoint of the interval, evaluated using $\mathbf{k}_1$.
   $$ \mathbf{k}_2 = f\left( x_k + \mathbf{k}_1 \frac{\Delta t}{2}, u_k \right) $$

3. **Third Slope ($\mathbf{k}_3$)**: A refined derivative at the midpoint, evaluated using $\mathbf{k}_2$.
   $$ \mathbf{k}_3 = f\left( x_k + \mathbf{k}_2 \frac{\Delta t}{2}, u_k \right) $$

4. **Fourth Slope ($\mathbf{k}_4$)**: The derivative at the end of the interval, evaluated using $\mathbf{k}_3$.
   $$ \mathbf{k}_4 = f(x_k + \mathbf{k}_3 \Delta t, u_k) $$

### 2.2 State Update

The final predicted state $x_{k+1}$ is computed by combining the four slopes. The midpoint slopes ($\mathbf{k}_2$ and $\mathbf{k}_3$) are given double weight as they represent the best estimate of the trajectory's curvature:

$$ x_{k+1} = x_k + \frac{\Delta t}{6} \Big( \mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4 \Big) $$

!!! tip "Error Characteristics"
    RK4 provides a local truncation error of $\mathcal{O}(\Delta t^5)$ and a global accumulated error of $\mathcal{O}(\Delta t^4)$. This ensures that NMPC predictions across the optimization horizon remain faithfully close to the real physics engine, securing control stability.
