# Solver Mathematical Foundations

!!! abstract "Overview"
    This document outlines the core optimization algorithms used to solve the Linear Quadratic Regulator (LQR) subproblem derived from the NMPC Newton steps, and the mathematical validation of convergence.

---

## 1. Discrete-time Riccati Recursion

To solve the sparse equality-constrained Quadratic Programming (QP) problem efficiently across a prediction horizon $H$, the NMPC utilizes **Dynamic Programming** via the Riccati Equation. This reduces the time complexity from $\mathcal{O}(H^3)$ (for dense inversion) to $\mathcal{O}(H)$.

### 1.1 Problem Formulation

Given the linearized system dynamics:
$$ \delta x_{k+1} = A_k \delta x_k + B_k \delta u_k + d_k $$

And the quadratic cost function approximation:
$$ J = \frac{1}{2} \delta x_H^T Q_H \delta x_H + q_H^T \delta x_H + \sum_{k=0}^{H-1} \left( \frac{1}{2} \delta x_k^T Q_k \delta x_k + q_k^T \delta x_k + \frac{1}{2} \delta u_k^T R_k \delta u_k + r_k^T \delta u_k \right) $$

We define the Optimal Value Function (Cost-to-Go) at step $k$:
$$ V_k(\delta x) = \frac{1}{2} \delta x^T P_k \delta x + p_k^T \delta x $$

### 1.2 Backward Pass (Value Iteration)

Starting from the terminal state $P_H = Q_H$ and $p_H = q_H$, we recurse backward ($k = H-1 \dots 0$) to calculate the optimal feedback policy:

1. **Control Hessian & Gradient**:
   $$ Q_{uu} = R_k + B_k^T P_{k+1} B_k + \lambda_{LM} I $$
   $$ Q_{ux} = B_k^T P_{k+1} A_k $$
   $$ q_u = r_k + B_k^T (p_{k+1} + P_{k+1} d_k) $$

   *(Note: $\lambda_{LM}$ is the Levenberg-Marquardt regularization factor to guarantee positive definiteness).*

2. **Optimal Policy Gains**:
   We solve for the Feedback ($K_k$) and Feedforward ($k_{ff, k}$) gains using LDLT Decomposition on $Q_{uu}$:
   $$ K_k = -Q_{uu}^{-1} Q_{ux} $$
   $$ k_{ff, k} = -Q_{uu}^{-1} q_u $$

3. **Value Function Update**:
   $$ P_k = Q_k + A_k^T P_{k+1} A_k + K_k^T Q_{ux} $$
   $$ p_k = q_k + A_k^T (p_{k+1} + P_{k+1} d_k) + Q_{ux}^T k_{ff, k} $$

### 1.3 Forward Pass (Trajectory Simulation)

Once the gains are computed, we simulate the optimal trajectory forward from the initial state $\delta x_0 = 0$:

$$ \delta u_k = K_k \delta x_k + k_{ff, k} $$
$$ \delta x_{k+1} = A_k \delta x_k + B_k \delta u_k + d_k $$

---

## 2. KKT Convergence Metrics

To verify if the non-linear optimization has successfully converged, the `KKTMonitor` checks the four Karush-Kuhn-Tucker conditions. A solution is optimal if all $L_\infty$ norms are below a `TOLERANCE` $\tau$ (e.g., $10^{-6}$).

1. **Stationarity (Gradient of Lagrangian is zero)**:
   $$ \| \nabla L(x, u, \lambda, \mu) \|_\infty \le \tau $$

2. **Primal Feasibility (Constraints are satisfied)**:
   $$ \| x_{k+1} - f(x_k, u_k) \|_\infty \le \tau $$

3. **Dual Feasibility (Inequality multipliers are non-negative)**:
   $$ \max(0, -\mu) \le \tau $$

4. **Complementary Slackness (Active constraint alignment)**:
   In the Interior Point Method, this is modified by the barrier parameter $\mu_{target}$:
   $$ \| S \mu - \mu_{target} \mathbf{1} \|_\infty \le \tau $$
   *(Where $S$ is the diagonal matrix of slack variables).*
