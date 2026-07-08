# Estimator Mathematical Foundations

!!! abstract "Overview"
    This document presents the mathematical formulations underlying the `Estimator` module, specifically covering the Extended Kalman Filter (EKF) and the Sparse Moving Horizon Estimator (MHE).

---

## 1. Extended Kalman Filter (EKF)

The Extended Kalman Filter provides recursive state estimation for non-linear dynamic systems. The system and measurement models are defined as:

$$
\begin{aligned}
x_k &= f(x_{k-1}, u_k) + w_k, \quad w_k \sim \mathcal{N}(0, Q) \\
z_k &= h(x_k) + v_k, \quad v_k \sim \mathcal{N}(0, R)
\end{aligned}
$$

Where $x_k \in \mathbb{R}^{N_x}$ is the state, $u_k \in \mathbb{R}^{N_u}$ is the control input, and $z_k \in \mathbb{R}^{N_z}$ is the measurement. In our architecture, the full state is directly measurable, hence $h(x_k) = x_k$.

### 1.1 Prediction Step (Time Update)

The state estimate and covariance are propagated forward in time:

$$
\begin{aligned}
\hat{x}_{k|k-1} &= f(\hat{x}_{k-1|k-1}, u_k) \\
P_{k|k-1} &= F_k P_{k-1|k-1} F_k^T + Q
\end{aligned}
$$

Where the Jacobian matrix $F_k$ is exactly computed using Automatic Differentiation (AD):

$$
F_k = \left. \frac{\partial f(x, u)}{\partial x} \right|_{x=\hat{x}_{k-1|k-1}, u=u_k}
$$

!!! tip "Architect's Note: SIMD SAXPY Optimization"
    In `EKF.hpp`, the naive computation of $F P F^T$ requires allocating a temporary transposed matrix $F^T$. To eliminate this overhead, the engine performs a **Zero-Copy SIMD SAXPY** operation, computing the elements of $P_{k|k-1}$ by physically unrolling the scalar loops.

### 1.2 Update Step (Measurement Update)

When a new measurement $z_k$ arrives, the innovation (residual) is computed. Note that the yaw angle residual is normalized to $[-\pi, \pi]$:

$$
y_k = z_k - \hat{x}_{k|k-1}
$$

The innovation covariance $S_k$ and Kalman Gain $K_k$ are evaluated:

$$
\begin{aligned}
S_k &= P_{k|k-1} + R \\
K_k &= P_{k|k-1} S_k^{-1}
\end{aligned}
$$

Finally, the state and covariance are updated:

$$
\begin{aligned}
\hat{x}_{k|k} &= \hat{x}_{k|k-1} + K_k y_k \\
P_{k|k} &= (I - K_k) P_{k|k-1}
\end{aligned}
$$

To preserve numerical stability, $P_{k|k}$ is explicitly forced to be symmetric:

$$
P_{k|k} \leftarrow \frac{1}{2} (P_{k|k} + P_{k|k}^T)
$$

---

## 2. Sparse Moving Horizon Estimator (MHE)

Unlike EKF, which strictly uses the latest measurement, MHE formulates state estimation over a rolling window (horizon) $HE$ as an optimization problem. This handles severe non-linearities and bounded constraints significantly better.

### 2.1 MHE Cost Function

The objective is to find the sequence of states $X = \{ x_0, x_1, \dots, x_{HE} \}$ that minimizes the following cost functional:

$$
\min_{X} J(X) = J_{\text{arrival}} + J_{\text{meas}} + J_{\text{dyn}}
$$

Where:
1. **Arrival Cost**: Penalizes deviation from the prior state estimate $\bar{x}_0$ at the start of the horizon.
   $$ J_{\text{arrival}} = \frac{1}{2} \left\| W_{\text{arr}} (x_0 - \bar{x}_0) \right\|_2^2 $$

2. **Measurement Cost**: Penalizes deviation between the estimated states and the actual sensor measurements $z_k$.
   $$ J_{\text{meas}} = \frac{1}{2} \sum_{k=1}^{HE} \left\| W_{\text{meas}} (z_k - x_k) \right\|_2^2 $$

3. **Dynamic Feasibility Cost (Multiple Shooting)**: Penalizes violations of the vehicle dynamics $f(x, u)$.
   $$ J_{\text{dyn}} = \frac{1}{2} \sum_{k=0}^{HE-1} \left\| W_{\text{dyn}} \Big( x_{k+1} - f(x_k, u_k) \Big) \right\|_2^2 $$

### 2.2 Gauss-Newton Formulation

To solve this non-linear least-squares problem efficiently within real-time bounds, we apply a Gauss-Newton step. We define the dynamic residual $r_k$:

$$
r_k = x_{k+1} - f(x_k, u_k)
$$

The cost function is linearized using the system Jacobian $J_k = \frac{\partial f}{\partial x_k}$. The resulting Hessian $H$ and gradient $g$ are assembled block by block:

#### Gradient Accumulation
For the arrival node $x_0$:
$$ g_0 \mathrel{+}= W_{\text{arr}}^2 (x_0 - \bar{x}_0) $$

For the measurement nodes:
$$ g_k \mathrel{-}= W_{\text{meas}}^2 (z_k - x_k) $$

For the dynamic sequence, expanding the residuals yields contributions to node $x_k$ and $x_{k+1}$:
$$ 
\begin{aligned}
g_{k+1} &\mathrel{+}= W_{\text{dyn}}^2 r_k \\
g_k &\mathrel{-}= J_k^T W_{\text{dyn}}^2 r_k
\end{aligned}
$$

#### Hessian Accumulation
The Hessian is severely sparse (Block Tridiagonal). 
- **Main Diagonal Blocks**:
  $$
  \begin{aligned}
  H_{0,0} &\mathrel{+}= W_{\text{arr}}^2 \\
  H_{k,k} &\mathrel{+}= W_{\text{meas}}^2 \\
  H_{k+1,k+1} &\mathrel{+}= W_{\text{dyn}}^2 \\
  H_{k,k} &\mathrel{+}= J_k^T W_{\text{dyn}}^2 J_k
  \end{aligned}
  $$

- **Off-Diagonal Blocks (Cross Terms)**:
  $$
  \begin{aligned}
  H_{k+1,k} &\mathrel{-}= W_{\text{dyn}}^2 J_k \\
  H_{k,k+1} &\mathrel{-}= J_k^T W_{\text{dyn}}^2
  \end{aligned}
  $$

### 2.3 LDLT Factorization and Primal Update

Once $H$ and $g$ are fully assembled, a minimal Tikhonov regularization is applied:
$$ H \leftarrow H + \lambda I \quad (\lambda = 10^{-3}) $$

The descent direction $\Delta X$ is computed by solving the linear system:
$$ H \Delta X = -g $$

!!! tip "Architect's Note: Bypassing Matrix Inversion"
    The $O(N^3)$ Gauss-Jordan matrix inversion is highly detrimental to deterministic execution. `SparseMHE.hpp` utilizes an inline **LDLT Decomposition solver**, securing the pipeline and completely sidestepping singularities.

The states are then updated with a clamped step size to prevent numerical divergence:
$$ x_k \leftarrow x_k + \text{clamp}(\Delta x_k, -5.0, 5.0) $$
