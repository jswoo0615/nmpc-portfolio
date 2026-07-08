# Solver API Reference

!!! info "API Documentation"
    This document provides the API surface for the optimization algorithms, safety monitoring, and mathematical resolution engines.

---

## :material-math-integral: 1. Riccati Solver

### :material-cube-outline: `RiccatiSolver`
```cpp
template <std::size_t H, std::size_t Nx, std::size_t Nu>
class RiccatiSolver;
```
The central engine that solves the discrete-time finite-horizon LQR problem $\mathcal{O}(H)$. It expects the user (or upper-level NMPC engine) to populate the linearized dynamic arrays (`A`, `B`, `d`) and quadratic cost arrays (`Q`, `R`, `q`, `r`) before invoking the solver.

### Method: `solve`
```cpp
SolverStatus solve(double reg_u = 1e-6, double reg_x = 0.0);
```
Executes the backward and forward passes.
- **`reg_u`**: Levenberg-Marquardt regularization factor added to the control Hessian $Q_{uu}$ to guarantee positive definiteness.
- **Returns**: A `SolverStatus` enum indicating whether the factorization succeeded or suffered a singularity (`MATH_ERROR`).

---

## :material-math-integral: 2. Search & Convergence

### :material-cube-outline: `MeritLineSearch`
```cpp
class MeritLineSearch {
    static constexpr double BETA = 0.5;
    static constexpr double C1 = 1e-4;
    static constexpr int MAX_ITER = 6;
    
    template <typename Evaluator>
    [[nodiscard]] static inline double run(Evaluator evaluator, double current_merit, double directional_derivative = 0.0);
};
```
Executes an Armijo backtracking line search. The `evaluator` lambda simulates the non-linear physics step at varying fractions (`alpha`) of the proposed Newton step until the sufficient decrease condition (`C1`) is met.

### :material-cube-outline: `KKTMonitor`
```cpp
template <std::size_t N_vars, std::size_t N_cons>
class KKTMonitor {
    static KKT_Metrics evaluate_IPM(const matrix::StaticVector<double, N_vars>& grad_L_res,
                                    const matrix::StaticVector<double, N_cons>& primal_res,
                                    const matrix::StaticVector<double, N_ineq>& s_ineq,
                                    const matrix::StaticVector<double, N_ineq>& lambda_ineq,
                                    double target_mu);
};
```
Extracts the infinity norms of the Karush-Kuhn-Tucker residuals using vectorized SIMD (AVX2/NEON) instructions.

---

## :material-math-integral: 3. Robustness & Fallback

### :material-cube-outline: `FallbackControl`
```cpp
template <typename T>
inline FallbackTriggerState<T> evaluateKKTAndFallback(const KKTMonitorParams<T>& params, const SolverKKTState<T>& state);

template <typename T>
inline ControlOutput<T> applyFallbackStrategy(const ControlOutput<T>& nmpc_optimal_u, 
                                              const FallbackTriggerState<T>& trigger_state,
                                              const T safe_deceleration_cmd,
                                              const T pure_pursuit_delta);
```
Overrides the `nmpc_optimal_u` with safety maneuvers (`safe_deceleration_cmd` and `pure_pursuit_delta`) if the `evaluateKKTAndFallback` detects massive slack variable violations or dual variable explosions.
