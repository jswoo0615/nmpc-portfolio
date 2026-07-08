# Solver Module

!!! abstract "Overview"
    The `Solver` module is the computational core where the constrained optimization problem of the NMPC is resolved. It contains the numerical algorithms (like Riccati Recursion) that search for the optimal sequence of control inputs.

## :material-shield-check: Design Philosophy

1. **Deterministic Resolution**: The optimal control sequence must be calculated within strict real-time deadlines (e.g., < 10ms for vehicles). All algorithms here are strictly bounded (e.g., maximum iterations limit, line search limit).
2. **Zero-Allocation Data Paths**: The Riccati recursion processes the trajectory backwards and forwards entirely through stack memory structures defined in `include/Optimization/Matrix/Core`.
3. **Safety & Fallback First**: Optimal control theory provides theoretical guarantees, but real-world physics breaks them. The solver incorporates a `KKTMonitor` and `FallbackControl` to instantly inject fail-safe logic (e.g., emergency braking) when it detects numerical collapse or infeasibility.

## :material-file-tree: Core Components

- **`RiccatiSolver.hpp`**: The $\mathcal{O}(N)$ dynamic programming engine that solves the Linear Quadratic Regulator (LQR) subproblem in the Newton steps.
- **`KKTMonitor.hpp`**: Uses SIMD acceleration to calculate infinity norms of Karush-Kuhn-Tucker residuals to determine convergence or detect dual variable explosion.
- **`MeritLineSearch.hpp`**: An Armijo backtracking line search to prevent the Newton step from overshooting in highly non-linear regimes.
- **`FallbackControl.hpp`**: Acts as the ultimate safety net, catching infeasible `SolverStatus` signals and overriding the control outputs.
- **`SolverStatus.hpp`**: Defines the absolute state machine enumerations (e.g., `SUCCESS`, `MATH_ERROR`, `INFEASIBLE`).
