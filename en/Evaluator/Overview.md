# Evaluator Module

!!! abstract "Overview"
    This module provides evaluation engines required by the optimization solvers. 
    It focuses on generating deterministic and memory-safe reference paths and evaluating costs/constraints in real-time.

## :material-shield-check: Design Philosophy

1. **Zero-Allocation**: Dynamic memory allocation (such as `new` or `malloc`) is completely removed to guarantee deterministic execution times within optimization loops. It heavily utilizes stack-allocated structures like `std::array`.
2. **Predictable Complexity**: Data structures like spline interpolators pre-allocate maximum allowed points (`MaxPoints`) to maintain an $\mathcal{O}(1)$ or deterministic complexity limit.
3. **Numerical Safety**: All geometric evaluators (like curvature) include zero-division guards to prevent optimization divergence when numerical limits are reached.

## :material-file-tree: Core Components

- **`StaticCubicSpline1D`**: A 1D cubic spline interpolator based strictly on static memory.
- **`StaticCubicSpline2D`**: A 2D path generator tracking accumulated distance (Arc length).
