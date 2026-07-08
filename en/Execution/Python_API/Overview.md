# Python API Module (Wrapper)

!!! abstract "Overview"
    The `src/nmpc_wrapper.cpp` is a PyBind11 wrapper that exposes the highly optimized C++ NMPC core to Python environments. It acts as the bridge for simulations, machine learning (e.g., reinforcement learning agents), and high-level Python planners to access the real-time C++ solver.

## :material-shield-check: Design Philosophy

1. **Zero-Copy Arrays**: The wrapper uses raw pointers (`py::buffer_info`) to pass numpy arrays directly into the C++ memory space, avoiding expensive memory copies.
2. **State Projection Handling**: Manages the conversion of global Cartesian states (x, y, yaw) into the Frenet frame (s, d, $\mu$) required by the NMPC. It features a robust "Cold Start" detection mechanism for global search initialization.
3. **Transparent Status Reporting**: Exports not just the optimal control inputs, but also detailed KKT diagnostic metrics (slack variables, dual variables, iteration counts) back to Python via `py::dict` to enable continuous monitoring.

## :material-file-tree: Core Components

- **`SparseNMPCWrapper`**: The central PyBind11 class encapsulating the NMPC solver, tuning configurations, cubic spline evaluator, and the dynamic environment evaluator.
