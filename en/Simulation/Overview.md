# Simulation Module

!!! abstract "Overview"
    The `Simulation` module provides numerical integration tools for simulating and predicting the continuous-time states of the physical system (plant) over discrete time steps. It bridges the gap between the continuous dynamics $ \dot{x} = f(x, u) $ and the discrete formulation required by the NMPC and Estimator modules.

## :material-shield-check: Design Philosophy

1. **Zero-Allocation**: No heap allocations occur during integration steps. All state vectors and intermediate slope calculations ($\mathbf{k_1}, \dots, \mathbf{k_4}$) are instantiated on the stack using `StaticVector`.
2. **Deterministic Execution**: The use of fixed-step integration algorithms ensures that the execution time of the prediction horizon remains strictly constant, which is a critical requirement for real-time optimal control.
3. **Template Metaprogramming**: High performance is achieved by compiling the dynamic model directly into the integration loop without using function pointers or `std::function` wrappers.

## :material-file-tree: Core Components

- **`Integrator.hpp`**: Houses the Runge-Kutta 4th Order (RK4) integration logic and the `IntegratorEngine` interface.
