# Utils Module

!!! abstract "Overview"
    The `Utils` module contains auxiliary components and macros that support the NMPC infrastructure. It provides environment evaluation logic and platform-specific compilation directives.

## :material-shield-check: Design Philosophy

1. **Platform Portability**: By using abstraction macros (like `CUDAMacros.hpp`), the codebase ensures that the same mathematical engines can be compiled both for standard CPU execution and GPU-accelerated environments without modifying the core logic.
2. **Environment Integration**: The module encapsulates the logic for evaluating external constraints, such as dynamic obstacles, in a format that the optimization solver can readily consume.

## :material-file-tree: Core Components

- **`CUDAMacros.hpp`**: Defines the `CUDA_CALLABLE` macro to tag functions for NVCC compilation when targeting GPUs.
- **`EnvironmentEvaluator.hpp`**: Contains the logic to model and evaluate moving obstacles within the Frenet frame, providing constraint values and Jacobians to the NMPC solver.
