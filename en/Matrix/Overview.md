# Matrix Module

!!! abstract "Overview"
    The `Matrix` module serves as the lowest-level mathematical backbone for the NMPC framework. It is entirely custom-built to circumvent the overhead, memory fragmentation, and latency issues commonly associated with generalized libraries (e.g., Eigen or Armadillo) when deployed to deeply embedded, real-time targets like Jetson Nano.

## :material-shield-check: Design Philosophy

1. **Zero-Allocation (Stack Only)**: The entire library strictly avoids heap allocations. Memory layouts are fixed at compile time using templates, entirely eliminating `malloc` latency and ensuring deterministic Worst-Case Execution Time (WCET).
2. **SIMD & Register Optimization**: Operations spanning linear algebra, automatic differentiation, and memory transfers are manually unrolled or vectorized using platform-specific instructions (ARM NEON / Intel AVX2) with 64-byte aligned memory buffers (`alignas(64)`).
3. **No-Transpose Policy**: $O(N^2)$ memory copying associated with matrix transposes is actively suppressed. Instead, algorithms traverse memory column-wise or row-wise virtually.
4. **CUDA Ready**: All core data structures and primitive functions are decorated with `CUDA_CALLABLE` to support transparent execution inside GPU kernels.

## :material-file-tree: Directory Structure

- **`Core/`**: Contains the fundamental `StaticMatrix`, `StaticMatrixView`, and `MathTraits`. These classes implement SIMD copy operators and arithmetic primitives.
- **`AD/`**: Contains the Automatic Differentiation engine (`DualScalar`, `DualVec`). It propagates gradients via dual numbers concurrently with primal evaluation, eliminating the need for finite differences.
- **`Linalg/`**: The linear algebra processing unit. Features highly tailored in-place factorizations (Cholesky, LU, LDLT, QR) written with native SIMD horizontal addition.
