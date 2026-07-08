# Estimator Module

!!! abstract "Overview"
    This module contains highly optimized state estimation algorithms, including an Extended Kalman Filter (EKF) and a Sparse Moving Horizon Estimator (MHE). These estimators are explicitly designed to overcome embedded hardware limitations, ensuring zero-allocation overhead and deterministic execution times.

## :material-shield-check: Design Philosophy

1. **Zero-Allocation**: No dynamic memory allocations (`new` or `malloc`) during the estimation loop.
2. **ALU Optimization**: Advanced techniques like avoiding modulo (`%`) operations and leveraging conditional selects (CSEL) to prevent branch prediction misses on ARM processors.
3. **SIMD & Register Level Operations**: Matrix inversions in the MHE and covariance propagations in the EKF bypass $O(N^3)$ operations by replacing them with LDLT decomposition pipelines and physical loop unrolling.
4. **Predictability**: Strict upper bounds on optimization iterations (e.g., `MAX_ITER = 2` in MHE) to guarantee hard real-time execution limits.

## :material-file-tree: Directory Structure

- `EKF.hpp`: High-speed Extended Kalman Filter avoiding scalar inversion and using Zero-Copy SIMD.
- `HistoryBuffer.hpp`: A circular buffer guaranteeing $O(1)$ SIMD copy without modulo operators.
- `SparseMHE.hpp`: A lightweight sparse Moving Horizon Estimator featuring SIMD optimizations and LDLT factorizations.
