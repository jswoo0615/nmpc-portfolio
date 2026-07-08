# Matrix API Reference

!!! info "API Documentation"
    This document outlines the API signatures for the Core, Auto-Differentiation (AD), and Linear Algebra (Linalg) classes.

---

## :material-math-integral: 1. Core Module

### :material-cube-outline: `StaticMatrix`
```cpp
template <typename T, std::size_t Rows, std::size_t Cols>
class alignas(64) StaticMatrix;
```
The central memory container. Allocates data on the stack, fully aligned to 64 bytes to synchronize perfectly with CPU cache lines. Incorporates SIMD optimized constructors and assignment operators.

### :material-cube-outline: `StaticMatrixView`
```cpp
template <typename T, std::size_t Rows, std::size_t Cols>
class StaticMatrixView;
```
A lightweight, zero-copy abstraction mapping over raw data pointers, acting identically to a `StaticMatrix`. Essential for interfacing seamlessly with memory blocks owned by other structures without triggering copies.

---

## :material-math-integral: 2. Automatic Differentiation (AD)

### :material-cube-outline: `DualVec`
```cpp
template <typename T, std::size_t N> 
struct alignas(64) DualVec;
```
Multidimensional Dual Number structure used to propagate exact first-order derivatives. 
`v` stores the primal value, while the array `g[N]` holds the gradient vector. It exploits hardware horizontal operations and aligns memory boundaries to guarantee zero bottleneck instantiation.

---

## :material-math-integral: 3. Linear Algebra (Linalg)

The `linalg` namespace avoids abstract polymorphism. Instead, it provides static inline solvers optimized for small to medium scale matrices. These routines prioritize **In-place memory reuse** to forbid stack oscillation.

### :material-function: Cholesky Factorization
```cpp
template <typename MatL, typename VecB, typename VecX>
inline void cholesky_solver(const MatL& L, const VecB& b, VecX& x) noexcept;
```
Solves $A x = b$ where $A = L L^T$. Completely bypasses temporary allocations. The innermost loop replaces standard array dumps with platform-specific native horizontal additions (`_mm256_hadd_pd` / `vpaddq_f64`).

### :material-function: LDLT Factorization
```cpp
template <class MatType, class VecB, class VecX>
inline void LDLT_solve(const MatType& mat, const VecB& b, VecX& x) noexcept;
```
Used primarily when the Hessian contains near-singular elements or negative eigenvalues due to numerical defects, which would normally crash Cholesky. It includes protective guards against `NaN` penetration.

### :material-function: LU Factorization
```cpp
template <typename MatLU, typename VecB, typename VecX>
inline void lu_solve(const MatLU& LU, const std::array<int, MatLU::NumRows>& p, const VecB& b, VecX& x) noexcept;
```
Applies pivot permutations directly onto the output vector `x`, cutting the temporary $O(N)$ allocation entirely.

### :material-function: QR Factorization
```cpp
template <class MatType, class VecTau>
inline MathStatus QR_decompose_Householder(MatType& mat, VecTau& tau) noexcept;
```
Applies Householder reflections leveraging aggressive SIMD dot-product acceleration. Used for robust overdetermined system solutions.
