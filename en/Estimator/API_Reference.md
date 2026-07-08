# Estimator API Reference

!!! info "API Documentation"
    This document outlines the API signatures for the high-performance State Estimation modules.

## :material-cube-outline: `EKF`

```cpp
template <size_t Nx = 8, size_t Nu = 2>
class EKF;
```
Extended Kalman Filter optimized for hardware constraints. Avoids generating transposed matrices dynamically in memory, instead unrolling computations during covariance prediction via Zero-Copy SIMD SAXPY updates.

### Method: `predict`
```cpp
template <typename Model>
void predict(Model& model, const matrix::StaticVector<double, Nu>& u, double dt);
```
Predicts the next state given a dynamic system model and a control input `u`. It evaluates Jacobians natively using Automatic Differentiation caching them optimally via a column-major memory layout.

### Method: `update`
```cpp
void update(const matrix::StaticVector<double, Nx>& z);
```
Applies the measurement update incorporating a robust LDLT decomposition solver instead of fragile naive matrix inverses.

---

## :material-cube-outline: `HistoryBuffer`

```cpp
template <size_t Dim, size_t Capacity>
class HistoryBuffer;
```
A circular buffer tailored to cache past estimations and sensor data. It explicitly discards modulo (`%`) division, which is known to cause severe pipeline stalls on ARM Cortex units, substituting it with a fast 1-cycle conditional pointer shift.

### Method: `operator[]`
```cpp
inline const matrix::StaticVector<double, Dim>& operator[](size_t age) const;
```
Returns a history element by `age` (where `0` is the most recent point in time). The internal index translation mechanism resolves to branchless CPU instructions (CSEL).

---

## :material-cube-outline: `SparseMHE`

```cpp
template <size_t HE, typename PlantModel = Dynamics::RealTimeDynamicsModel, size_t Nx = 8, size_t Nu = 2, size_t Nz = 8>
class SparseMHE;
```
A sparse moving horizon estimator designed strictly for real-time applications. It compresses dimension space and binds an LDLT factorizer into a regularized cost-functional to guarantee robust non-linear tracking despite high amounts of sensor noise.

### Method: `solve`
```cpp
bool solve();
```
Executes the optimization routine to update the entire state history window. The Gauss-Jordan elimination has been excised, restricting execution times strictly to a tight bounded $O(N^2)$ execution loop under maximum 2 iterations.
