# Simulation API Reference

!!! info "API Documentation"
    This document outlines the API signatures for the numerical integration module.

---

## :material-cube-outline: `step_rk4`

```cpp
template <size_t Nx, size_t Nu, typename Model, typename T>
inline matrix::StaticVector<T, Nx> step_rk4(const Model& model, 
                                            const matrix::StaticVector<T, Nx>& x, 
                                            const matrix::StaticVector<T, Nu>& u, 
                                            double dt_double);
```
Performs a single, discrete integration step over the interval `dt_double` using the classic Runge-Kutta 4th Order method. It evaluates the non-linear continuous model four times to guarantee highly accurate predictions of the future state. 

All intermediate vector arrays (`k1` through `k4`) are instantiated safely on the stack.

---

## :material-cube-outline: `IntegratorEngine`

```cpp
template <size_t Nx, size_t Nu, typename Model, typename T>
struct IntegratorEngine;
```
A static wrapper acting as the primary interface for simulating vehicle dynamics. It binds the concrete RK4 step logic under a unified `compute` method.

### Method: `compute`
```cpp
static inline matrix::StaticVector<T, Nx> compute(const Model& model, 
                                                  const matrix::StaticVector<T, Nx>& x, 
                                                  const matrix::StaticVector<T, Nu>& u, 
                                                  double dt);
```
Executes the numeric integration. This static method is called by the MHE predictor, the NMPC dynamic constraint evaluation, and the standalone simulation loop.
