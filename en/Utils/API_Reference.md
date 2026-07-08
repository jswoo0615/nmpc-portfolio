# Utils API Reference

!!! info "API Documentation"
    This document outlines the API for the auxiliary utilities used in the NMPC framework.

---

## :material-cube-outline: `CUDAMacros.hpp`

```cpp
#ifdef __CUDAACC__
    #define CUDA_CALLABLE __host__ __device__
#else
    #define CUDA_CALLABLE
#endif
```
A cross-platform compilation macro. When compiled with `nvcc`, it injects `__host__ __device__` into function signatures, allowing core NMPC structures (like `StaticMatrix`) to be utilized inside CUDA kernels.

---

## :material-cube-outline: `EnvironmentEvaluator`

```cpp
template <std::size_t Nx_active, std::size_t MaxObs = 10>
class EnvironmentEvaluator;
```
A dedicated evaluator for dynamic obstacle avoidance. It tracks obstacles in the Frenet coordinate system and calculates collision margins.

### Structures
```cpp
struct ObstacleFrenet {
    double s = 0.0;
    double d = 0.0;
    double r = 0.5;
    double vs = 0.0;
    double vd = 0.0;
};
```
Defines a dynamic obstacle with its longitudinal (`s`) and lateral (`d`) positions, radius (`r`), and velocities.

```cpp
template <std::size_t Nx_active>
struct ConstraintGradient {
    double c_val = 0.0;
    matrix::StaticVector<double, Nx_active> J_x;
    bool is_active = false;
};
```
The data structure passed to the solver. Contains the constraint violation value `c_val` and its Jacobian `J_x` with respect to the active states.

### Method: `evaluate_obstacles`
```cpp
std::array<ConstraintGradient<Nx_active>, MaxObs> evaluate_obstacles(double current_s, double current_d, double time_future) const;
```
Predicts the position of each obstacle at `time_future` based on their velocities. Calculates the Euclidean distance from the ego vehicle to the obstacle. If the obstacle is within the Region of Interest (ROI), it computes the constraint violation and its gradient. Excludes distant obstacles from the solver's computational burden by setting `is_active = false`.
