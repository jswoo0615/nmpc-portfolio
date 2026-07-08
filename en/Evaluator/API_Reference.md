# Evaluator API Reference

!!! info "API Documentation"
    This document outlines the API signatures for the reference evaluation and path generation module.

## :material-cube-outline: `StaticCubicSpline1D`

```cpp
template <std::size_t MaxPoints>
class StaticCubicSpline1D;
```
This class performs cubic spline coefficient calculation and interpolation for a single variable.

=== "Key Member Variables"
    - `std::array<double, MaxPoints> a, b, c, d`: Coefficients of each term in the spline polynomial.
    - `std::array<double, MaxPoints> x`: Independent variable array of reference points.
    - `std::size_t num_points`: The number of actual valid data points.

=== "Member Functions"
    | Function Name | Signature | Description |
    | :--- | :--- | :--- |
    | `build` | `void build(const std::array<double, MaxPoints>& x_in, const std::array<double, MaxPoints>& y_in, std::size_t n)` | Calculates spline coefficients by solving linear equations based on the input points. A minimum of 3 points is required for validation. |
    | `calc` | `double calc(double t) const` | Returns the interpolated spline value $S(t)$ at the specified point $t$. |
    | `calc_d1` | `double calc_d1(double t) const` | Returns the first derivative $S'(t)$ at the specified point $t$. |
    | `calc_d2` | `double calc_d2(double t) const` | Returns the second derivative $S''(t)$ at the specified point $t$. |
    | `search_index` | `std::size_t search_index(double t) const` | (`private`) Derives the index of the polynomial interval to which the input $t$ belongs using binary search. It has a search complexity of $\mathcal{O}(\log n)$. |

---

## :material-cube-outline: `StaticCubicSpline2D`

```cpp
template <std::size_t MaxPoints>
class StaticCubicSpline2D;
```
A 2D path spline interpolator that uses the accumulated distance $s$ as a parameter. It is used to generate a continuous reference state for trajectory tracking in vehicle dynamics-based control.

=== "Key Member Variables"
    - `StaticCubicSpline1D<MaxPoints> sx, sy`: Internal 1D spline objects that interpolate X and Y coordinates with respect to the parameter $s$.
    - `std::array<double, MaxPoints> s`: Array of calculated accumulated distances (Arc Length).
    - `std::size_t num_points`: The number of actual valid data points.

=== "Member Functions"
    | Function Name | Signature | Description |
    | :--- | :--- | :--- |
    | `build` | `void build(const std::array<double, MaxPoints>& x_in, const std::array<double, MaxPoints>& y_in, std::size_t n)` | Generates the parameter $s$ array by accumulating the Euclidean distance between adjacent points, and initializes the `sx` and `sy` splines based on this. |
    | `get_max_s` | `double get_max_s() const` | Returns the total accumulated length (maximum $s$ value) of the generated trajectory. Used to determine the termination condition. |
    | `calc_x` | `double calc_x(double t) const` | Returns the interpolated X coordinate at the accumulated distance $t$. |
    | `calc_y` | `double calc_y(double t) const` | Returns the interpolated Y coordinate at the accumulated distance $t$. |
    | `calc_yaw` | `double calc_yaw(double t) const` | Calculates and returns the trajectory heading (Yaw) at the accumulated distance $t$ using the $\text{arctan2}(y', x')$ operation. |
    | `calc_curvature` | `double calc_curvature(double t) const` | Calculates the curvature $\kappa$ at the accumulated distance $t$. To ensure computational stability, it clamps to $0.0$ if the denominator is less than `1e-6`. |
