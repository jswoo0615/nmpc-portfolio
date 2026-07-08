# Python API Reference

!!! info "API Documentation"
    This document outlines the Python bindings exported from `nmpc_wrapper.cpp`. The compiled module is typically named `nmpc_core`.

---

## :material-language-python: `SparseNMPCWrapper`

The main class bridging Python and the C++ NMPC solver.

### `__init__`
```python
def __init__(self, ego_state_arr: np.ndarray, wp_x_arr: np.ndarray, wp_y_arr: np.ndarray, obstacle_arr: np.ndarray)
```
Initializes the wrapper and memory pointers.
- **`ego_state_arr`**: Array containing the vehicle state `[x, y, yaw, vx, vy, yaw_rate]`.
- **`wp_x_arr`, `wp_y_arr`**: Arrays containing the global coordinates of the reference waypoints.
- **`obstacle_arr`**: Flattened array containing dynamic obstacle data `[s, d, r, vs, vd]` per obstacle.

### `set_target_speed`
```python
def set_target_speed(self, speed: float) -> None
```
Updates the target cruising speed (`config.target_vx`) for the controller.

### `update_config`
```python
def update_config(self, opt_dict: dict) -> None
```
Dynamically updates the cost function weights and solver parameters at runtime. Supported keys include:
- `"dt"`: Discretization time step.
- `"Q_D"`, `"Q_mu"`, `"Q_Vx"`, `"Q_Vy"`: State tracking penalties.
- `"R_Steer"`, `"R_Accel"`: Control effort penalties.
- `"R_SteerRate"`, `"R_AccelRate"`: Control rate penalties.

### `solve`
```python
def solve(self, num_wp: int) -> tuple[dict, tuple[float, float]]
```
Executes a single step of the NMPC optimization.
1. Builds the `StaticCubicSpline2D` using the provided waypoints.
2. Projects the ego vehicle's global state onto the Frenet frame (s, d, $\mu$).
3. Updates the target trajectory curvature (`kappa_ref`) over the prediction horizon.
4. Solves the Non-linear Programming problem via the Interior Point Method (`solve_ipm`).
5. Returns a tuple containing diagnostic data and optimal controls.

**Returns:**
- `monitor_data` (dict): Contains solver metrics:
  - `"status"`: Completion status message.
  - `"kkt_error"`: Maximum KKT tolerance error.
  - `"sqp_iter"`: Number of SQP iterations executed.
  - `"min_slack"`, `"max_lambda"`: Diagnostic values for physical constraints.
- `u_opt` (tuple): Optimal control command `(acceleration, steering_angle)`.
