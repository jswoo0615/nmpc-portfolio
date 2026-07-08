# Python Scripts Module

!!! abstract "Overview"
    The Python Scripts layer forms the operational front-end for the NMPC ecosystem. It provides the logic to connect the highly optimized C++ mathematical core to the CARLA autonomous driving simulator, facilitating robust trajectory generation, scenario analysis, and automated stress testing.

## :material-shield-check: Design Philosophy

1. **Simulation Integration**: Seamlessly connect the abstract physics models optimized in C++ to the complex 3D simulation environment of CARLA, ensuring that sensor inputs and control outputs match physical realities.
2. **Robust Testing & Validation**: Autonomous systems require extensive verification. Scripts like the `nmpc_stress_test.py` and `analyze_stress_results.py` automatically seek out edge cases (e.g., sharp intersections) and bombard the controller to prove mathematical stability (KKT convergence) and physical safety.
3. **Adaptive Curvature Tracking**: High-level local planners (`reference_generator.py`) provide not just static paths, but dynamic context (like lane-changes and tight corners) so the NMPC solver can alter its cost matrices dynamically on-the-fly.

## :material-file-tree: Core Scripts

- **`reference_generator.py`**: A path smoothing and adaptive penalty generator. Uses Laplacian smoothing for waypoint discontinuity mitigation and feeds adaptive Q and R weights to the solver based on immediate road curvature.
- **`nmpc_core_planner.py`**: The main interface loop. Consumes CARLA localization data, invokes the C++ `SparseNMPCWrapper`, handles fallback overrides, and dispatches steering/throttle commands back to CARLA.
- **`carla_route_inspector.py`**: A Pygame-based visual tool to inspect map spawn points, analyze topological junctions, and export custom routes to JSON.
- **`nmpc_carla_runner.py`**: A headless execution engine that runs the NMPC vehicle on specific exported JSON routes or random spawn points, logging performance metrics.
- **`nmpc_stress_test.py`**: A CI/CD-style automated testing suite. It hunts the map for the most dangerous intersections (e.g., > 60° turns) and launches the vehicle to test convergence reliability, isolating failure scenarios.
- **`analyze_stress_results.py`**: Parses the JSON outputs of the stress tests, categorizing failures (Timeout, Collision, Off-Route, KKT Solver Collapse) into a comprehensive final report.
