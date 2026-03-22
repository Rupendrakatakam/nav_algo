# Custom Navigation Stack: A* & DWA with Dynamic Obstacle Avoidance

This repository implements a lightweight, fully custom **2D Navigation Stack** built entirely from scratch in C++. It features a hybrid architecture combining global pathfinding with highly reactive local obstacle avoidance.

## Architecture Highlights
The navigation system is split into three core interconnected algorithms:

1. **[A* Global Planner](docs/astar_global_planner.md):** Rapidly finds the optimal path around static mapped geometry.
2. **[Dynamic Window Approach (DWA) Local Planner](docs/dwa_local_planner.md):** A reactive forward-simulation controller that heavily scores trajectory endpoints to compute optimal physical $(v, \omega)$ velocity commands while smoothly tracing the A* path via pure pursuit lookaheads.
3. **[Dynamic 2D Costmap](docs/costmap.md):** Converts static environments and live sensor hits (e.g. moving humans) into 2-dimensional continuous cost fields (0.0 to 100.0) with custom radial inflation, ensuring the robot yields long before a physical collision can occur.

## Getting Started

### Prerequisites
* **C++ Compiler:** Supporting C++11 or newer (e.g., `g++` or `clang++`).
* **Python 3.x:** For running the visualization suite.
  * Required packages: `pandas`, `matplotlib`

### 1. Compile the Simulation Engine
The core mathematical simulation is written in C++ for maximum execution speed.

```bash
g++ astar.cpp -o nav_sim
```

### 2. Run the God Mode Simulation
The simulation will test the navigation stack against a predefined map with a rapidly moving human obstacle blocking a narrow corridor. The robot will recognize the highly-lethal dynamic obstacle, pause or safely navigate via an alternate route, and reach the goal.

```bash
./nav_sim
```
*This command outputs the generated trajectories and sensor logs to `sim_log.csv`.*

### 3. Run the Visualizer
Execute the Python script to animate the robot's decisions visually:

```bash
python3 visualization.py
```
*The script will read the generated logs and showcase the robot seamlessly dodging the human pedestrian on its way to the green goal!*

## Advanced Configuration
Tuning the fear array! The `calculate_dwa()` function in `astar.cpp` accepts a cost equation with several highly sensitive coefficients:

```cpp
float cost = 1.0 * dist_to_carrot 
           + 100.0 * obstacle_penalty 
           + 1.0 * heading_alignment 
           - 2.0 * speed;
```
Try modifying the `100.0` penalization parameter up or down if you wish to see the robot act more aggressively inside tight spaces!
