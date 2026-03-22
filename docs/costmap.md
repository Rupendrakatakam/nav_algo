# Costmap System

## Overview
A critical component of this project is our **Dynamic Costmap 2D**. Path planners (both global A* and local DWA) don't understand abstract geometric shapes directly; they read 2D discrete numerical arrays ranging from 0.0 (safe) to 100.0 (lethal).

## Static Costmap
The Static Costmap represents the permanent architectural features of the environment (e.g. solid walls), calculated at initialization.
* We read a binary occupancy grid ($1$ for obstacle, $0$ for free space).
* A queue-based **Breadth-First Search (BFS)** computes the discrete Euclidean distance from all free space cells to the nearest static obstacle.

## Dynamic Costmap Update (`live_map`)
This represents transient obstacles that sensor streams (like 2D Lidar hits or tracked humans) pick up in real-time. In this simulation, dynamic tracking consists of updating the human's location linearly.
* During the primary loop, `update_live_sensors()` injects the human's current $(x, y)$ coordinate into a clean clone of the `static_costmap`.
* We perform another BFS diffusion strictly bounded within an `inflation_radius` outward from the human's newly sensed position.
* We overlay this dynamic cost footprint directly on top of the static map. The highest cost prevails (`max` operation).

## Obstacle Inflation
Point obstacles must be artificially padded or "inflated". Otherwise, navigation algorithms would calculate paths that safely graze an obstacle by a fraction of a millimeter—guaranteeing physical collision for any non-point robot.
1. **Lethal Radius (`robot_radius` = $1.2m$):** Any cell physically less than the robot's extended boundary from an obstacle is marked outright with a $100.0$ cost. No algorithms may traverse these zones.
2. **Inflation Decay:** For distances extending beyond the lethal radius up to the `inflation_radius` limit ($3.5m$), the cost decays exponentially: $Cost = 100 \cdot e^{-2(dist - lethal\_radius)}$, ensuring a gradual decline in assumed danger. The planners will naturally shy away from higher-cost cells unless avoiding a collision forces them into them.
