# A* Global Planner

## Overview
The A* (A-Star) Global Planner is responsible for generating a collision-free path from the robot's starting position to its goal, considering only the **static** obstacles in the environment. It provides a long-term "global" trajectory that the local planner (DWA) will attempt to follow.

## How it works in this project
Our A* implementation operates on a discrete grid where each cell corresponds to a specific area in the map. The planner uses an 8-connected grid search (horizontal, vertical, and diagonal movements) to find the shortest path.

### 1. Cost Evaluation (`g`, `h`, `f`)
We evaluate the suitability of adding a grid node to our path using the standard A* equation: 
$$f(n) = g(n) + h(n)$$

* **g-cost (Path Cost):** The accumulated distance to reach the current node. Orthogonal moves add `1.0`, while diagonal moves add `1.414`. We additionally penalize the $g$-cost if the node's static cost map value is high (but passable), naturally encouraging the robot to stay further away from walls when possible.
* **h-cost (Heuristic):** The estimated distance to the goal. We use the **Octile Distance** heuristic, which is perfect for an 8-connected grid as it accurately estimates orthogonal and diagonal distance combinations.

### 2. Node Expansion
The algorithm expands the node with the lowest $f$-cost from an open Priority Queue. It checks all 8 neighbors:
* **Lethal Check:** If a neighboring cell has a costmap value $\ge 100.0$ (meaning it is inside an obstacle or its inflation radius), it is skipped entirely.
* If a valid neighbor is found with a cheaper path than previously discovered, its parent pointer is updated, and it is pushed back into the Priority Queue.

### 3. Path Reconstruction
Once the goal is reached, we trace back the network of parent pointers from the goal node to the starting node, reverse the list, and produce the final `global_path`.

## Pure Pursuit Lookahead
To smoothly follow this global trajectory, our DWA local planner uses a Pure Pursuit strategy. Instead of rigidly aiming at a fixed index in the path, it continuously finds the closest point on the global path to the robot's *current* position, and searches a fixed number of steps ahead (our lookahead window) to place the "carrot". This ensures the robot constantly pulls itself forward along the path regardless of dynamic detours.
