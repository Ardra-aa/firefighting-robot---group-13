# Firefighting Robot Simulation

## Overview
This project demonstrates a firefighting robot simulation using Python. The robot operates on a graph-based representation of an environment, where:
- **Nodes** represent specific locations (e.g., sections within forest fires, buildings in a neighborhood).
- **Edges** represent paths between these locations with costs influenced by factors such as distance, time, and danger level.

The robot dynamically extinguishes fires by navigating through the graph while avoiding dangerous paths that exceed a predefined danger threshold. It employs advanced pathfinding algorithms such as A* and Dijkstra to determine the safest and most efficient routes to its targets.

## Key Features
- **Graph-Based Environment**: The environment is modeled as a graph with nodes and weighted edges.
- **Dynamic Pathfinding**: The robot utilizes A* and Dijkstra's algorithms for optimal navigation.
- **Risk Management**: The robot avoids dangerous paths with danger levels exceeding the threshold.
- **Dynamic Updates**: The graph updates periodically to simulate real-time changes in fire intensity and priority.
- **Visualization**: A GUI built using Pygame displays the graph, robot movements, and fire locations.
- **Threaded Updates**: Real-time updates to the graph using Python's threading module.

## Applications
- Emergency response systems
- Industrial safety solutions
- Disaster recovery operations
- Smart building integrations

## Algorithms
### 1. A* Algorithm
A heuristic-based approach that ensures optimal navigation by balancing path cost and estimated distance to the target. It dynamically explores alternate routes to maintain safety.

### 2. Dijkstra's Algorithm
Provides a global perspective of all possible routes, enabling the robot to identify feasible alternatives when constraints arise during A* pathfinding.

## Installation
### Prerequisites
- Python 3.8+
- Pygame library

## How It Works
1. **Graph Initialization**: The environment is initialized with nodes (locations) and edges (paths).
2. **Danger Management**: Dangerous paths are flagged, ensuring the robot avoids them during navigation.
3. **Real-Time Updates**: Danger levels and priorities of nodes dynamically update to simulate real-world conditions.
4. **Robot Movement**: The robot identifies and follows the safest route to extinguish fires.
5. **Visualization**: The GUI displays the graph, robot, fires, and paths.

## File Descriptions
- `firefighting_robot.py`: Main script containing the simulation logic.
- `fire_emoji.png`: Fire icon used for visualization.

## Usage
1. **Simulation Start**:
   - The robot begins at a random node and moves to extinguish fires in order of priority and proximity.
2. **Pathfinding**:
   - The robot uses Dijkstra's or A* algorithm to navigate the graph safely.
3. **Dynamic Changes**:
   - Fire intensities and priorities update periodically, requiring the robot to adjust its path dynamically.

## Visualization
- **Nodes**: Represented as circles with fire icons if a fire is present.
- **Edges**: Represent paths between nodes, color-coded based on danger level.
  - Green: Safe
  - Red: Dangerous
  - Blue: Active path
- **Robot**: Represented as a blue circle, moves dynamically to extinguish fires.

## Acknowledgments
- **Pygame**: For GUI and visualization.
- **Graph Theory**: For the underlying data structures and algorithms.
- **AI Pathfinding Algorithms**: For dynamic and efficient navigation.


