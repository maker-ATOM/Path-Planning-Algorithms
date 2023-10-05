# Path-Planner-with-ROS2

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#About">About</a></li>
    <li><a href="#Using-this-Project">Using this Project</a></li>
    <li><a href="#Description-of-Algorithms">Description of Algorithms</a></li>
    <li><a href="#Project-Status">Project Status</a></li>
    <li><a href="#Project-Upgradation">Project Upgradation</a></li>
  </ol>
</details>

## About

### Design aspects of system

Nodes, topic structure 

### Color Scheme of CostMap: 

-128 to -1 => RED to YELLOW
-1 => GREY
0 => BLACK
1 to 98 => BLUE to RED
99 => CYAN
100 => PINK
101 to 127 => GREEN

## Description of Algorithms

### Fundamentals

- Depth First Dearch
- Breadth First Search

### Grid-based search algorithms
- dijkstra
- greedy
- A*
- D*
- Uniform Cost Search

### Sampling-based search algorithms
- Rapidly-Exploring Random Trees RRT
- RRT*
Probabilistic Roadmap (PRM)

### Potential Field Algorithms

- Artificial Potential Field (APF)

### Cell Decomposition Algorithms:

- Voronoi Diagrams
- Visibility Graphs: 

## Optimal Control Algorithms:

- Trajectory Optimization
- Model Predictive Control (MPC)

### Search-Based Algorithms:

 -Probabilistic Graph Search
- Anytime Algorithm

### Sampling-Based Hybrid Methods:

- PRM* (Probabilistic Roadmap Star)
- Informed RRT*









## Using this Project

Move into your workspace's src folder
```
cd ~/ros2_ws/src
```
Clone the project
```
git clone
```
Build the project.
```
cd ~/ros2_ws && colcon build
```

Launch rviz2 and map_node using bringup launch file
```
ros2 launch pathplanners bringup.launch.py
```

To see individual algorithms in action, run individual scripts.
```
ros2 run pathplanners algorithm_name
```

