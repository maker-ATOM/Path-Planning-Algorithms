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

<p align="center">
	<b>PlayGround</b>
</p>
<p align="center">
	<img src="media/playground.png" width="663" height="727"/>
</p>

## About

### Design aspects of system

**bringup.launch.py**

```python
Launches rviz2 and map_node
```

**map_node**

```python
Publishes 
    Map with obstacles
    PointStamped message containing initial and goal point
    # Other data elements common to algorithms

    init and goal pose represent by PointStamped ros msg
    published on topic so to visualize in rivz2
    map data stored in 2d matrix for easy of understyanding on code
    convert the 2d array with walls to adj lust
    adj list is tuple of x,y within a list of list
        so using custom interface defined in the packge custom interface within ros learning repo
        not using dict for ease of algorithm
    convert the map data to OccupancyGrid and fill in the init and goal pose for riviz
```

### Color Scheme of CostMap: 

```python
-128 to -1 => RED to YELLOW
-1 => GREY
0 => BLACK
1 to 98 => BLUE to RED
99 => CYAN
100 => PINK
101 to 127 => GREEN
```

## Description of Algorithms

### Fundamentals


<p align="center">
	<b>Breadth First Search</b>
</p>
<p align="center">
	<img src="/media/bfs.gif" width="556" height="564"/>
</p>
- Depth First Search

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
ros2 run pathplanners <algorithm_name>
```

