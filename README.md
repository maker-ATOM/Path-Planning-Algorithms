# Path-Planner-with-ROS2

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#About">About</a></li>
    <li><a href="#Description-of-Algorithms">Description of Algorithms</a></li>
    <li><a href="#Using-this-Project">Using this Project</a></li>
  </ol>
</details>

<p align="center">
	<b>PlayGround</b>
</p>
<p align="center">
	<img src="media/playground.png" width="663" height="727"/>
</p>



TO Do:
Service based - iNterseting we have sderive presetn
changer dx, dy list structure ifrst top left botto n=m then right
break while loop of dfs on find the point
Store previously visited ndoe and draw the path

refer for psudocode
https://www.youtube.com/watch?v=KiCBXu4P-2Y&list=PLDV1Zeh2NRsDGO4--qE8yH72HFL1Km93P&index=6



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
    and init and goal pose
    # Other data elements common to algorithms

    map data stored in 2d matrix for easy of understyanding on code

    instead of adj list trnasmitting driectly ref map via serivice which has all obstacle, init andf goal pose.
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

Executable list
- bfs

Replace these with <algorithm_name> to run the specific algorithm

