# Path Planning In ROS

Implementation of the A\* algorithm and a variant of the Block A\* in ROS.

### A\* VS Block A\* Variant

![Comparison](https://raw.githubusercontent.com/FedericoGarciaGarcia/PathPlanningInROS/master/PathPlanningInROS/Pictures/3_a.png)

Execution of the **A\* (left)** and **Block A\* variant** (right) on the map *willow-pr2-5cm.world*.

Expanded nodes are shown in blue for A\* and red for Block A\*, while path nodes are shown in green. Due to the low number of nodes expanded in the Block A\* algorithm, their size has been increased for better visualization.

Below is a table compraing the results of the exectuion of the image above.

| Algorithm         |  Time(s)      |  Expanded nodes | Distance |
|-------------------|:-------------:|:---------------:|:--------:|
| A\*               | 117.8         | 28149           | 24.68    |
| Block A\* Variant | 0             | 473             | 29.48    |

### In action

![Title](https://raw.githubusercontent.com/FedericoGarciaGarcia/PathPlanningInROS/master/PathPlanningInROS/Pictures/3_ba256.png)