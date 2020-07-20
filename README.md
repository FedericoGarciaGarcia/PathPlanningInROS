# Path Planning In ROS

Implementation of the A\* algorithm and a variant of the Block A\* in ROS.

### Block A\* Variant

Preprocessing is used to find fixed-size blocks of adjacent nodes free of obstacles. Only the central node of each block is stored in a list. The list will contain nodes to consider for path planning. 

The process is repeated several times, reducing the block size by half each time until its size is 2.

The *objective node* is also added to the list.

### A\* VS Block A\* Variant

![Comparison](https://raw.githubusercontent.com/FedericoGarciaGarcia/PathPlanningInROS/master/PathPlanningInROS/Pictures/3_a.png)

Execution of **A\*** (left) and **Block A\* variant** (right) on the map *willow-pr2-5cm.world*

Expanded nodes are shown in blue for A\* and red for Block A\*, while path nodes are shown in green. Due to the low number of nodes expanded in the Block A\* algorithm, their size has been increased for better visualization.

Below is a table comparing the results of the execution of the image above.

| **Algorithm**     | **Time(s)** | **Expanded nodes** | **Distance** |
|-------------------|:-----------:|:------------------:|:------------:|
| A\*               | 117.8       | 28149              | 24.68        |
| Block A\* Variant | 0           | 473                | 29.48        |


Though the path found by Block A\* was 19.49% longer than that of A\*, Block A\* reduced the number of expanded nodes by 98.32% an executed instantly. Since time was rounded to one decimal place and the execution time of Block A\* variant was just a few milliseconds, its time is shown as 0.

### In action

![Title](https://raw.githubusercontent.com/FedericoGarciaGarcia/PathPlanningInROS/master/PathPlanningInROS/Pictures/3_ba256.png)