# RBE 550 - Basic Search Algorithms Implementation

## Overview

In this assignment, you are going to implement **BFS**, **DFS**, **Dijkstra** and **A*** algorithms with Python 3. These are the basic algorithms for discrete planning, which also share a lot of similarities. This template is provided to you as a starting point. After you finish coding, you would be able to create your own map to test different algorithms, and visualize the path found by them.

Files included:

- **search.py** is the file where you will implement your algorithms. As we will use a grader code to help us grade your assignments more efficiently. Please do not change the existing functions names.
- **main.py** is the scrip that provides helper functions that load the map from csv files and visualize the map and path. You are not required to modify anything but you are encouraged to read and understand the code.
- **map.csv** is the map file you could modify to create your own map.
- **test_map.csv** restores a test map for doc test purpose only. Do not modify this file.

Please finish reading all the instructions and rubrics below before starting actual coding.

## Get Started

Before starting any coding, please run the code first:

`python search.py`

When running **search.py** as a main function, it will run a doc test for all the algorithms. It loads **test_map.csv** as the map for testing.

As you haven't written anything yet, you would see you fail all the doc tests. After implementing each algorithm, you should run this file again and make sure to pass the doc tests (you will see nothing if you pass the test). 

But please be noted that, passing doc tests does not necessarily mean that your algorithm is done without any problems. It just shows that your algorithms are working in this simple **test_map.csv** with its given start and end position. (It may still fail, for example, if you change the goal to an obstacle.)

---

For visualization, please run:

`python main.py`

There should be 4 maps shown representing the results of 4 algorithms. As said before, there would be no path shown in the graph as you haven't implemented anything yet. The **main.py** loads the map file **map.csv**, which you are free to modify to create your own map.

## More details

- Please first read the algorithm description in **seach.py** and make sure you understand the input/arguments and required output/return of the algorithms.
- Keep in mind that, the coordinate system used here is **[row, col]**, which could be different from [x, y] in Cartesian coordinates. 
- For Dijkstra and A*, the cost to move each step is set to be 1. The heuristic for each node is its Manhattan distance to the goal.
- When you explore the nearby nodes in the map, please follow this order **"right, down, left, up"**, which means "[0, +1], [+1, 0], [0, -1], [-1, 0]" in coordinates. There is nothing wrong using other exploring orders. It is just that the test result was gotten by algorithms using this order. A different order may result in different results, which could let you fail the test and the grader code.
- Also, for the output of the function, steps and path should include and count the first / start position and the goal / end position. (We assume that the start and goal won't be the same.)

Until now, I hope you have a basic understanding of the template code and the requirements. After you go through the Rubrics below, you may start coding! 

Run your code with `python main.py`. You could use the map **map.csv** I provided, but I encourage you to create your own map. Run the test and see the paths you found and the steps/time it takes for these algorithms to find them. Try to briefly explain the reason why different algorithm gives different or same result.

## A* Test Case 1
![a](https://github.com/cskate1997/BFS_DFS_Dijkstra_A-_MotionPlanningAlgorithms/assets/94412831/72d5765d-467b-4904-9bb1-da7d885ccc01)

## A* Test Case 2
![as](https://github.com/cskate1997/BFS_DFS_Dijkstra_A-_MotionPlanningAlgorithms/assets/94412831/efc67c46-b440-4063-bd0b-b25ce66df882)

## BFS Test Case 1
![b](https://github.com/cskate1997/BFS_DFS_Dijkstra_A-_MotionPlanningAlgorithms/assets/94412831/a997cdd0-347d-4c9a-b8f9-f47e5a9299d3)

## BFS Test Case 2
![bfs](https://github.com/cskate1997/BFS_DFS_Dijkstra_A-_MotionPlanningAlgorithms/assets/94412831/84943072-f179-4669-9409-736addc27907)

## DFS Test Case 1
![d](https://github.com/cskate1997/BFS_DFS_Dijkstra_A-_MotionPlanningAlgorithms/assets/94412831/68c40707-ef2a-4101-8853-27b9cdcf1203)

## DFS Test Case 2
![dfs](https://github.com/cskate1997/BFS_DFS_Dijkstra_A-_MotionPlanningAlgorithms/assets/94412831/ac7061d0-5d7e-42a1-8798-c83ba2423dcb)

## Dijakstra Test Case 1
![d](https://github.com/cskate1997/BFS_DFS_Dijkstra_A-_MotionPlanningAlgorithms/assets/94412831/814c6c6c-d5c1-4172-8307-f4645bbd764b)

## Dijakstra Test Case 2
![dj](https://github.com/cskate1997/BFS_DFS_Dijkstra_A-_MotionPlanningAlgorithms/assets/94412831/439195e2-c7d3-4a1f-a06e-41780bd7d624)










  - Reference paper and resources if any

  Include the documentation as a pdf file, or if you prefer, a md file.
