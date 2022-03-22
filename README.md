# Project 3: Implementation A* algorithm for a mobile Robot
## _ENPM661_
[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

## Authors
Rohit M Patil
Email ID: rpatil10@umd.edu

Anish Mitra
Email ID: amitra12@umd.edu

## Description
Implemention of A* Algorithm to find a path between start and end point on a given map for a robot (radius = user input; clearance = 10 mm). Checks the feasibility of all inputs/outputs (if user gives start and goal nodes that are in the obstacle space they will be informed by a message and they should try again). Code outputs an animation/video of optimal path generation between start and goal point on the map. It shows both the node exploration as well as the optimal path generated.

## Dependencies

| Plugin | 
| ------ |
| heapq | 
| numpy | 
| cv2 | 
| math | 

## Instructions to run
Clone or download the project repository and open the folder,
Open terminal in the project folder and type below command:
```bash
python a_star_algo.py
```
or
```bash
python3 a_star_algo.py
```
Terminal asks for user input for step size, radius, start and goal location with orientation:
Input step size as shown in below fashion, **step_size**
```sh
Enter the step size of the robot: 5
```
Input radius as shown in below fashion, **radius**
```sh
Enter the radius of the robot: 5
```
Input start location as shown in below fashion, **x-coordinate** _space_ **y-coordinate** _space_ **theta**
```sh
Enter start location point and orientation[x,y,theta](eg: if (10,10,30), then enter: 10 10 30): 10 10 60
```
Input goal location as shown in below fashion, **x-coordinate** _space_ **y-coordinate** _space_ **theta**
```sh
Enter goal location point and orientation[x,y,theta](eg: if (115,185,60), then enter: 350 220 60
```
## Output
**Test case 1:**
Step size: 5
Radius: 5
Start location: 10 10 60
Goal location: 350 220 60
Visualization video: [Test case 1](https://youtu.be/5aQ9GAvQfFs).
![Optimal_path_testcase1](/outputs/Optimal_path_testcase1.png?raw=true)

**Test case 2:**
Step size: 10
Radius: 5
Start location: 10 10 0
Goal location: 100 240 0
Visualization video: [Test case 2](https://youtu.be/zyopxe8h77s).
![Optimal_path_testcase2](/outputs/Optimal_path_testcase2.png?raw=true)