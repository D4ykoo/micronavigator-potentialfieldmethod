# Potential Fields Path Planning Algorithm
This repository provides a planning system using the potential field method. Made for CASE-M2 IR.

![](/home/dario/thi/micronavigator-potentialfieldmethod/ressources/local_minima_problem_scenario3_example.png)  
Fig 1: Showing a local minium in a bade case scenario
## Introduction

The potential fields path planning algorithm is a method for finding a path from a starting position to a goal position in an environment with obstacles. The algorithm uses two types of potentials: attractive potential and repulsive potential.

The attractive potential encourages the robot to move towards the goal position, while the repulsive potential pushes the robot away from obstacles. The sum of these potentials produces a resultant potential field, which guides the robot towards the goal while avoiding obstacles.

An evaluation and visualization is also provided. Please note that both had no persistence per default.

## Evaluation
There are 3 criteria evaluated for each scenario:
* path length
* travel time
* number of obstacles encountered.

## Visualization
Two plots for each scenario:
* 2D plot for the path of the robot from start to goal and obstacles
* 3D plot for the potential field - great for having a look at local minima

## How to run
Install all the required packages:
```bash
pip install -r requirements.txt
```

Now run the main file:
```bash
python main.py
```

## How do define custom scenarios?
In the [scenarios.py](scenarios.py) file are three examples. Define your own and provide to the ScenarioGenerator instance inside the [main.py](main.py) loop.

## Example Occupancy grid 
```
1 2 3 4 5   i
_________
0 1 0 0 0 | 1
0 1 0 0 0 | 2
0 0 1 0 0 | 3
0 0 0 0 0 | 4
0 0 0 0 0 | 5
```

## License

© 2024 [@D4ykoo](https://github.com/D4ykoo). All rights reserved.

This project is licensed under the [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html).




