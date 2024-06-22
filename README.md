# Maze Solver Robot

## Overview
The Maze Solver Robot project is an engaging application designed to navigate a robot find exit out of a randomly generated maze. Utilizing Eller's algorithm, the maze is created to ensure solvability with four exits, one on each side. The core challenge tackled in this project is to find the shortest path from the starting point of the robot to any exit, and then to make a robot follow this path to exit the maze.

## Key Challenges and Solutions

### 1. Generating a Solvable Maze
- **Challenge**: Ensuring the generated maze is randomized yet solvable.
- **Solution**: Eller's algorithm was implemented to strike a balance between randomness and solvability. For more information about the Eller's algorithm, you may check [here](https://weblog.jamisbuck.org/2010/12/29/maze-generation-eller-s-algorithm).

### 2. Placing Walls for Maze Construction
- **Challenge**: Determining how to accurately place walls in the maze.
- **Solution**: The MazeCell class was utilized, encompassing variables to decide whether to place top, bottom, left, or right walls for each cell. Moreover, a static VWall agent was used to represent left or right walls, and a static HWall agent was used to represent top and bottom walls.

### 3. Wall Placement Positioning
- **Challenge**: Correctly positioning walls within the maze in the Enviro environment.
- **Solution**: By identifying the coordinates of the maze's leftmost corner cell, we could then accurately calculate other cells' positions using the cell width and wall width.

### 4. Finding the Shortest Path to an Exit
- **Challenge**: Identifying the shortest path from the robot starting point to any exit.
- **Solution**: Breadth-First Search (BFS) algorithm was applied. Once an exit cell was reached, the path was reconstructed in reverse order. We also add an extra point after the exit cell to make sure that the robot could go out of the maze.

### 5. Robot Path Following
- **Challenge**: Making the robot follow the calculated path.
- **Solution**: Initially, the plan was to use sensors to keep the robot follow the directions of the shortest path. However, this approach was not ideal. The final solution involved passing the path's coordinates directly to the robot, using the `move_toward` function for navigation.

## Installation and Running the Code

### Prerequisites
- Docker installed on your system.

### Steps
1. **Clone the Repository**:
   ```sh
   git clone https://github.com/sqz0914/maze-solver-robot.git
   ```
2. **Navigate to the project folder**:
    ```sh
    cd maze-solver-robot
    ```
3. **Run docker and mount the folder to docker**:
   ```sh
   docker run -p80:80 -p8765:8765 -v $PWD:/source -it klavins/enviro:v1.6 bash
   ```
4. **Run Enviro**:
   ```sh
   make
   esm start
   enviro
   ```
5. Open a web browser and navigate to `http://localhost` to see the project in action

## Using the Project
- On project launch, a maze will be generated with the robot positioned at the center.
- The robot will calculate and then follow the shortest path to the nearest exit.
- Run enviro again to generate a new maze and watch the robot solve it again.

## Acknowledgements
- This project was a practical application of maze-solving algorithms and robotics navigation, inspired by algorithmic concepts and real-world navigation challenges.
- Appreciation to the University of Washington [EEP 520](https://github.com/sosper30/eep520-wi24) course and  open-source [Enviro](https://github.com/klavinslab/enviro)  project for providing the tools and frameworks utilized in this project.
