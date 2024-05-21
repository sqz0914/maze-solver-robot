#ifndef __MAZE_GENERATOR_AGENT__H
#define __MAZE_GENERATOR_AGENT__H 

#include "enviro.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <random>
#include <numeric>
#include <queue>
#include <functional>

using namespace enviro;

// Constants for maze dimensions and wall size.
const int MAZE_WIDTH = 15; // Specifies the width of the maze in terms of the number of cells.
const int MAZE_HEIGHT = 15; // Specifies the height of the maze in terms of the number of cells.
const double WALL_SIZE = 40.0; // Defines the size of the wall in the maze. This could refer to either the thickness of the wall or the length of the wall segments.
const double MAZE_START_X = -MAZE_WIDTH * WALL_SIZE / 2.0 + WALL_SIZE / 2.0; // Determines the starting X-coordinate of the maze. It is calculated to center the maze in the available space.
const double MAZE_START_Y = -MAZE_HEIGHT * WALL_SIZE / 2.0 + WALL_SIZE / 2.0; // Determines the starting Y-coordinate of the maze. It is calculated to center the maze in the available space.
const double MAZE_CELL_SIZE = WALL_SIZE; // Represents the size of each cell in the maze, assumed here to be equal to the wall size.
const double EPSILON = 1e-3; // A small value used to handle floating-point arithmetic comparisons, especially for values that are meant to be close to zero.

// Represents a single cell in the maze.
class MazeCell {
    public:
    bool topWall = true;
    bool rightWall = true;
    bool bottomWall = true;
    bool leftWall = true;
    int row, col;

    MazeCell() : row(0), col(0) {}
    MazeCell(int r, int c) : row(r), col(c) {}
};

// Represents a cell as x-y coordinates in the maze.
class Point {
    public:
    int x, y;
    Point(): x(0), y(0) {}
    Point(int x, int y) : x(x), y(y) {}

    // Operators for equality checks.
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
};

// Specialization of std::hash for Point.
namespace std {
    template <>
    struct hash<Point> {
        size_t operator()(const Point& p) const {
            // Compute individual hash values for two components and combine them using XOR and bit shifting.
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
};


class MazeGeneratorController : public Process, public AgentInterface {
    public:
    MazeGeneratorController() : Process(), AgentInterface() {}

    // Initialize the maze with walls and generate the maze.
    void init() {
        initializeMaze();
        generateMaze();
        addExits();
        placeWalls();
    }

    // Generate path and emit it as an event.
    void start() {
        Point start(MAZE_HEIGHT / 2, MAZE_WIDTH / 2);
        std::vector<Point> path = bfs(start);
        json pathData = generatePathData(path);
        emit(Event("path_found", pathData));
    }
    void update() {}
    void stop() {}
    

    private:
    std::unordered_map<int, std::unordered_set<MazeCell*>> sets; // Used in maze generation. Maps an integer ID to a set of MazeCells, representing groups of connected cells.
    int id; // A counter for generating unique IDs for each set of connected cells during maze generation.
    std::vector<std::vector<MazeCell>> maze; // A 2D vector representing the maze grid, where each element is a MazeCell.
    std::default_random_engine rng; // Random number generator used in the maze generation process for randomness.
    std::unordered_set<MazeCell*> exits; // A set containing pointers to cells designated as exits in the maze.

    // Reset the maze cells to their initial state.
    void initializeMaze() {
        maze = std::vector<std::vector<MazeCell>>(MAZE_HEIGHT, std::vector<MazeCell>(MAZE_WIDTH));
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            for (int x = 0; x < MAZE_WIDTH; ++x) {
                maze[y][x] = MazeCell(y, x);
            }
        }
        rng = std::default_random_engine(std::random_device{}()); // Reseed RNG.
    }

    // Generate the maze using Eller's algorithm randomly.
    void generateMaze() {
        id = 0;  // Reset id to its initial value.
        sets.clear();  // Clear the sets map.

        for (int row = 0; row < MAZE_HEIGHT; ++row) {
            if (row < MAZE_HEIGHT - 1) {
                for (int col = 0; col < MAZE_WIDTH; ++col) {
                    if (findSet(maze[row][col]) == -1) {
                        sets[id].insert(&maze[row][col]);
                        ++id;
                    }
                }
                // Horizontal connections.
                for (int col = 0; col < MAZE_WIDTH - 1; ++col) {
                    MazeCell& cell = maze[row][col];
                    MazeCell& rightNeighbor = maze[row][col + 1];
                    if (findSet(cell) != findSet(rightNeighbor) && std::uniform_int_distribution<>(0, 1)(rng) == 0) {
                        cell.rightWall = false;
                        rightNeighbor.leftWall = false;
                        mergeSets(findSet(cell), findSet(rightNeighbor));
                    }
                }

                // Vertical connections.
                std::unordered_set<int> currRowSetIds;
                for (int col = 0; col < MAZE_WIDTH; ++col) {
                    currRowSetIds.insert(findSet(maze[row][col]));
                }
                for (int col = 0; col < MAZE_WIDTH; ++col) {
                    MazeCell& cell = maze[row][col];
                    int setId = findSet(cell);
                    // Ensure each set must have at least one vertical connection.
                    bool shouldLinkDown = std::uniform_int_distribution<>(0, 1)(rng) == 0 || currRowSetIds.find(setId) != currRowSetIds.end();
                    if (shouldLinkDown) {
                        MazeCell& downNeighbor = maze[row + 1][col];
                        cell.bottomWall = false;
                        downNeighbor.topWall = false;
                        sets[setId].insert(&downNeighbor);
                        if (currRowSetIds.find(setId) != currRowSetIds.end()) {
                            currRowSetIds.erase(setId);
                        }
                    }
                }
            }
            else {
                // Handle last row.
                for (int col = 0; col < MAZE_WIDTH; ++col) {
                    if (findSet(maze[row][col]) == -1) {
                        sets[id].insert(&maze[row][col]);
                        ++id;
                    }
                }
                // Horizontal connections.
                for (int col = 0; col < MAZE_WIDTH - 1; ++col) {
                    MazeCell& cell = maze[row][col];
                    MazeCell& rightNeighbor = maze[row][col + 1];
                    if (findSet(cell) != findSet(rightNeighbor)) {
                        cell.rightWall = false;
                        rightNeighbor.leftWall = false;
                        mergeSets(findSet(cell), findSet(rightNeighbor));
                    }
                }
            }
        }
    }

    // Add randomized exits to the maze borders with each side one exit.
    void addExits() {        
        // Collect border cells.
        std::unordered_map<int, std::vector<MazeCell*>> borderCells;
        collectBorderCells(borderCells);

        // Create one exit for each border.
        for (auto& [direction, cells] : borderCells) {
            std::uniform_int_distribution<int> dist(0, cells.size() - 1);
            int exitIndex = dist(rng);
            MazeCell* exitCell = cells[exitIndex];
            exits.insert(exitCell);

            // Determine which wall to remove based on the cell's position.
            if (exitCell->row == 0) exitCell->topWall = false;          // Top border
            else if (exitCell->row == MAZE_HEIGHT - 1) exitCell->bottomWall = false; // Bottom border
            else if (exitCell->col == 0) exitCell->leftWall = false;    // Left border
            else if (exitCell->col == MAZE_WIDTH - 1) exitCell->rightWall = false;  // Right border
        }
    }

    // Place walls in the maze.
    void placeWalls() {
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            for (int x = 0; x < MAZE_WIDTH; ++x) {
                double _x = MAZE_START_X + x * WALL_SIZE;
                double _y = MAZE_START_Y + y * WALL_SIZE;

                // Add horizontal walls at the top of the cell.
                if (maze[y][x].topWall) {
                    add_agent("HWall", _x, _y - WALL_SIZE / 2.0, 0, {{"fill", "gray"}, {"stroke", "None"}});
                }
                // Add vertical walls at the left side of the cell.
                if (maze[y][x].leftWall) {
                    add_agent("VWall", _x - WALL_SIZE / 2.0, _y, 0, {{"fill", "gray"}, {"stroke", "None"}});
                }
            }
        }

        // Add walls along the rightmost and bottommost edges.        
        for (int x = 0; x < MAZE_WIDTH; ++x) {
            double _x = MAZE_START_X + x * WALL_SIZE;
            if (maze[MAZE_HEIGHT - 1][x].bottomWall) {
                add_agent("HWall", _x, MAZE_START_Y + MAZE_HEIGHT * WALL_SIZE - WALL_SIZE / 2.0, 0, {{"fill", "gray"}, {"stroke", "None"}});
            }
        }
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            double _y = MAZE_START_Y + y * WALL_SIZE;
            if (maze[y][MAZE_WIDTH - 1].rightWall) {
                add_agent("VWall", MAZE_START_X + MAZE_WIDTH * WALL_SIZE - WALL_SIZE / 2.0, _y, 0, {{"fill", "gray"}, {"stroke", "None"}});
            }
        }
    }

    // Breadth-first search algorithm for path finding.
    // \param start The start point which is the center of the maze.
    // \return a shortest path from the start point to the closest exit.
    std::vector<Point> bfs(Point start) {
        std::queue<Point> q;
        std::unordered_map<Point, Point> came_from;
        std::vector<Point> path;

        q.push(start);
        came_from[start] = start; // Mark the start point.

        while (!q.empty()) {
            Point current = q.front();
            q.pop();

            // Check if the current cell is an exit.
            MazeCell* currentCell = &maze[current.y][current.x];
            if (exits.find(currentCell) != exits.end()) {
                // Reconstruct the path from start to the exit.
                Point at = current;
                while (at != start) {
                    path.push_back(at);
                    at = came_from[at];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            // Continue BFS.
            for (Point next : getNeighbors(current)) {
                if (came_from.find(next) == came_from.end()) {
                    q.push(next);
                    came_from[next] = current;
                }
            }
        }

        return path; // Return an empty path if no exit is found.
    }

    // Generate path data in JSON format.
    // \param path The shortest path founded.
    // \return a json data array that consists of x-y coordinates of the shortest path in the Enviro environment. 
    json generatePathData(const std::vector<Point>& path) {
        json pathData = json::array();
        // Translate the cell position to real x-y coordinates in Enviro environment.
        for (const Point& p : path) {
            double realX = MAZE_START_X  + p.x * MAZE_CELL_SIZE;
            double realY = MAZE_START_Y  + p.y * MAZE_CELL_SIZE;
            pathData.push_back({{"x", realX}, {"y", realY}});
        }

        // Handle the next cell after the exit so that the robot can go out of the maze.
        Point exit = path[path.size() - 1];
        double realX = MAZE_START_X  + exit.x * MAZE_CELL_SIZE;
        double realY = MAZE_START_Y  + exit.y * MAZE_CELL_SIZE;
        MazeCell exitCell = maze[exit.y][exit.x];
        if (exit.x == 0) { // exit cell at the left border
            if(exitCell.leftWall == false) {
                pathData.push_back({{"x", realX - MAZE_CELL_SIZE}, {"y", realY}});
            }
        }
        else if (exit.x == MAZE_WIDTH - 1) { // exit cell at the right border
            if(exitCell.rightWall == false) {
                pathData.push_back({{"x", realX + MAZE_CELL_SIZE}, {"y", realY}});
            }
        }
        else if (exit.y == 0) {
            if(exitCell.topWall == false) { // exit cell at the top border
                pathData.push_back({{"x", realX}, {"y", realY - MAZE_CELL_SIZE}});
            }
        }
        else if (exit.y == MAZE_HEIGHT - 1) {
            if(exitCell.bottomWall == false) { // exit cell at the bottom border
                pathData.push_back({{"x", realX}, {"y", realY + MAZE_CELL_SIZE}});
            }
        }

        return pathData;
    }

    /* Helper methods for maze generation and path finding */

    // Find the set a cell belongs to in the maze generation algorithm.
    // \param cell The maze cell.
    // \return the set id that the cell belongs to.
    int findSet(MazeCell& cell) {
        for (auto const& x : sets) {
            if (x.second.find(&cell) != x.second.end()) {
                return x.first;
            }
        }
        return -1;
    }

    // Merge two sets in the maze generation algorithm.
    // \param set1 & set2 The sets to be merged.
    void mergeSets(int set1, int set2) {
        if (sets.find(set1) != sets.end() && sets.find(set2) != sets.end()) {
            sets[set1].insert(sets[set2].begin(), sets[set2].end());
            sets.erase(set2);
        }
    }

    // Collect border cells for adding exits to the maze.
    // \param borderCells A map stores the maze cell on borders.
    void collectBorderCells(std::unordered_map<int, std::vector<MazeCell*>>& borderCells) {
        // Top and Bottom borders
        for (int col = 0; col < MAZE_WIDTH; ++col) {
            borderCells[0].push_back(&maze[0][col]);             // Top border
            borderCells[1].push_back(&maze[MAZE_HEIGHT - 1][col]); // Bottom border
        }

        // Left and Right borders
        for (int row = 1; row < MAZE_HEIGHT - 1; ++row) { // Skip corners already included in top and bottom.
            borderCells[2].push_back(&maze[row][0]);            // Left border
            borderCells[3].push_back(&maze[row][MAZE_WIDTH - 1]); // Right border
        }
    }

    // Get neighboring cells that can be moved to from a given cell.
    // \param p The maze cell that needs to find its neighbors.
    // \return a Point array that contains the cell's neighbors that are reachable.
    std::vector<Point> getNeighbors(Point p) {
        std::vector<Point> neighbors;

        // Directions in which the robot can move: up, down, left, right.
        std::vector<std::pair<int, int>> directions = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};

        for (auto &dir : directions) {
            int newX = p.x + dir.first;
            int newY = p.y + dir.second;

            // Check if the new position is within the maze bounds and not a wall.
            if (newX >= 0 && newX < MAZE_WIDTH && newY >= 0 && newY < MAZE_HEIGHT && isAccessible(p.x, p.y, newX, newY)) {
                neighbors.push_back(Point(newX, newY));
            }
        }

        return neighbors;
    }

    // Checks if movement from one cell to another is possible without encountering a wall.
    // \param x The x-coordinate (column) of the current cell.
    // \param y The y-coordinate (row) of the current cell.
    // \param newX The x-coordinate (column) of the neighboring cell to check.
    // \param newY The y-coordinate (row) of the neighboring cell to check.
    // \return a boolean value indicating if the neighboring cell is accessible from the current cell.
    bool isAccessible(int x, int y, int newX, int newY) {
        MazeCell cell = maze[y][x];
        MazeCell neighbor = maze[newY][newX];

        // Checking vertical movement (up or down).
        if(x == newX) {
            if(y > newY) {
                return cell.topWall == false && neighbor.bottomWall == false;
            }
            else {
                return cell.bottomWall == false && neighbor.topWall == false;
            }
        }
        // Checking horizontal movement (left or right).
        else if(x < newX) {
            return cell.rightWall == false && neighbor.leftWall == false;
        }
        else {
            return cell.leftWall == false && neighbor.rightWall == false;
        }
    }
};

class MazeGenerator : public Agent {
    public:
    MazeGenerator(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
    private:
    MazeGeneratorController c;
};

DECLARE_INTERFACE(MazeGenerator)

#endif