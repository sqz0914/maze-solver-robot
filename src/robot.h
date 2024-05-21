#ifndef __MY_ROBOT_AGENT__H
#define __MY_ROBOT_AGENT__H 

#include "enviro.h"
#include <queue>

using namespace enviro;

const double MOVE_SPEED = 10; // Move speed of the robot

class MyRobotController : public Process, public AgentInterface {
    public:
    MyRobotController() : Process(), AgentInterface(), movingToGoal(false), targetX(0), targetY(0) {}

    // Watches for the 'path_found' event and queues the points from the event data.
    void init() {
        watch("path_found", [this](const Event& e) {
            auto pathData = e.value();
            for (auto& point : pathData) {
                double x = point["x"];
                double y = point["y"];
                // std::cout << x << ", " << y << std::endl;
                path.push({x, y});
            }
            // If there are points in the path and the robot isn't already moving, start moving to the next point.
            if (!path.empty() && !movingToGoal) {
                moveToNextPoint();
            }
        });
    }
    void start() {}

    // The update method is called at each tick of the simulation.
    void update() {
        // If the robot is moving towards a goal, continue moving towards the current target point.
        if (movingToGoal) {
            // Calculate the distance to the target point.
            double distance = sqrt(pow(targetX - position().x, 2) + pow(targetY - position().y, 2));
            // If the robot is not yet close to the target, continue moving towards it.
            if (distance > MOVE_SPEED / 2) {
                move_toward(targetX, targetY, MOVE_SPEED);
            } else {
                // If close enough, consider the point reached, pop it from the queue, and proceed to the next point if available.
                path.pop();
                if (!path.empty()) {
                    moveToNextPoint();
                } else {
                    movingToGoal = false; // All points reached
                    track_velocity(0, 0); // Stop the robot
                }
            }
        }
    }
    void stop() {}

    private:
    std::queue<std::pair<double, double>> path; // Queue of coordinates the robot must travel to.
    bool movingToGoal; // Indicates whether the robot is currently moving towards a goal.
    double targetX, targetY; // Coordinates of the current target point.

    // Sets the next point in the path as the current target and starts moving towards it.
    void moveToNextPoint() {
        if (!path.empty()) {
            std::pair<double, double> p = path.front();
            targetX = p.first;
            targetY = p.second;
            movingToGoal = true;
        }
    }
};

class MyRobot : public Agent {
    public:
    MyRobot(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
    private:
    MyRobotController c;
};

DECLARE_INTERFACE(MyRobot)

#endif
