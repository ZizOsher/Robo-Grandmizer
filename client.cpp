#include <iostream>
#include <libplayerc++/playerc++.h>
#include <opencv2/opencv.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <fstream>
#include "pathfinding.hpp"

using namespace boost;
using namespace PlayerCc;
using namespace cv;
using namespace Pathfinding;
namespace pf = Pathfinding;

void moveInDirection(const std::string& targetDirection, std::string& currentDirection, Position2dProxy& pp) {
    if (targetDirection == currentDirection) {
        pp.SetSpeed(0.7, 0); // Move forward
        return;
    }

    // Calculate rotation based on current and target directions
    double rotation = 0.0; // 0 means no rotation
    if ((currentDirection == "up" && targetDirection == "right") || 
        (currentDirection == "right" && targetDirection == "down") || 
        (currentDirection == "down" && targetDirection == "left") || 
        (currentDirection == "left" && targetDirection == "up")) {
        rotation = -0.8; // Clockwise
    } else {
        rotation = 0.8;  // Counter-clockwise
    }

    pp.SetSpeed(0, rotation);  // Start the rotation
    usleep(2000000);           // Wait for rotation to complete
    pp.SetSpeed(0, 0);         // Stop the rotation
    pp.SetSpeed(0.7, 0);       // Move forward

    currentDirection = targetDirection;
}


int main(int argc, char *argv[]) {
    Eigen::MatrixXd matrix = loadMatrix("../binary_image.txt");
    PlayerClient client("localhost");
    Position2dProxy pp(&client, 0);
    pp.SetMotorEnable(true);
    pf::Point currentLocation{91, 276};
    std::string direction = "right";
    int i = 0;
    while (i < 1) {
        std::cout << "CurrentLocation: " << currentLocation.x << ", " << currentLocation.y << std::endl;
        client.Read();
        pf::Point target;
        std::cout << "Enter the target coordinates (x y): ";
        std::cin >> target.x >> target.y;

        std::string dup_direction = direction;
        std::vector<pf::Point> path = astar(matrix, currentLocation, target, direction);
        std::cout << "Path: ";
        for (const pf::Point& point : path) {
            std::cout << point.toString() << ",";
        }
        std::cout << std::endl;
        
        direction = dup_direction;
        for (const pf::Point& point : path) {
            std::cout << "Direction: " << direction << std::endl;
            std::string targetDirection;
            if (point.x > currentLocation.x) 
                targetDirection = "right";
            else if (point.x < currentLocation.x) 
                targetDirection = "left";
            else if (point.y < currentLocation.y) 
                targetDirection = "up";
            else if (point.y > currentLocation.y) 
                targetDirection = "down";

            moveInDirection(targetDirection, direction, pp);            
            direction = targetDirection;
            currentLocation = point;
            usleep(180000);
            i++;
        }
        pp.SetSpeed(0, 0);
    }
    pp.SetMotorEnable(false);
    return 0;
}
