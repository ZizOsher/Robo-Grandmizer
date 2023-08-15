#include <iostream>
#include <libplayerc++/playerc++.h>
#include <opencv2/opencv.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <fstream>
#include "pathfinding.hpp"
#include "measurements.hpp"

using namespace boost;
using namespace PlayerCc;
using namespace cv;
using namespace Pathfinding;
namespace pf = Pathfinding;
namespace msrmnt = measurements;

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double timeToTurnInMicroseconds(double xRadians, double angularSpeed) {
    double timeInSeconds = xRadians / angularSpeed;

    return timeInSeconds * 1e6;  // Convert time to microseconds
}

void goToPoint(const pf::Point& currentLocation,
                PlayerCc::Position2dProxy& pp,
                double linearSpeed,
                double angularSpeed,
                const pf::Point& target) {
    // Converting current location and target from pixels to world coordinates
    double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
    double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
    double targetWorldX = msrmnt::pixel_to_world_x(target.x);
    double targetWorldY = msrmnt::pixel_to_world_y(target.y);

    // Calculating difference in x and y
    double dx = targetWorldX - currentWorldX;
    double dy = targetWorldY - currentWorldY;

    // Calculating the desired yaw (orientation) and yaw difference
    double desiredYaw = normalizeAngle(atan2(dy, dx));
    std::cout << "Desired yaw: " << desiredYaw << std::endl;
    double yawDifference = normalizeAngle(desiredYaw - pp.GetYaw());

    // If the yaw difference is beyond a threshold, correct the yaw first
    const double yawThreshold = 0.1;
    if (std::abs(yawDifference) > yawThreshold) {
        double turnSpeed = (yawDifference < 0) ? ((-1 ) * angularSpeed) : angularSpeed;  // Adjust angular speed based on yaw difference direction
        std::cout << "Adjusting yaw. Yaw difference: " << yawDifference << std::endl;
        std::cout << "Initial Yaw: " << pp.GetYaw() << std::endl;
        pp.SetSpeed(0, turnSpeed);  // Turn in place
        usleep(timeToTurnInMicroseconds(std::abs(yawDifference), angularSpeed)+10);  // Sleep for a short duration to give time for the robot to turn
        std::cout << "New Yaw: " << pp.GetYaw() << std::endl;
        pp.SetSpeed(0, 0);
        // return;  // Do not proceed further in this cycle. Just adjust the orientation.
    }

    // Calculate distance to target
    double distance = sqrt(dx*dx + dy*dy);

    // Convert the distance from world units to pixels to decide on speed
    int pixelDistance = msrmnt::world_distance_to_pixel_distance(distance);

    // Depending on the pixel distance, choose a speed
    double speed = (pixelDistance > 20) ? linearSpeed : linearSpeed / 2.0;  // Reduce speed if close to the target

    // If the distance is very small, stop the robot
    if (pixelDistance < 1) {
        pp.SetSpeed(0, 0);
    } else {
        pp.SetSpeed(speed, 0);
    }
    
    // Estimate time required in microseconds
    double timeRequired = distance / linearSpeed; // This gives time in seconds
    int sleepDuration = static_cast<int>(timeRequired * 1000000);  // Convert to microseconds
    
    // Sleep for the estimated duration
    usleep(sleepDuration);
}


int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <linearSpeed> <angularSpeed>" << std::endl;
        return 1;
    }

    double linearSpeed = std::stod(argv[1]);
    double angularSpeed = std::stod(argv[2]);

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
        std::vector<pf::Point> truePath = getTruePath(path, currentLocation, matrix, 1);
        std::cout << "True Path: ";
        for (const pf::Point& point : truePath) {
            std::cout << point.toString() << ",";
        }
        std::cout << std::endl;
        
        for (const pf::Point& point : truePath) {
            goToPoint(currentLocation, pp, linearSpeed,angularSpeed, point);          
            // Update current location to the point just reached
            currentLocation = point;
        }
        i++;
        usleep(500000);
        pp.SetSpeed(0, 0);
    }
    pp.SetMotorEnable(false);
    return 0;
}