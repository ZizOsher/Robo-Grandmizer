#include <iostream>
#include <libplayerc++/playerc++.h>
#include <opencv2/opencv.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <fstream>
#include "pathfinding.hpp"
#include "measurements.hpp"
#include <nlohmann/json.hpp>
#include "jsonOperations.hpp"

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
    // Convert time to microseconds
    return timeInSeconds * 1e6;
}

double estimateTime(const std::vector<pf::Point>& truePath,
                    double linearSpeed, 
                    double angularSpeed,
                    const pf::Point& startingLocation) {
    double totalTime = 0.0;

    // Current location starts from the starting location.
    pf::Point currentLocation = startingLocation;

    for (const pf::Point& target : truePath) {
        double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
        double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
        double targetWorldX = msrmnt::pixel_to_world_x(target.x);
        double targetWorldY = msrmnt::pixel_to_world_y(target.y);

        // Calculating difference in x and y
        double dx = targetWorldX - currentWorldX;
        double dy = targetWorldY - currentWorldY;

        // Time to Turn:
        double requiredYaw = normalizeAngle(atan2(dy, dx)); // Angle in radians
        double currentYaw = 0; // Assuming robot starts with yaw of 0 for simplicity.
        double yawError = normalizeAngle(requiredYaw - currentYaw);
        totalTime += std::abs(yawError) / angularSpeed;

        // Time to Move:
        double distanceToTarget = sqrt(dx*dx + dy*dy);
        totalTime += distanceToTarget / linearSpeed;

        // Update the current location for the next iteration.
        currentLocation = target;
    }
    
    return totalTime;
}

bool goToPoint(pf::Point& currentLocation,
                PlayerCc::Position2dProxy& pp,
                double linearSpeed,
                double angularSpeed,
                const pf::Point& target,
                PlayerClient& client) {
    double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
    double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
    double targetWorldX = msrmnt::pixel_to_world_x(target.x);
    double targetWorldY = msrmnt::pixel_to_world_y(target.y);

    // Calculating difference in x and y
    double dx = targetWorldX - currentWorldX;
    double dy = targetWorldY - currentWorldY;

    // Step 1: Calculate the Required Yaw Angle in 
    double requiredYaw = normalizeAngle(atan2(dy, dx)); // Angle in radians

    // Step 2: Rotate the Robot to the Required Yaw Angle
    double currentYaw;
    do {
        client.Read();
        currentYaw = pp.GetYaw();
        double yawError = normalizeAngle(requiredYaw - currentYaw);
        
        // Set angular speed based on the error
        if (fabs(yawError) > 0.025) {
            pp.SetSpeed(0, ((yawError > 0) ? angularSpeed : (-1) * angularSpeed));
        }
        usleep(timeToTurnInMicroseconds(std::abs(yawError), angularSpeed));
    } while(fabs(normalizeAngle(requiredYaw - currentYaw)) > 0.025);
    
    pp.SetSpeed(0, 0);
    
    // Step 3: Drive the Robot to the Target Point
    double distanceToTarget;
    int pixelDistance;
    double minDistanceToTarget = std::numeric_limits<double>::max(); // Initialized with a large value

    do {
        client.Read();

        currentWorldX = pp.GetXPos();
        currentWorldY = pp.GetYPos();
        currentLocation.x = msrmnt::world_to_pixel_x(currentWorldX);
        currentLocation.y = msrmnt::world_to_pixel_y(currentWorldY);

        // std::cout << "Current Location: " << currentWorldX << ", " << currentWorldY << std::endl;
        // std::cout << "Target Location: " << targetWorldX << ", " << targetWorldY << std::endl;

        dx = targetWorldX - currentWorldX;
        dy = targetWorldY - currentWorldY;
        distanceToTarget = sqrt(dx*dx + dy*dy);
        pixelDistance = msrmnt::world_distance_to_pixel_distance(distanceToTarget);
        // std::cout << "Distance to Target: " << distanceToTarget << std::endl;

        if ((distanceToTarget - minDistanceToTarget) > 0.5) {
            pp.SetSpeed(0, 0);
            return false;
        }

        if (pixelDistance > 1 || distanceToTarget > 0.5) {
            pp.SetSpeed(linearSpeed, 0);
        }
    
        double timeRequired = distanceToTarget / linearSpeed; // This gives time in seconds
        int sleepDuration = static_cast<int>(timeRequired * 1000000);  // Convert to microseconds
        usleep(sleepDuration/10000);

        minDistanceToTarget = (distanceToTarget < minDistanceToTarget) ? distanceToTarget : minDistanceToTarget;

    } while(pixelDistance > 1 || distanceToTarget > 0.5);
    
    // Step 4: Stop the Robot
    pp.SetSpeed(0, 0);
    return true;
}

std::vector<pf::Point> getPath(const Eigen::MatrixXd& matrix, 
                                pf::Point start, 
                                pf::Point target, 
                                const std::map<std::tuple<pf::Point, pf::Point>, std::string>& RoomDoorMapping,
                                const std::map<std::string, std::tuple<pf::Point, pf::Point>>& roomInOutMap) {
    std::vector<pf::Point> path = astar(matrix, start, target);
    path.insert(path.begin(), start);
    std::vector<pf::Point> orderlyPath = computeOrderlyPath(path, RoomDoorMapping, roomInOutMap);
    orderlyPath.erase(path.begin());
    std::vector<pf::Point> truePath = getTruePath(orderlyPath, start, matrix, 1);
    std::cout << "True Path: ";
    for (const pf::Point& point : truePath) {
        std::cout << point.toString() << ",";
    }
    std::cout << std::endl;
    return truePath;
}

int main(int argc, char *argv[]) {
    if (!argv[1]) {
        double linearSpeed = 3.0; // m/s
        
    }
    if (!argv[2]) {
        double angularSpeed = 1.0; // rad/s
    }

    try {
        double linearSpeed = std::stod(argv[1]);
        double angularSpeed = std::stod(argv[2]);

        // Administrative code to read the json file and map doors to rooms
        auto jsonData = readJsonFile("../csfloor_mapping.json");
        auto doorRoomMap = mapDoorsToRooms(jsonData);
        auto roomInOutMap = mapRoomsInOut(jsonData);
        Eigen::MatrixXd matrix = loadMatrix("../binary_image.txt");
        PlayerClient client("localhost");
        Position2dProxy pp(&client, 0);
        
        pp.SetMotorEnable(true);
        pf::Point currentLocation{91, 276};
        double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
        double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
        pp.SetOdometry(currentWorldX, currentWorldY, 0);
        
        int i = 0;
        while (i < 1) {
            std::cout << "CurrentLocation: " << currentLocation.x << ", " << currentLocation.y << std::endl;
            client.Read();
            pf::Point target;
            std::cout << "Enter the target coordinates (x y): ";
            std::cin >> target.x >> target.y;
            
            std::vector<pf::Point> path = getPath(matrix, currentLocation, target, doorRoomMap, roomInOutMap);
            
            double estimatedTime = estimateTime(path, linearSpeed, angularSpeed, currentLocation);
            std::cout << "Estimated time to complete the path: " << estimatedTime << " seconds" << std::endl;

            int time_before_loop_begins = time(NULL);

            // for (const pf::Point& point : path) {
            //     bool success = goToPoint(currentLocation, pp, linearSpeed,angularSpeed, point, client);          
            //     if (!success) {
            //         std::cout << "Failed to reach the target point" << std::endl;
            //         break;
            //     }
            //     currentLocation = point;
            // }

            int j = 0;
            while (j < path.size()) {
                bool success = goToPoint(currentLocation, pp, linearSpeed, angularSpeed, path[j], client);
                if (success) {
                    std::cout << "Reached the target point: " << path[j].toString() << std::endl;
                    j++;
                }
            }
        
            int time_after_loop_ends = time(NULL);
            int time_diff = time_after_loop_ends - time_before_loop_begins;

            std::cout << "Time taken to complete the path: " << time_diff << " seconds" << std::endl;

            i++;
            usleep(500000);
            pp.SetSpeed(0, 0);
        }
        pp.SetMotorEnable(false);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
    }
}
