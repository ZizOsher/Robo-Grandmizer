#include <iostream>
#include <libplayerc++/playerc++.h>
#include <opencv2/opencv.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <fstream>
#include <algorithm>
#include <map>
#include <numeric>
#include <list>
#include <nlohmann/json.hpp>
#include "pathfinding.hpp"
#include "measurements.hpp"
#include "jsonOperations.hpp"

using namespace boost;
using namespace PlayerCc;
using namespace cv;
using namespace Pathfinding;
namespace pf = Pathfinding;
namespace msrmnt = measurements;

std::vector<pf::Point> getPath(const Eigen::MatrixXd& matrix, 
                                const pf::Point& start, 
                                const pf::Point& target, 
                                const std::map<std::tuple<pf::Point, pf::Point>, std::string>& RoomDoorMapping,
                                const std::map<std::string, std::tuple<pf::Point, pf::Point>>& roomNameToInOut) {
    std::vector<pf::Point> path = astar(matrix, start, target);
    path.insert(path.begin(), start);
    std::vector<pf::Point> orderlyPath = computeOrderlyPath(path, RoomDoorMapping, roomNameToInOut);
    orderlyPath.erase(orderlyPath.begin());
    std::vector<pf::Point> truePath = getTruePath(orderlyPath, start, matrix, 1);
    return truePath;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double timeToTurnInMicroseconds(double xRadians, double angularSpeed) {
    double timeInSeconds = xRadians / angularSpeed;
    return timeInSeconds * 1e6; // Convert time to microseconds
}

double estimateTime(const std::vector<pf::Point>& truePath,
                    double linearSpeed, 
                    double angularSpeed,
                    const pf::Point& startingLocation) {
    double totalTime = 0.0;

    // Current location starts from the starting location.
    pf::Point currentLocation = startingLocation;

    for (int i=0; i<truePath.size() -1; i++) {
        // std::cout << "True Path: " << truePath[i].toString() << std::endl;
        double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
        double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
        double targetWorldX = msrmnt::pixel_to_world_x(truePath[i].x);
        double targetWorldY = msrmnt::pixel_to_world_y(truePath[i].y);

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
        currentLocation = truePath[i];
    }
    return totalTime;
}

// Compute the round trip based on time estimation as the distance metric.
std::vector<pf::Point> computeRoundTrip(const pf::Point& start,
                                        const std::vector<pf::Point>& destinations,
                                        const Eigen::MatrixXd& matrix,
                                        const std::map<std::tuple<pf::Point, pf::Point>, std::string>& doorToRoomName,
                                        const std::map<std::string, std::tuple<pf::Point, pf::Point>>& roomNameToInOut,
                                        double linearSpeed,
                                        double angularSpeed) {
    std::vector<pf::Point> unvisited = destinations;
    std::vector<pf::Point> result;
    pf::Point current = start;

    while (!unvisited.empty()) {
        double shortestTime = std::numeric_limits<double>::max();
        pf::Point closestPoint;
        if (unvisited.size() == 1) {
            closestPoint = unvisited[0];
            result.push_back(closestPoint);
            result.push_back(start);
            break;
        } else {
            for (const pf::Point& next : unvisited) {
                // Compute the path from the current to next.
                std::vector<pf::Point> path = getPath(matrix, current, next, doorToRoomName, roomNameToInOut);
                // Estimate the time to travel the path.
                double estimatedTime = estimateTime(path, linearSpeed, angularSpeed, current);
                if (estimatedTime < shortestTime) {
                    shortestTime = estimatedTime;
                    closestPoint = next;
                }
            }
        }
        result.push_back(closestPoint);
        unvisited.erase(std::remove(unvisited.begin(), unvisited.end(), closestPoint), unvisited.end());
        current = closestPoint;
    }
    return result;
}

bool adjustYaw(PlayerCc::Position2dProxy& pp, 
               const pf::Point& currentLocation, 
               const pf::Point& target, 
               PlayerClient& client, 
               double angularSpeed) {
    double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
    double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
    double targetWorldX = msrmnt::pixel_to_world_x(target.x);
    double targetWorldY = msrmnt::pixel_to_world_y(target.y);

    // Calculating difference in x and y
    double dx = targetWorldX - currentWorldX;
    double dy = targetWorldY - currentWorldY;

    double requiredYaw = normalizeAngle(atan2(dy, dx)); // In radians
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
    return true;
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
        if (fabs(yawError) > 0.01) {
            pp.SetSpeed(0, ((yawError > 0) ? angularSpeed : (-1) * angularSpeed));
        }
        usleep(timeToTurnInMicroseconds(std::abs(yawError), angularSpeed));
    } while(fabs(normalizeAngle(requiredYaw - currentYaw)) > 0.01);
    
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

        dx = targetWorldX - currentWorldX;
        dy = targetWorldY - currentWorldY;
        distanceToTarget = sqrt(dx*dx + dy*dy);
        pixelDistance = msrmnt::world_distance_to_pixel_distance(distanceToTarget);

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

int main(int argc, char *argv[]) {
    try {
        double linearSpeed = argv[1] ? std::stod(argv[1]) : 3.0;
        double angularSpeed = !argv[2] ? std::stod(argv[2]) : 1.0;

        // Administrative code: json parsing, matrix loading, player/stage proxy initialization, etc...
        auto jsonData = readJsonFile("../csfloor_mapping.json");
        std::map<std::tuple<pf::Point, pf::Point>, std::string> doorToRoomName = mapDoorsToRoomNames(jsonData);      
        std::map<std::string, std::tuple<pf::Point, pf::Point>> roomNameToInOut = mapRoomNameToInOut(jsonData);
        Eigen::MatrixXd matrix = loadMatrix("../binary_image.txt");
        PlayerClient client("localhost");
        Position2dProxy pp(&client, 0);

        // Administrative code: setting up the robot
        pp.SetMotorEnable(true);
        pf::Point currentLocation{91, 276};
        double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
        double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
        pp.SetOdometry(currentWorldX, currentWorldY, 0);
        
        std::cout << "CurrentLocation: " << currentLocation.x << ", " << currentLocation.y << std::endl;
        client.Read();      
        std::string room1;
        std::string room2;
        std::string room3;

        std::cout << "Enter the three faculty rooms you want visit: ";
        std::cin >> room1 >> room2 >> room3;
        
        pf::Point dest_1 = std::get<1>(roomNameToInOut[room1]);
        pf::Point dest_2 = std::get<1>(roomNameToInOut[room2]);
        pf::Point dest_3 = std::get<1>(roomNameToInOut[room3]);
        std::vector<pf::Point> waypoints = {dest_1, dest_2, dest_3};

        for (int i = 0; i < waypoints.size(); i++) {
            std::cout << "Waypoint " << i << ": " << waypoints[i].toString() << ", ";
        }
        std::cout << std::endl;

        // Estimate and sort waypoints using computeRoundTrip
        std::vector<pf::Point> sortedWaypoints = computeRoundTrip(currentLocation, waypoints, matrix, doorToRoomName, roomNameToInOut, linearSpeed, angularSpeed);
        
        std::cout << "Sorted Waypoints: " << std::endl;
        for (int i = 0; i < sortedWaypoints.size(); i++) {
            std::cout << "Waypoint " << i << ": " << sortedWaypoints[i].toString() << ", ";
        }

        // Assume a round trip time
        double roundTripTime = 700;  // seconds
        double waypointTime = 20;    // seconds per waypoint
        // Add extra time for all the waypoints
        roundTripTime += waypoints.size() * waypointTime;
        
        std::cout << "Before Loop." << std::endl;
        for (int i = 0; i < sortedWaypoints.size(); i++) {
            // Compute the path to the next waypoint
            std::vector<pf::Point> path = getPath(matrix, currentLocation, sortedWaypoints[i], doorToRoomName, roomNameToInOut);
            
            std::cout << "Path to waypoint: " << sortedWaypoints[i].toString() << std::endl;
            for (int j = 0; j < path.size(); j++) {
                std::cout << path[j].toString() << ", ";
            }
            std::cout << std::endl;

            int time_before_waypoint = time(NULL);
            // Drive using goToPoint
            int j = 0;
            while (j < path.size()) {
                bool success = goToPoint(currentLocation, pp, linearSpeed, angularSpeed, path[j], client);
                if (success) {
                    std::cout << "Reached the target point: " << path[j].toString() << std::endl;
                    j++;
                }
            }
            int time_after_waypoint = time(NULL);
            int time_taken = time_after_waypoint - time_before_waypoint;
            
            // Invoke the door behavior
            std::cout << "Arrived at waypoint: " << sortedWaypoints[i].toString() << " in " << time_taken << " seconds" << std::endl;
            std::cout << "Here be door behavior" << std::endl;
            
            currentLocation = sortedWaypoints[i];
        }
        // After all waypoints have been reached and we're back at the starting location
        std::cout << "Now that we're all here, we can party!" << std::endl;
        usleep(500000);
        pp.SetSpeed(0, 0);
        pp.SetMotorEnable(false);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
    }
}
