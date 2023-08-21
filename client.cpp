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
#include <nlohmann/json.hpp>
#include "pathfinding.hpp"
#include "measurements.hpp"
#include "jsonOperations.hpp"
#include "mcl.hpp"
#include <unordered_set>

using namespace boost;
using namespace PlayerCc;
using namespace cv;
using namespace Pathfinding;
namespace pf = Pathfinding;
namespace msrmnt = measurements;

std::string findRoomNameByPoint(const std::map<std::string, std::tuple<pf::Point, pf::Point>>& roomMap, const pf::Point& target) {
    for (const auto& entry : roomMap) {
        const auto& inPoint = std::get<0>(entry.second);
        const auto& outPoint = std::get<1>(entry.second);

        if (inPoint == target || outPoint == target) {
            return entry.first;
        }
    }

    return "";  // Return an empty string if not found.
}

std::vector<pf::Point> getPath(const Eigen::MatrixXd& matrix, 
                                const pf::Point& start,
                                const pf::Point& target,
                                const std::map<std::tuple<pf::Point, pf::Point>, std::string>& RoomDoorMapping,
                                const std::map<std::string, std::tuple<pf::Point, pf::Point>>& roomNameToInOut) {
    std::vector<pf::Point> path = astar(matrix, start, target);
    path.insert(path.begin(), start);
    std::vector<pf::Point> orderlyPath = computeOrderlyPath(path, RoomDoorMapping, roomNameToInOut);
    orderlyPath.erase(orderlyPath.begin());
    std::vector<pf::Point> truePath = getTruePath(orderlyPath, start, matrix, 4);
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
    pf::Point currentLocation = startingLocation;

    for (int i=0; i<truePath.size() -1; i++) {
        // Calculating difference in x and y
        double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
        double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
        double targetWorldX = msrmnt::pixel_to_world_x(truePath[i].x);
        double targetWorldY = msrmnt::pixel_to_world_y(truePath[i].y);
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

        currentLocation = truePath[i];
    }
    return totalTime;
}

struct RoundTrip {
    std::vector<pf::Point> path;
    std::vector<double> segmentTimes;
    double totalTime;
};

// Compute the round trip based on time estimation as the distance metric.
RoundTrip computeRoundTrip(const pf::Point& start,
                                        const std::vector<pf::Point>& destinations,
                                        const Eigen::MatrixXd& matrix,
                                        const std::map<std::tuple<pf::Point, pf::Point>, std::string>& doorToRoomName,
                                        const std::map<std::string, std::tuple<pf::Point, pf::Point>>& roomNameToInOut,
                                        double linearSpeed,
                                        double angularSpeed) {
    std::vector<pf::Point> unvisited = destinations;
    std::vector<pf::Point> result;
    std::vector<double> segmentTimes;
    double time = 0.0;
    
    pf::Point current = start;

    while (!unvisited.empty()) {
        double shortestTime = std::numeric_limits<double>::max();
        pf::Point closestPoint;
        if (unvisited.size() == 1) {
            closestPoint = unvisited[0];
            std::vector<pf::Point> path = getPath(matrix, current, closestPoint, doorToRoomName, roomNameToInOut);
            double estimatedTime = estimateTime(path, linearSpeed, angularSpeed, current);
            segmentTimes.push_back(estimatedTime);
            time += estimatedTime;
            result.push_back(closestPoint);
            path = getPath(matrix, closestPoint, start, doorToRoomName, roomNameToInOut);
            estimatedTime = estimateTime(path, linearSpeed, angularSpeed, current);
            segmentTimes.push_back(estimatedTime);
            time += estimatedTime;
            result.push_back(start);
            break;
        } else {
            for (const pf::Point& next : unvisited) {
                // Compute the path and estimate the time to travel the path.
                std::vector<pf::Point> path = getPath(matrix, current, next, doorToRoomName, roomNameToInOut);
                double estimatedTime = estimateTime(path, linearSpeed, angularSpeed, current);
                if (estimatedTime < shortestTime) {
                    shortestTime = estimatedTime;
                    closestPoint = next;
                }
            }
        }
        result.push_back(closestPoint);
        segmentTimes.push_back(shortestTime);
        time += shortestTime;
        unvisited.erase(std::remove(unvisited.begin(), unvisited.end(), closestPoint), unvisited.end());
        current = closestPoint;
    }
    return {result, segmentTimes, time};
}

bool adjustYaw(PlayerCc::Position2dProxy& pp, 
               const pf::Point& currentLocation, 
               const pf::Point& target, 
               PlayerClient& robot, 
               double angularSpeed) {
    double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
    double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
    double targetWorldX = msrmnt::pixel_to_world_x(target.x);
    double targetWorldY = msrmnt::pixel_to_world_y(target.y);
    double dx = targetWorldX - currentWorldX;
    double dy = targetWorldY - currentWorldY;

    double requiredYaw = normalizeAngle(atan2(dy, dx)); // In radians
    double currentYaw;
    do {
        robot.Read();
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
                PlayerClient& robot
                ) {
    double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
    double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
    double targetWorldX = msrmnt::pixel_to_world_x(target.x);
    double targetWorldY = msrmnt::pixel_to_world_y(target.y);
    double dx = targetWorldX - currentWorldX;
    double dy = targetWorldY - currentWorldY;

    // Step 1: Calculate the Required Yaw Angle in 
    double requiredYaw = normalizeAngle(atan2(dy, dx)); // Angle in radians

    // Step 2: Rotate the Robot to the Required Yaw Angle
    double currentYaw;
    do {
        robot.Read();
        currentYaw = pp.GetYaw();
        double yawError = normalizeAngle(requiredYaw - currentYaw);
        // Set angular speed based on the error
        if (fabs(yawError) > 0.009) {
            pp.SetSpeed(0, ((yawError > 0) ? angularSpeed : (-1) * angularSpeed));
        }

        double turningTimeMs = timeToTurnInMicroseconds(std::abs(yawError), angularSpeed);
        double iterTimeMs = turningTimeMs;
        double iterTimeS = iterTimeMs / 1000000.0;  // Convert microseconds to seconds
 
        // mcl.motionUpdate(0, 0, angularSpeed * iterTimeS, 0.1);
        usleep(iterTimeMs);
    } while(fabs(normalizeAngle(requiredYaw - currentYaw)) > 0.005);
    
    pp.SetSpeed(0, 0);
    
    // Step 3: Drive the Robot to the Target Point
    double distanceToTarget;
    int pixelDistance;
    double minDistanceToTarget = std::numeric_limits<double>::max(); // Initialized with a large value

    do {
        robot.Read();

        //Computations for mcl
        // std::vector<double> measurements;
        // for (int i = 0; i < rp.GetRangeCount(); i++) {
        //     measurements.push_back(rp[i]);
        // }
        // mcl.measurementUpdate(measurements);
        // mcl.resampleParticles();

        // Estimate position using MCL
        // Particle estimatedPosition = mcl.getBestParticle();

        currentWorldX = pp.GetXPos();
        currentWorldY = pp.GetYPos();
        currentLocation.x = msrmnt::world_to_pixel_x(currentWorldX);
        currentLocation.y = msrmnt::world_to_pixel_y(currentWorldY);

        dx = targetWorldX - currentWorldX;
        dy = targetWorldY - currentWorldY;
        distanceToTarget = sqrt(dx*dx + dy*dy);
        pixelDistance = msrmnt::world_distance_to_pixel_distance(distanceToTarget);

        // double estimatedDistanceToTarget = sqrt(pow(target.x - estimatedPosition.x, 2) + pow(target.y - estimatedPosition.y, 2));

        // std::cout << "distanceToTarget: " << distanceToTarget << std::endl;
        // std::cout << "estimatedDistanceToTarget: " << estimatedDistanceToTarget << std::endl;

        currentYaw = pp.GetYaw();
        double yawError = normalizeAngle(requiredYaw - currentYaw);
        // Set angular speed based on the error
        if (fabs(yawError) > 0.009) {
            pp.SetSpeed(0, 0);
            return false;
        }

        if ((distanceToTarget - minDistanceToTarget) > 0.3) {
            pp.SetSpeed(0, 0);
            return false;
        }

        if (pixelDistance > 1 || distanceToTarget > 0.2) {
            pp.SetSpeed(linearSpeed, 0);
        }

        double drivingTimeMs = (distanceToTarget / linearSpeed) * 1000000; // This gives time in seconds
        double sleepTimeMs = drivingTimeMs / 50000.0; 
        double sleepTimeS = drivingTimeMs / 1000000.0;
        
        // mcl.motionUpdate(linearSpeed * sleepTimeS, 0, 0, 0.1);
        usleep(sleepTimeMs);
        minDistanceToTarget = (distanceToTarget < minDistanceToTarget) ? distanceToTarget : minDistanceToTarget;

    } while(pixelDistance > 1 || distanceToTarget > 0.2);
    
    // Step 4: Stop the Robot
    pp.SetSpeed(0, 0);
    return true;
}

void leaveRoom(pf::Point currentLocation,
                std::string roomName,
                std::map<std::tuple<Pathfinding::Point, Pathfinding::Point>, std::string> doorToRoomName,
                const std::map<std::string, std::tuple<pf::Point, pf::Point>>& roomNameToInOut,
                PlayerCc::Position2dProxy& pp,
                PlayerClient& robot,
                double linearSpeed,
                double angularSpeed,
                const Eigen::MatrixXd& matrix) {
    std::cout << "Leaving room: " << roomName << std::endl;
    std::cout << "..." << std::endl;
    pf::Point roomIn = std::get<0>(roomNameToInOut.at(roomName));
    pf::Point roomOut = std::get<1>(roomNameToInOut.at(roomName));

    if (currentLocation != roomIn) {
        std::vector<pf::Point> path = getPath(matrix,
                                                currentLocation,
                                                roomIn,
                                                doorToRoomName,
                                                roomNameToInOut);
        int j = 0;
        while (j < path.size()) {
            bool success = goToPoint(currentLocation, pp, linearSpeed, angularSpeed, path[j], robot);
            if (success) {
                std::cout << "Reached the intermediate point: " << path[j].toString() << std::endl;
                j++;
            }
        }
    }
    std::cout << "Oh boy! I sure am happy a door hasn't been installed yet!" << std::endl;
    std::cout << "If it had been installed, I would have played a pre-recorded message like:" << std::endl << std::endl;
    std::cout << '"' << "Please open the door, I'm a simple robot and I want to leave the room!" << '"' << std::endl << std::endl;
    std::cout << "Then I would have waited for the door to open and only then would have left the room." << std::endl;
    sleep(5);
    std::cout << "Now exiting room " << roomName << std::endl;
    bool success = false;
    while (!success) {
        bool success = goToPoint(currentLocation, pp, linearSpeed, angularSpeed, roomOut, robot);
        if (success) {
            std::cout << "Reached the intermediate point: " << roomOut.toString() << std::endl;
            break;
        }
    }
}

int main(int argc, char *argv[]) {
    try {
        double linearSpeed = 0.3;
        double angularSpeed = 0.09;

        // Administrative code: json parsing, matrix loading, player/stage proxy initialization, etc...
        auto jsonData = readJsonFile("../csfloor_mapping.json");
        std::map<std::tuple<pf::Point, pf::Point>, std::string> doorToRoomName = mapDoorsToRoomNames(jsonData);      
        std::map<std::string, std::tuple<pf::Point, pf::Point>> roomNameToInOut = mapRoomNameToInOut(jsonData);
        Eigen::MatrixXd matrix = loadMatrix("../binary_image.txt");
        PlayerClient robot("localhost");
        Position2dProxy pp(&robot, 0);
        RangerProxy rp(&robot, 0);

        // Administrative code: setting up the robot
        pp.SetMotorEnable(true);
        pf::Point currentLocation{91, 276};
        double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
        double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
        pp.SetOdometry(currentWorldX, currentWorldY, 0);

        std::cout << "CurrentLocation: " << currentLocation.x << ", " << currentLocation.y << std::endl;
        robot.Read();      
        std::string room1;
        std::string room2;
        std::string room3;

        std::cout << "Enter the three faculty rooms you want visit: ";
        std::cin >> room1 >> room2 >> room3;
        
        pf::Point dest_1 = (roomNameToInOut.find(room1) != roomNameToInOut.end()) ? std::get<1>(roomNameToInOut[room1]) : pf::Point{0, 0};
        pf::Point dest_2 = (roomNameToInOut.find(room2) != roomNameToInOut.end()) ? std::get<1>(roomNameToInOut[room2]) : pf::Point{0, 0};
        pf::Point dest_3 = (roomNameToInOut.find(room3) != roomNameToInOut.end()) ? std::get<1>(roomNameToInOut[room3]) : pf::Point{0, 0};

        // Removing duplicates and pruning waypoints
        std::vector<pf::Point> waypoints = {dest_1, dest_2, dest_3};
        std::sort(waypoints.begin(), waypoints.end());
        waypoints.erase(std::unique(waypoints.begin(), waypoints.end()), waypoints.end());
        waypoints.erase(std::remove(waypoints.begin(), waypoints.end(), pf::Point{0,0}), waypoints.end());

        if (waypoints.empty()) {
            std::cout << "No valid waypoints entered. Exiting..." << std::endl;
            return 0;
        }

        // Estimate and sort waypoints using computeRoundTrip
        RoundTrip roundTrip = computeRoundTrip(currentLocation,
                                               waypoints,
                                               matrix,
                                               doorToRoomName,
                                               roomNameToInOut,
                                               linearSpeed,
                                               angularSpeed);
        
        
        std::vector<pf::Point> roundTripPath = roundTrip.path;
        roundTripPath.back() = std::get<1>(roomNameToInOut["327"]);
        std::vector<double> segmentTimes = roundTrip.segmentTimes;
        double totalTime = roundTrip.totalTime;

        double waypointTime = 35;    // aproximate seconds spent at each waypoint
        totalTime += waypoints.size() * waypointTime;
        
        // std::cout << "Round trip times: " << std::endl;
        // for (int i = 0; i < segmentTimes.size(); i++) {
        //     std::cout << "Segment " << i << ": " << segmentTimes[i] << " seconds" << std::endl;
        // }
        // std::cout << "Total time: " << totalTime << " seconds" << std::endl;

        std::cout << "Round trip room order: " << std::endl;
        for (int i = 0; i < roundTripPath.size() - 1; i++) {
            std::cout << "room: " << findRoomNameByPoint(roomNameToInOut, roundTripPath[i]) << ", ";
        }
        std::cout << "and back to room: 330" << std::endl;

        // Leaving the Robotics lab

        std::string currentRoom = "330";
        leaveRoom(currentLocation,
                    currentRoom,
                    doorToRoomName,
                    roomNameToInOut,
                    pp,
                    robot,
                    linearSpeed,
                    angularSpeed,
                    matrix);

        // currentLocation = std::get<1>(roomNameToInOut[currentRoom]);
        // std::cout << "CurrentLocation: " << currentLocation.x << ", " << currentLocation.y << std::endl;
        std::cout << "CurrentLocation: " << std::get<1>(roomNameToInOut[currentRoom]).toString() << std::endl;
        std::cout << "Current Location based on odometry: " << msrmnt::world_to_pixel(pp.GetXPos(),pp.GetYPos()).toString() << std::endl;
        currentLocation = msrmnt::world_to_pixel(pp.GetXPos(),pp.GetYPos());
        currentRoom = "hallway";
        
        for (int i = 0; i < roundTripPath.size(); i++) {
            // Compute the path to the next waypoint
            std::vector<pf::Point> path = getPath(matrix, currentLocation, roundTripPath[i], doorToRoomName, roomNameToInOut);
            
            std::cout << "Path to waypoint: " << roundTripPath[i].toString() << std::endl;
            for (int j = 0; j < path.size(); j++) {
                std::cout << path[j].toString() << ", ";
            }
            std::cout << std::endl;

            int time_before_waypoint = time(NULL);
            // Drive using goToPoint
            int j = 0;
            while (j < path.size()) {
                bool success = goToPoint(currentLocation, pp, linearSpeed, angularSpeed, path[j], robot);
                if (success) {
                    std::cout << "Reached the intermediate point: " << path[j].toString() << std::endl;
                    j++;
                }
                currentLocation = msrmnt::world_to_pixel(pp.GetXPos(),pp.GetYPos());
            }
            int time_after_waypoint = time(NULL);
            int time_taken = time_after_waypoint - time_before_waypoint;
            
            // Invoke the door behavior
            std::cout << "Arrived at waypoint: " << roundTripPath[i].toString() << " in " << time_taken << " seconds" << std::endl;
            std::cout << "Here be door behavior" << std::endl;
            
            currentLocation = roundTripPath[i];
        }
        // After all waypoints have been reached and we're back at the starting location
        std::cout << "Now that we're all here, we can party!" << std::endl;
        usleep(500000);
        pp.SetSpeed(0, 0);
        pp.SetMotorEnable(false);
        return 0;
    } catch (const std::exception& ex) {
        std::cout << "Exception: " << ex.what() << std::endl;
        std::cerr << ex.what() << std::endl;
    }
}
