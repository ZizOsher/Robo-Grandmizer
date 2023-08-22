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
#include <unordered_set>
#include <unistd.h>

using namespace boost;
using namespace PlayerCc;
using namespace cv;
using namespace Pathfinding;
namespace pf = Pathfinding;
namespace msrmnt = measurements;

std::string findRoomNameByPoint(const std::map<std::string,
                                std::tuple<pf::Point, pf::Point>>& roomMap,
                                const pf::Point& target) {
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
    std::vector<pf::Point> truePath = getTruePath(orderlyPath, start, matrix, 3);
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

void AvoidObstacles(double *linearSpeed,
                    double *angularSpeed,
                    PlayerCc::RangerProxy& rp,
                    int *evasiveAction) {
    // Constants for robot behavior
    const double safe_distance = 0.5;  // robot keeps at least 50cm from obstacles
    const double side_threshold = 0.4;  // distance from side walls we want to avoid
    const double max_linear_speed = 0.3;
    const double max_angular_speed = 0.15;

    // Check quadrants for obstacles
    int front_left = 0;
    int front_right = 0;
    int far_left = 0;
    int far_right = 0;

    for (int i=0; i<180; i++) {
        if (i<45 && rp[i] < side_threshold) {
            far_right++;
        } else if (i>=45 && i<90 && rp[i] < safe_distance) {
            front_right++;
        } else if (i>=90 && i<135 && rp[i] < safe_distance) {
            front_left++;
        } else if (i>=135 && i<180 && rp[i] < side_threshold) {
            far_left++;
        }
    }

    // Adjust robot's speed
    if (front_left > 2 || far_left > 2) {
        *evasiveAction = 1;
        std::cout << "Obstacle detected on the left" << std::endl;
        *linearSpeed = 0.2;
        *angularSpeed = max_angular_speed;  // turn right
    } else if(front_right > 2 || far_right > 2) {
        *evasiveAction = 1;
        std::cout << "Obstacle detected on the right" << std::endl;
        *linearSpeed = 0.2;
        *angularSpeed = -max_angular_speed;  // turn left
    } else {
        if (*evasiveAction==1) {
            *evasiveAction = 2;
            std::cout << "Obstacle avoided" << std::endl;
        } else {
            *evasiveAction = 0;
        }
        *linearSpeed = max_linear_speed;
        *angularSpeed = 0;  // go straight
    }
}


double shortestDistanceToLine(double currentLocationX,
                                double currentLocationY,
                                double targetX,
                                double targetY,
                                double yaw
                                ) {
    double m = tan(yaw);
    double numerator = std::abs(m * targetX - targetY + currentLocationY - m * currentLocationX);
    double denominator = std::sqrt(m * m + 1);
    return numerator / denominator;
}

bool goToPoint(pf::Point& currentLocation,
                PlayerCc::Position2dProxy& pp,
                double linearSpeed,
                double angularSpeed,
                const pf::Point& target,
                PlayerClient& robot,
                PlayerCc::RangerProxy& rp
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
        if (fabs(yawError) > 0.01) {
            pp.SetSpeed(0, ((yawError > 0) ? angularSpeed : (-1) * angularSpeed));
        }

        double turningTimeMs = timeToTurnInMicroseconds(std::abs(yawError), angularSpeed);
        double iterTimeMs = turningTimeMs;
        double iterTimeS = iterTimeMs / 1000000.0;  // Convert microseconds to seconds
 
        usleep(iterTimeMs / 50);
    } while(fabs(normalizeAngle(requiredYaw - currentYaw)) > 0.01);
    
    pp.SetSpeed(0, 0);
    
    // Step 3: Drive the Robot to the Target Point
    double distanceToTarget;
    int pixelDistance;
    double minDistanceToTarget = std::numeric_limits<double>::max(); // Initialized with a large value
    
    int evasiveAction = 0;
    do {
        robot.Read();

        currentWorldX = pp.GetXPos();
        currentWorldY = pp.GetYPos();
        currentLocation.x = msrmnt::world_to_pixel_x(currentWorldX);
        currentLocation.y = msrmnt::world_to_pixel_y(currentWorldY);

        dx = targetWorldX - currentWorldX;
        dy = targetWorldY - currentWorldY;
        distanceToTarget = sqrt(dx*dx + dy*dy);

        robot.Read();
        AvoidObstacles(&linearSpeed, &angularSpeed, rp, &evasiveAction);

        if (evasiveAction == 2) {
            // After avoiding obstacle, reorient to the target
            pp.SetSpeed(0, 0);
            return false;
        }

        if ((distanceToTarget - minDistanceToTarget) > 0.2) {
            pp.SetSpeed(0, 0);
            return false;
        }

        if (distanceToTarget > 0.1) {
            pp.SetSpeed(linearSpeed, 0);
        }

        double drivingTimeMs = (distanceToTarget / linearSpeed) * 1000000; // This gives time in microseconds
        double sleepTimeMs = drivingTimeMs / 20000.0; 
        
        usleep(sleepTimeMs);
        minDistanceToTarget = (distanceToTarget < minDistanceToTarget) ? distanceToTarget : minDistanceToTarget;

    } while (distanceToTarget > 0.1);
    
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
                const Eigen::MatrixXd& matrix,
                PlayerCc::RangerProxy& rp) {
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
            bool success = goToPoint(currentLocation, pp, linearSpeed, angularSpeed, path[j], robot, rp);
            if (success) {
                std::cout << "Reached the intermediate point: " << path[j].toString() << std::endl;
                j++;
            }
        }
    }
    std::cout << std::endl;
    std::cout << "Robot: " << std::endl;
    std::cout << "\"Oh boy! I sure am happy a door hasn't been installed yet!" << std::endl;
    std::cout << "If it had been installed, I would have played a pre-recorded message like:" << std::endl << std::endl;
    std::cout << '"' << "Please open the door for me." << '"' << std::endl << std::endl;
    std::cout << "Then I would have waited for the door to open and only then would have left the room.\"" << std::endl;
    sleep(5);
    std::cout << "Now exiting room " << roomName << std::endl;
    bool success = false;
    while (!success) {
        bool success = goToPoint(currentLocation, pp, linearSpeed, angularSpeed, roomOut, robot, rp);
        if (success) {
            std::cout << "Reached the intermediate point: " << roomOut.toString() << std::endl;
            break;
        }
    }
}

void enterRoom(pf::Point currentLocation,
                std::string roomName,
                std::map<std::tuple<Pathfinding::Point, Pathfinding::Point>, std::string> doorToRoomName,
                const std::map<std::string, std::tuple<pf::Point, pf::Point>>& roomNameToInOut,
                PlayerCc::Position2dProxy& pp,
                PlayerClient& robot,
                double linearSpeed,
                double angularSpeed,
                const Eigen::MatrixXd& matrix,
                PlayerCc::RangerProxy& rp) {
    std::cout << "Entering room: " << roomName << std::endl;
    std::cout << "..." << std::endl;
    pf::Point roomIn = std::get<0>(roomNameToInOut.at(roomName));
    pf::Point roomOut = std::get<1>(roomNameToInOut.at(roomName));

    if (currentLocation != roomOut) {
        std::vector<pf::Point> path = getPath(matrix,
                                                currentLocation,
                                                roomOut,
                                                doorToRoomName,
                                                roomNameToInOut);
        int j = 0;
        while (j < path.size()) {
            bool success = goToPoint(currentLocation, pp, linearSpeed, angularSpeed, path[j], robot, rp);
            if (success) {
                std::cout << "Reached the intermediate point: " << path[j].toString() << std::endl;
                j++;
            }
        }
    }
    std::cout << std::endl;
    std::cout << "Robot: " << std::endl;
    std::cout << "\"Oh boy! I sure am happy a door hasn't been installed yet!" << std::endl;
    std::cout << "If it had been installed, I would have played a pre-recorded message like:" << std::endl << std::endl;
    std::cout << "\"Please open the door for me.\"" << std::endl << std::endl;
    std::cout << "Then I would have waited for the door to open and only then would have entered the room.\"" << std::endl<< std::endl;
    sleep(5);
    bool success = false;
    while (!success) {
        bool success = goToPoint(currentLocation, pp, linearSpeed, angularSpeed, roomIn, robot, rp);
        if (success) {
            std::cout << "Reached the intermediate point: " << roomIn.toString() << std::endl;
            break;
        }
    }
}


int main(int argc, char *argv[]) {
    try {
        const double linearSpeed = 0.35;
        const double angularSpeed = 0.15;

        // Administrative code: json parsing, matrix loading, player/stage proxy initialization, etc...
        auto jsonData = readJsonFile("../csfloor_mapping.json");
        std::map<std::tuple<pf::Point, pf::Point>, std::string> doorToRoomName = mapDoorsToRoomNames(jsonData);      
        std::map<std::string, std::tuple<pf::Point, pf::Point>> roomNameToInOut = mapRoomNameToInOut(jsonData);
        Eigen::MatrixXd matrix = loadMatrix("../binary_image.txt");
        PlayerClient robot("localhost");
        Position2dProxy pp(&robot, 0);
        RangerProxy rp(&robot, 0);
        std::cout << "Robot is connected" << std::endl;

        // Administrative code: setting up the robot
        pp.SetMotorEnable(true);
        pf::Point currentLocation{91, 276};
        double currentWorldX = msrmnt::pixel_to_world_x(currentLocation.x);
        double currentWorldY = msrmnt::pixel_to_world_y(currentLocation.y);
        pp.SetOdometry(currentWorldX, currentWorldY, 0);

        sleep(1);
        std::cout << std::endl;
        std::cout << "Robot: \"Oh boy! I'm going on an adventure!\"" << std::endl;
        std::cout << std::endl;
        sleep(1);

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

        // Estimate time and sort waypoints using computeRoundTrip
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
        
        double totalTimeEstimate = roundTrip.totalTime;
        double elapsedTime = 0.0;
        double waypointTime = 35;    // aproximate seconds spent at each waypoint
        totalTimeEstimate += waypoints.size() * waypointTime;
        totalTimeEstimate = totalTimeEstimate *2;
        
        std::cout << "Round trip room order: " << std::endl;
        for (int i = 0; i < roundTripPath.size() - 1; i++) {
            std::cout << "room: " << findRoomNameByPoint(roomNameToInOut, roundTripPath[i]) << ", ";
        }
        std::cout << "and back to The robotics lab." << std::endl;
        std::cout << "This should take me about " << totalTimeEstimate/60 << " minutes." << std::endl<< std::endl;

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
                    matrix,
                    rp);

        std::cout << "CurrentLocation: " << std::get<1>(roomNameToInOut[currentRoom]).toString() << std::endl;
        std::cout << "Current Location based on odometry: " << msrmnt::world_to_pixel(pp.GetXPos(),pp.GetYPos()).toString() << std::endl;
        currentLocation = msrmnt::world_to_pixel(pp.GetXPos(),pp.GetYPos());
        currentRoom = "hallway";
        
        for (int i = 0; i < roundTripPath.size(); i++) {
            // Compute the path to the next waypoint
            int timeBefore = time(NULL);
            std::vector<pf::Point> path = getPath(matrix, currentLocation, roundTripPath[i], doorToRoomName, roomNameToInOut);
            
            std::cout << "Path to waypoint: " << roundTripPath[i].toString() << std::endl;
            for (int j = 0; j < path.size(); j++) {
                std::cout << path[j].toString() << ", ";
            }
            std::cout << std::endl;

            // Drive using goToPoint
            int j = 0;
            while (j < path.size()) {
                bool success = goToPoint(currentLocation, pp, linearSpeed, angularSpeed, path[j], robot, rp);
                if (success) {
                    std::cout << "Reached the intermediate point: " << path[j].toString() << std::endl;
                    j++;
                }
                robot.Read();
                currentLocation = msrmnt::world_to_pixel(pp.GetXPos(),pp.GetYPos());
            }
            int timeAfter = time(NULL);
            int timeToCurrentWaypoint = timeAfter - timeBefore;
            elapsedTime += timeToCurrentWaypoint;
            
            // Invoke the door behavior
            std::cout << "Arrived at room: " << findRoomNameByPoint(roomNameToInOut, roundTripPath[i]) << ". And it only took me " << timeToCurrentWaypoint/60 << " minutes" << std::endl;
            currentRoom = findRoomNameByPoint(roomNameToInOut, roundTripPath[i]);
            enterRoom(currentLocation,
                        currentRoom,
                        doorToRoomName,
                        roomNameToInOut,
                        pp,
                        robot,
                        linearSpeed,
                        angularSpeed,
                        matrix,
                        rp);
            std::cout << "after entering" << std::endl;
            if (i < roundTripPath.size() - 1) {
                std::cout << std::endl;
                std::cout << "Robot: " << std::endl;
                std::cout << "\"Hey, listen up! I have been sent to invite you to meet me and two others at the Robotics lab in ";
                std::cout << "about " << (totalTimeEstimate - elapsedTime)/60 << " minutes. \n See you there!\"" << std::endl << std::endl;
                robot.Read();
                currentLocation = msrmnt::world_to_pixel(pp.GetXPos(),pp.GetYPos());

                leaveRoom(currentLocation,
                            currentRoom,
                            doorToRoomName,
                            roomNameToInOut,
                            pp,
                            robot,
                            linearSpeed,
                            angularSpeed,
                            matrix,
                            rp);

                std::cout << "CurrentLocation: " << std::get<1>(roomNameToInOut[currentRoom]).toString() << std::endl;
                std::cout << "Current Location based on odometry: " << msrmnt::world_to_pixel(pp.GetXPos(),pp.GetYPos()).toString() << std::endl;
                robot.Read();
                currentLocation = msrmnt::world_to_pixel(pp.GetXPos(),pp.GetYPos());
                currentRoom = "hallway";
            }


        }
        std::cout << std::endl;
        std::cout << "Robot: " << std::endl;
        std::cout << "\"So, I guess you're wondering why I gathered you all here today...\"";
        std::cout << std::endl;
        std::cout << "Get a load of this:" << std::endl;
        std::cout << "https://www.youtube.com/watch?v=dQw4w9WgXcQ&list=TLPQMjIwODIwMjNjsmgphzecSw&index=4" << std::endl;
        // After all waypoints have been reached and we're back at the starting location
        sleep(3);
        std::cout << "Ok, ok, I'm just kidding. I'm actually here to tell you that I'm done with my path. And this is probably more fitting: " << std::endl;
        std::cout << "https://www.youtube.com/watch?v=9p_Si21ig7c&t=91s" << std::endl;
        sleep(10);
        std::cout << "Robot: " << std::endl;
        std::cout << "\"Oh by the way, my original time estimate for the complete path was:" << totalTimeEstimate/60 << " minutes.\"" << std::endl;
        std::cout << "\"But I actually completed the path in: " << elapsedTime/60 << " minutes.\"" << std::endl;
        pp.SetSpeed(0, 0);
        pp.SetMotorEnable(false);
        return 0;
    } catch (const std::exception& ex) {
        std::cout << "Exception: " << ex.what() << std::endl;
        std::cerr << ex.what() << std::endl;
    }
}
