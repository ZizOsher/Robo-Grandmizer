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

void goToPoint(pf::Point& currentLocation,
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
    do {
        client.Read();

        currentWorldX = pp.GetXPos();
        currentWorldY = pp.GetYPos();
        currentLocation.x = msrmnt::world_to_pixel_x(currentWorldX);
        currentLocation.y = msrmnt::world_to_pixel_y(currentWorldY);

        std::cout << "Current Location: " << currentWorldX << ", " << currentWorldY << std::endl;
        std::cout << "Target Location: " << targetWorldX << ", " << targetWorldY << std::endl;

        dx = targetWorldX - currentWorldX;
        dy = targetWorldY - currentWorldY;
        distanceToTarget = sqrt(dx*dx + dy*dy);
        pixelDistance = msrmnt::world_distance_to_pixel_distance(distanceToTarget);
        std::cout << "Distance to Target: " << distanceToTarget << std::endl;
        
        if (pixelDistance > 1 || distanceToTarget > 0.5) {
            pp.SetSpeed(linearSpeed, 0);
        }
    
        double timeRequired = distanceToTarget / linearSpeed; // This gives time in seconds
        int sleepDuration = static_cast<int>(timeRequired * 1000000);  // Convert to microseconds
        usleep(sleepDuration/10000);
    } while(pixelDistance > 1 || distanceToTarget > 0.5);
    
    // Step 4: Stop the Robot
    pp.SetSpeed(0, 0); // Set linear speed to 0
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <linearSpeed> <angularSpeed>" << std::endl;
        return 1;
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

            std::vector<pf::Point> path = astar(matrix, currentLocation, target);

            path.insert(path.begin(), currentLocation);
            std::vector<pf::Point> orderlyPath = computeOrderlyPath(path, doorRoomMap, roomInOutMap);
            orderlyPath.erase(path.begin());

            std::vector<pf::Point> truePath = getTruePath(orderlyPath, currentLocation, matrix, 1);
            std::cout << "True Path: ";
            for (const pf::Point& point : truePath) {
                std::cout << point.toString() << ",";
            }
            std::cout << std::endl;
            
            for (const pf::Point& point : truePath) {
                goToPoint(currentLocation, pp, linearSpeed,angularSpeed, point, client);          
                // Update current location to the point just reached
                currentLocation = point;
            }
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
