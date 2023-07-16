#include <iostream>
#include <libplayerc++/playerc++.h>
#include <opencv2/opencv.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <fstream>

using namespace boost;
using namespace std;
using namespace PlayerCc;
using namespace cv;

namespace Pathfinding {
    struct Point {
        int x, y;

        bool operator==(const Point& other) const {
            return x == other.x && y == other.y;
        }
    };

    struct Node {
        Point point;
        double cost; // Cost of reaching this node from the start.
        double priority;

        Node(Point point, double cost, double priority) : point(point), cost(cost), priority(priority) {}
    };

    struct ComparePriority {
        bool operator()(const Node& a, const Node& b) {
            return a.priority > b.priority;
        }
    };
}

struct ComparePoints {
    bool operator()(const Pathfinding::Point& a, const Pathfinding::Point& b) const {
        return std::tie(a.x, a.y) < std::tie(b.x, b.y);
    }
};


std::vector<Pathfinding::Point> reconstructPath(const std::map<Pathfinding::Point, Pathfinding::Point, ComparePoints>& cameFrom, const Pathfinding::Point& current);
std::vector<Pathfinding::Point> getNeighbors(const Pathfinding::Point& point, const Eigen::MatrixXd& matrix);

std::vector<Pathfinding::Point> astar(const Eigen::MatrixXd& matrix, const Pathfinding::Point& start, const Pathfinding::Point& target, std::string direction) {
    int rows = matrix.rows();
    int cols = matrix.cols();

    // Create a priority queue of nodes to explore.
    std::priority_queue<Pathfinding::Node, std::vector<Pathfinding::Node>, Pathfinding::ComparePriority> queue;

    // Create a map to store the best path to each node.
    std::map<Pathfinding::Point, Pathfinding::Point, ComparePoints> cameFrom;

    // Create a map to store the cost of reaching each node.
    std::map<Pathfinding::Point, double, ComparePoints> costSoFar;

    // Initialize the start node and add it to the queue.
    Pathfinding::Node startNode {start, 0.0, 0.0};
    queue.push(startNode);

    while (!queue.empty()) {
        // Get the node with the highest priority from the queue.
        Pathfinding::Node current = queue.top();
        queue.pop();

        // If we reached the target, reconstruct and return the path.
        if (current.point == target) {
            return reconstructPath(cameFrom, current.point);
        }

        // Check all neighboring nodes.
        for (const Pathfinding::Point& next : getNeighbors(current.point, matrix)) {
            double newCost = costSoFar[current.point] + 1; // Assume all movements have the same cost.
            if (direction == "right") {
                if (next.y < current.point.y || next.x != current.point.x) {
                    newCost += 1;
                }
            } else if (direction == "left") {
                if (next.y > current.point.y || next.x != current.point.x) {
                    newCost += 1;
                }
            } else if (direction == "up") {
                if (next.x > current.point.x || next.y != current.point.y) {
                    newCost += 1;
                }
            } else if (direction == "down") {
                if (next.x > current.point.x || next.y != current.point.y) {
                    newCost += 1;
                }
            }
            // If this node hasn't been visited yet, or if the new path is cheaper, update the data.
            if (!costSoFar.count(next) || newCost < costSoFar[next]) {
                costSoFar[next] = newCost;
                double priority = newCost;
                queue.push(Pathfinding::Node {next, newCost, priority});
                cameFrom[next] = current.point;
            }
        }
    }

    // If there's no path, return an empty path
    return {};
}

std::vector<Pathfinding::Point> getNeighbors(const Pathfinding::Point& point, const Eigen::MatrixXd& matrix) {
    std::vector<Pathfinding::Point> neighbors;
    std::vector<Pathfinding::Point> possibleMoves = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // Moves in 4 directions: up, down, left, right.

    for (const Pathfinding::Point& move : possibleMoves) {
        Pathfinding::Point next {point.x + move.x, point.y + move.y};

        // Check if the point is within the matrix boundaries and is not blocked.
        if (next.x >= 0 && next.x < matrix.rows() && next.y >= 0 && next.y < matrix.cols() && matrix(next.x, next.y) == 1) {
            neighbors.push_back(next);
        }
    }

    return neighbors;
}

std::vector<Pathfinding::Point> reconstructPath(const std::map<Pathfinding::Point, Pathfinding::Point, ComparePoints>& cameFrom, const Pathfinding::Point& current) {
    std::vector<Pathfinding::Point> path;
    Pathfinding::Point next = current;

    while (cameFrom.count(next) > 0) {
        path.push_back(next);
        next = cameFrom.at(next);
    }

    // Reverse the path to start from the beginning.
    std::reverse(path.begin(), path.end());

    return path;
}

// Matrix maker
Eigen::MatrixXd loadMatrix(const std::string& filename) {
    std::ifstream in(filename);
    std::vector<double> data;
    int rows = 0;
    int cols = 0;

    for (std::string line; std::getline(in, line);) {
        std::stringstream lineStream(line);
        double value;
        int rowSize = 0;

        while (lineStream >> value) {
            data.push_back(value);
            rowSize++;
        }

        if (rows == 0) { // save the number of columns from the first row
            cols = rowSize;
        } else if (rowSize != cols) { // all rows must have the same number of columns
            throw std::invalid_argument("All rows must have the same number of columns");
        }

        rows++;
    }

    // Copy the data to the Eigen matrix.
    Eigen::MatrixXd matrix(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            matrix(i, j) = data[i * cols + j];
        }
    }

    return matrix;
}


int main(int argc, char *argv[]) {
    Eigen::MatrixXd matrix = loadMatrix("../binary_image.txt");  
    // Initialize the client object.
    PlayerClient client("localhost");
    // Initialize the position2d proxy to control the robot.
    Position2dProxy pp(&client, 0);
    pp.SetMotorEnable(true);
    // Set the initial position of the robot.
    Pathfinding::Point currentLocation{76, 81};
    std::string direction = "right";

    while (true) {
        // Move the robot along the path.
        std::cout << "CurrentLocation: ";
        std::cout << currentLocation.x << ", " << currentLocation.y << std::endl;

        // Read data from the server.
        client.Read();

        // Get the target location from the user.
        Pathfinding::Point target;
        std::cout << "Enter the target coordinates (x y): ";
        std::cin >> target.x >> target.y;

        // Calculate the path from the current location to the target.
        std::string dup_direction = direction;
        std::vector<Pathfinding::Point> path = astar(matrix, currentLocation, target, direction);
        std::cout << "Path: ";
        for (const Pathfinding::Point& point : path) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
        
        direction = dup_direction;
        
        for (const Pathfinding::Point& point : path) {
//            std::cout << "X: " << point.x << " Y: " << point.y << endl;
            std::cout << "Direction: " << direction << std::endl;
            if (point.x > currentLocation.x) {
                if (direction == "down") {
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "up") {
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "right") {
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "left"){
                    pp.SetSpeed(0, 0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                }
                direction = "down";      
            } else if (point.x < currentLocation.x) {
                if (direction == "up") {
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "down") {
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "left") {
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "right"){
                    pp.SetSpeed(0, 0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                }
                direction = "up";
            } else if (point.y < currentLocation.y) {
                if (direction == "left") {
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "right") {
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "down") {
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "up"){
                    pp.SetSpeed(0, 0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                }
                direction = "left";
            } else if (point.y > currentLocation.y) {
                    if (direction == "right") {
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "left") {
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "up") {
                    pp.SetSpeed(0, -0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                } else if (direction == "down"){
                    pp.SetSpeed(0, 0.8);
                    usleep(2000000);
                    pp.SetSpeed(0.7, 0);
                }
                direction = "right";
            }
            // Update the current location.
            currentLocation = point;
            // Sleep for a while to give the robot time to move.
            usleep(180000);
        }
        pp.SetSpeed(0, 0);
    }

    // Disable the motors.
    pp.SetMotorEnable(false);

    return 0;
}

