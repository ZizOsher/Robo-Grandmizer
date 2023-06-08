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

namespace MyNamespace {
    struct Point {
        int x, y;

        bool operator==(const Point& other) const {
            return x == other.x && y == other.y;
        }
    };

    struct Node {
        Point point;
        double cost; // Cost of reaching this node from the start.
        double priority; // Priority in the queue (cost + heuristic estimate).

        Node(Point point, double cost, double priority) : point(point), cost(cost), priority(priority) {}
    };

    struct ComparePriority {
        bool operator()(const Node& a, const Node& b) {
            return a.priority > b.priority;
        }
    };
}

struct ComparePoints {
    bool operator()(const MyNamespace::Point& a, const MyNamespace::Point& b) const {
        return std::tie(a.x, a.y) < std::tie(b.x, b.y);
    }
};

double heuristic(const MyNamespace::Point& a, const MyNamespace::Point& b);
std::vector<MyNamespace::Point> reconstructPath(const std::map<MyNamespace::Point, MyNamespace::Point, ComparePoints>& cameFrom, const MyNamespace::Point& current);
std::vector<MyNamespace::Point> getNeighbors(const MyNamespace::Point& point, const Eigen::MatrixXd& matrix);

std::vector<MyNamespace::Point> astar(const Eigen::MatrixXd& matrix, const MyNamespace::Point& start, const MyNamespace::Point& target) {
    int rows = matrix.rows();
    int cols = matrix.cols();

    // Create a priority queue of nodes to explore.
    std::priority_queue<MyNamespace::Node, std::vector<MyNamespace::Node>, MyNamespace::ComparePriority> queue;

    // Create a map to store the best path to each node.
    std::map<MyNamespace::Point, MyNamespace::Point, ComparePoints> cameFrom;

    // Create a map to store the cost of reaching each node.
    std::map<MyNamespace::Point, double, ComparePoints> costSoFar;

    // Initialize the start node and add it to the queue.
    MyNamespace::Node startNode {start, 0.0, heuristic(start, target)};
    queue.push(startNode);

    while (!queue.empty()) {
        // Get the node with the highest priority from the queue.
        MyNamespace::Node current = queue.top();
        queue.pop();

        // If we reached the target, reconstruct and return the path.
        if (current.point == target) {
            return reconstructPath(cameFrom, current.point);
        }

        // Check all neighboring nodes.
        for (const MyNamespace::Point& next : getNeighbors(current.point, matrix)) {
            double newCost = costSoFar[current.point] + 1; // Assume all movements have the same cost.

            // If this node hasn't been visited yet, or if the new path is cheaper, update the data.
            if (!costSoFar.count(next) || newCost < costSoFar[next]) {
                costSoFar[next] = newCost;
                double priority = newCost + heuristic(next, target);
                queue.push(MyNamespace::Node {next, newCost, priority});
                cameFrom[next] = current.point;
            }
        }
    }

    // If there's no path, return an empty path
    return {};
}

std::vector<MyNamespace::Point> getNeighbors(const MyNamespace::Point& point, const Eigen::MatrixXd& matrix) {
    std::vector<MyNamespace::Point> neighbors;
    std::vector<MyNamespace::Point> possibleMoves = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // Moves in 4 directions: up, down, left, right.

    for (const MyNamespace::Point& move : possibleMoves) {
        MyNamespace::Point next {point.x + move.x, point.y + move.y};

        // Check if the point is within the matrix boundaries and is not blocked.
        if (next.x >= 0 && next.x < matrix.rows() && next.y >= 0 && next.y < matrix.cols() && matrix(next.x, next.y) >= 0) {
            neighbors.push_back(next);
        }
    }

    return neighbors;
}

double heuristic(const MyNamespace::Point& a, const MyNamespace::Point& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

std::vector<MyNamespace::Point> reconstructPath(const std::map<MyNamespace::Point, MyNamespace::Point, ComparePoints>& cameFrom, const MyNamespace::Point& current) {
    std::vector<MyNamespace::Point> path;
    MyNamespace::Point next = current;

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


double calculateTurnrate(const MyNamespace::Point& currentLocation, const MyNamespace::Point& target) {
    // Calculate angle to the target in radians
    double targetAngle = std::atan2(target.y - currentLocation.y, target.x - currentLocation.x);

    // Calculate current angle. Here we assume the robot starts facing to the right (0 rad). Adjust if necessary.
    double currentAngle = std::atan2(currentLocation.y, currentLocation.x);

    // Calculate the difference between the current and target angles
    double angleDiff = targetAngle - currentAngle;

    // Calculate turnrate based on the angle difference. Here we assume a simple linear relationship, you might need to adjust this.
    double turnrate = 0.1 * angleDiff;

    // Limit the maximum turnrate to a sensible value, for example 0.5 rad/s.
    turnrate = std::min(turnrate, 0.5);
    turnrate = std::max(turnrate, -0.5);

    return turnrate;
}

int main(int argc, char *argv[]) {
    // Load the matrix from the file.
    Eigen::MatrixXd matrix = loadMatrix("../binary_image.txt");
    
    // Initialize the client object.
    PlayerClient client("localhost");

    // Initialize the position2d proxy to control the robot.
    Position2dProxy pp(&client, 0);

    // Enable the motors.
    pp.SetMotorEnable(true);

    // Set the initial position of the robot.
    MyNamespace::Point currentLocation{21, 1015};
    cout << matrix(21,1015);

    while (true) {
        // Read data from the server.
        client.Read();

        // Get the target location from the user.
        MyNamespace::Point target;
        std::cout << "Enter the target coordinates (x y): ";
        std::cin >> target.x >> target.y;

        // Calculate the path from the current location to the target.
        std::vector<MyNamespace::Point> path = astar(matrix, currentLocation, target);
//        std::cout << "Path: ";
//        for (const MyNamespace::Point& point : path) {
//            std::cout << "(" << point.x << ", " << point.y << ") ";
//        }
//        std::cout << std::endl;

        // Move the robot along the path.
        int i = 0;
        for (const MyNamespace::Point& point : path) {
            // Calculate the speed and direction to get to the next point.
            double turnrate = calculateTurnrate(currentLocation, point);


            // Set the robot's speed and direction.
            pp.SetSpeed(1, turnrate);

            // Update the current location.
            currentLocation = point;

            // Sleep for a while to give the robot time to move.
            usleep(10000);
        }

        // Stop the robot at the target.
        pp.SetSpeed(0, 0);
    }

    // Disable the motors.
    pp.SetMotorEnable(false);

    return 0;
}

