#include "pathfinding.hpp"
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <opencv2/opencv.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <fstream>

// using namespace boost;
// using namespace std;
using namespace PlayerCc;
using namespace cv;
namespace pf = Pathfinding;

bool pf::Point::operator==(const pf::Point& other) const {
    return x == other.x && y == other.y;
}

pf::Node::Node(Point p, double c, double pr) : point(p), cost(c), priority(pr) {}

std::string pf::Point::toString() const {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }

std::string pf::Node::toString() const {
    return "Point: " + point.toString() + ", Cost: " + std::to_string(cost) + ", Priority: " + std::to_string(priority);
}

bool pf::ComparePriority::operator()(const pf::Node& a, const pf::Node& b) {
    return a.priority > b.priority;
}

bool ComparePoints::operator()(const pf::Point& a, const pf::Point& b) const {
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

std::vector<pf::Point> getNeighbors(const pf::Point& point, const Eigen::MatrixXd& matrix) {
    std::vector<pf::Point> neighbors;
    std::vector<pf::Point> possibleMoves = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // Moves in 4 directions: up, down, left, right.

    for (const pf::Point& move : possibleMoves) {
        pf::Point next {point.x + move.x, point.y + move.y};

        if (next.x >= 0 && next.x < matrix.cols() && next.y >= 0 && next.y < matrix.rows()) {
            if (matrix(next.y, next.x) == 0) {
                neighbors.push_back(next);
            }
        }
    }
    return neighbors;
}

std::vector<pf::Point> reconstructPath(const std::map<pf::Point, pf::Point, ComparePoints>& cameFrom,
                                        const pf::Point& current) {
    std::vector<pf::Point> path;
    pf::Point next = current;

    while (cameFrom.count(next) > 0) {
        path.push_back(next);
        next = cameFrom.at(next);
    }

    // Reverse the path to start from the beginning.
    std::reverse(path.begin(), path.end());

    return path;
}

double heuristic(pf::Point a, pf::Point b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

std::vector<pf::Point> astar(const Eigen::MatrixXd& matrix,
                             const pf::Point& start,
                             const pf::Point& target,
                             std::string direction) {
    int rows = matrix.rows();
    int cols = matrix.cols();

    std::priority_queue<pf::Node, std::vector<pf::Node>, pf::ComparePriority> queue;
    std::map<pf::Point, pf::Point, ComparePoints> cameFrom;
    std::map<pf::Point, double, ComparePoints> costSoFar;
    std::set<pf::Point, ComparePoints> closedList;  // The closed set

    costSoFar[start] = 0;
    pf::Node startNode {start, 0.0, heuristic(start, target)};
    queue.push(startNode);

    while (!queue.empty()) {
        pf::Node current = queue.top();
        queue.pop();

        // If the node has already been processed, continue to the next iteration
        if (closedList.count(current.point)) continue;

        if (current.point == target) {
            return reconstructPath(cameFrom, current.point);
        }

        closedList.insert(current.point);  // Mark the current node as processed

        for (const pf::Point& next : getNeighbors(current.point, matrix)) {
            if (closedList.count(next)) continue;  // Skip processing nodes in the closed list

            double newCost = costSoFar[current.point] + 1; // Assume all movements have the same cost.
            
            // Directional penalties
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

            double priority = newCost + heuristic(next, target);

            if (!costSoFar.count(next) || newCost < costSoFar[next]) {
                costSoFar[next] = newCost;
                queue.push(pf::Node {next, newCost, priority});
                cameFrom[next] = current.point;
            }
        }
    }
    return {};
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
