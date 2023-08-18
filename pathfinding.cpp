#include "pathfinding.hpp"
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <opencv2/opencv.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <fstream>
// #include <nlohmann/json.hpp>
#include <tuple>
// #include "jsonOperations.hpp"
#include <vector>

// using namespace boost;
// using namespace std;
using namespace PlayerCc;
// using namespace cv;
namespace pf = Pathfinding;


bool pf::Point::operator==(const pf::Point& other) const {
    return x == other.x && y == other.y;
}

bool pf::Point::operator<(const pf::Point& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
}

bool pf::Point::operator!=(const pf::Point& other) const {
    return !(*this == other);
}

double pf::Point::distance(const Point& other) const {
    int dx = x - other.x;
    int dy = y - other.y;
    return std::sqrt(dx*dx + dy*dy);
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
            if (matrix(next.y, next.x) == 0 &&
                !(
                    matrix(next.y+1, next.x) == 1 ||
                    matrix(next.y, next.x+1) == 1 ||
                    matrix(next.y-1, next.x) == 1 ||
                    matrix(next.y, next.x-1) == 1 ||
                    matrix(next.y-1, next.x-1) == 1 ||
                    matrix(next.y-1, next.x+1) == 1 ||
                    matrix(next.y+1, next.x-1) == 1 ||
                    matrix(next.y+1, next.x+1) == 1)
                    ) {
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


int countOnesInRadius(const Eigen::MatrixXd& matrix, int y, int x) {
    int count = 0;

    // Define the bounds of the 2 cell radius
    int startY = std::max(0, y - 2);
    int endY = std::min(matrix.rows() - 1, long(y + 2));
    int startX = std::max(0, x - 2);
    int endX = std::min(matrix.cols() - 1, long(x + 2));

    // Loop through the surrounding cells
    for (int i = startY; i <= endY; ++i) {
        for (int j = startX; j <= endX; ++j) {
            if (i == y && j == x) // Skip the center cell itself
                continue;
            if (matrix(i, j) == 1) {
                count++;
            }
        }
    }

    return count;
}

std::vector<pf::Point> astar(const Eigen::MatrixXd& matrix,
                             const pf::Point& start,
                             const pf::Point& target) {
    int rows = matrix.rows();
    int cols = matrix.cols();

    std::priority_queue<pf::Node, std::vector<pf::Node>, pf::ComparePriority> queue;
    std::map<pf::Point, pf::Point, ComparePoints> cameFrom;
    std::map<pf::Point, double, ComparePoints> costSoFar;
    std::set<pf::Point, ComparePoints> closedList;

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

            //proximity penalty
            if (countOnesInRadius(matrix, next.y, next.x) > 0) {
                newCost += 10;
            }

            double priority = newCost;

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

bool isPathClear(const Eigen::MatrixXd& matrix,
                    const pf::Point& start,
                    const pf::Point& end,
                    int berth) {
    // This function checks if the straight line path from start to end is clear 
    // and not wider than b pixels, without any point where matrix(y,x) == 1.
    int dx = end.x - start.x;
    int dy = end.y - start.y;

    double length = std::sqrt(std::pow(start.x - end.x, 2) + std::pow(start.y - end.y, 2));

    for (double t = 0.0; t <= 1.0; t += 0.5 / length) {
        int x = static_cast<int>(start.x + t * dx);
        int y = static_cast<int>(start.y + t * dy);

        // Check the matrix value and width constraint
        if (matrix(y, x) == 1 ||
            matrix(y+1, x) == 1 ||
            matrix(y, x+1) == 1 ||
            matrix(y-1, x) == 1 ||
            matrix(y, x-1) == 1 ||
            matrix(y-1, x-1) == 1 ||
            matrix(y-1, x+1) == 1 ||
            matrix(y+1, x-1) == 1 ||
            matrix(y+1, x+1) == 1
            )
            return false;
    }
    return true;
}

std::vector<pf::Point> getTruePath(const std::vector<pf::Point>& path,
                                        pf::Point start,
                                        const Eigen::MatrixXd& matrix,
                                        int b) {
    std::vector<pf::Point> truePath;
    pf::Point currentPoint = start;
    size_t i = 0;
    while (i < path.size()) {
        size_t farthestReachable = i;
        for (size_t j = i; j < path.size(); j++) {
            if (isPathClear(matrix, currentPoint, path[j], b)) {
                farthestReachable = j;
            } else {
                break;
            }
        }

        currentPoint = path[farthestReachable];
        truePath.push_back(currentPoint);
        i = farthestReachable + 1;
    }

    return truePath;
}

enum class Orientation {
    COLLINEAR,
    CLOCKWISE,
    COUNTERCLOCKWISE
};

// Helper function to find the orientation of an ordered triplet (p, q, r).
Orientation orientation(const pf::Point& p,
                        const pf::Point& q,
                        const pf::Point& r) {
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    if (val == 0) return Orientation::COLLINEAR;
    return (val > 0) ? Orientation::CLOCKWISE : Orientation::COUNTERCLOCKWISE;
}

// Helper function to check if a point q lies on line segment 'pr'
bool onSegment(const pf::Point& p,
                const pf::Point& q,
                const pf::Point& r) {
    return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
           q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
}

bool doSegmentsIntersect(const pf::Point& p1,
                            const pf::Point& q1,
                            const pf::Point& p2,
                            const pf::Point& q2) {
    // Find the four orientations
    Orientation o1 = orientation(p1, q1, p2);
    Orientation o2 = orientation(p1, q1, q2);
    Orientation o3 = orientation(p2, q2, p1);
    Orientation o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4) return true;

    // Special cases for collinearity
    if (o1 == Orientation::COLLINEAR && onSegment(p1, p2, q1)) return true;
    if (o2 == Orientation::COLLINEAR && onSegment(p1, q2, q1)) return true;
    if (o3 == Orientation::COLLINEAR && onSegment(p2, p1, q2)) return true;
    if (o4 == Orientation::COLLINEAR && onSegment(p2, q1, q2)) return true;

    return false; // If none of the conditions are satisfied
}

bool isInPolygon(const pf::Point& p1, const pf::Point& p2, 
                 const pf::Point& q1, const pf::Point& q2, 
                 const pf::Point& r) {
    // Determine bounds of the rectangle
    int minX = std::min({p1.x, q1.x, p2.x, q2.x}) - 1;
    int maxX = std::max({p1.x, q1.x, p2.x, q2.x}) + 1;
    int minY = std::min({p1.y, q1.y, p2.y, q2.y}) - 1;
    int maxY = std::max({p1.y, q1.y, p2.y, q2.y}) + 1;

    // Check if r is inside the rectangle
    return r.x >= minX && r.x <= maxX && r.y >= minY && r.y <= maxY;
}

std::vector<pf::Point> computeOrderlyPath(
    std::vector<pf::Point>& path, 
    const std::map<std::tuple<pf::Point, pf::Point>, std::string>& RoomDoorMapping,
    const std::map<std::string, std::tuple<pf::Point, pf::Point>>& RoomData)
{
    std::vector<pf::Point> resPath;
    size_t i = 0;

    while (i < path.size()) {
        bool foundInAnyRectangle = false;

        for (const auto& entry : RoomDoorMapping) {
            const auto& doorSegment = entry.first;
            const auto& roomTuple = RoomData.at(entry.second);

            if (isInPolygon(std::get<0>(doorSegment), std::get<1>(doorSegment), 
                            std::get<0>(roomTuple), std::get<1>(roomTuple), path[i])) {

                foundInAnyRectangle = true;

                // Add the current point and the closer point from roomTuple to the result
                resPath.push_back(path[i]);
                pf::Point closerPoint = (path[i].distance(std::get<0>(roomTuple)) < path[i].distance(std::get<1>(roomTuple)))
                                        ? std::get<0>(roomTuple) : std::get<1>(roomTuple);
                resPath.push_back(closerPoint);

                // Skip the subsequent points which are still in the same rectangle
                while (i < path.size() && isInPolygon(std::get<0>(doorSegment), std::get<1>(doorSegment),
                                                      std::get<0>(roomTuple), std::get<1>(roomTuple), path[i])) {
                    i++;
                }

                // If we haven't reached the end, add the farther point to resPath
                if (i < path.size()) {
                    pf::Point fartherPoint = (closerPoint == std::get<0>(roomTuple)) 
                                             ? std::get<1>(roomTuple) : std::get<0>(roomTuple);
                    resPath.push_back(fartherPoint);
                }

                break;  // Exit the inner loop
            }
            //TODO: Add a check for the case of intersection
        }

        if (!foundInAnyRectangle) {
            resPath.push_back(path[i]);
            i++;
        }
    }

    // Ensure the last element of the path is added
    if (!resPath.empty() && path.back() != resPath.back()) {
        resPath.push_back(path.back());
    }

    return resPath;
}
