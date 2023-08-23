#include "pathfinding.hpp"
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <opencv2/opencv.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <fstream>
#include <tuple>
#include <vector>
#include "measurements.hpp"

using namespace PlayerCc;
namespace pf = Pathfinding;
namespace msrmnt = Measurements;

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

int countOnesInRadius(const Eigen::MatrixXd& matrix, int y, int x, int radius) {
    int count = 0;

    // Define the bounds of the 2 cell radius
    int startY = std::max(0, y - radius);
    int endY = std::min(matrix.rows() - 1, long(y + radius));
    int startX = std::max(0, x - radius);
    int endX = std::min(matrix.cols() - 1, long(x + radius));

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

std::vector<pf::Point> getNeighbors(const pf::Point& point, const Eigen::MatrixXd& matrix) {
    std::vector<pf::Point> neighbors;
    std::vector<pf::Point> possibleMoves = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // Moves in 4 directions: up, down, left, right.

    for (const pf::Point& move : possibleMoves) {
        pf::Point next {point.x + move.x, point.y + move.y};
        if (next.x >= 0 && next.x < matrix.cols() && next.y >= 0 && next.y < matrix.rows() && matrix(next.y, next.x) == 0) {
            if (!countOnesInRadius(matrix, next.y, next.x, 2)) {
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

        std::vector<pf::Point> neighbors = getNeighbors(current.point, matrix);

        for (const pf::Point& nextPoint : neighbors) {
            if (closedList.count(nextPoint)) continue;  // Skip processing nodes in the closed list
            double newCost = costSoFar[current.point] + 1; // Assume all movements have the same cost.
            // proximity penalty
            int onesInRadius = countOnesInRadius(matrix, nextPoint.y, nextPoint.x, 4);
            newCost += onesInRadius * 100;

            double priority = newCost;

            if (!costSoFar.count(nextPoint) || newCost < costSoFar[nextPoint]) {
                costSoFar[nextPoint] = newCost;
                queue.push(pf::Node {nextPoint, newCost, priority});
                cameFrom[nextPoint] = current.point;
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
        if (countOnesInRadius(matrix, y, x, berth) > 0)
            return false;
    }
    return true;
}

bool tupleExistsInMap(const pf::Point& p1, const pf::Point& p2,
                      const std::map<std::string, std::tuple<pf::Point, pf::Point>>& m) {
    std::tuple<pf::Point, pf::Point> target1 = std::make_tuple(p1, p2);
    std::tuple<pf::Point, pf::Point> target2 = std::make_tuple(p2, p1);

    for (const auto& [key, value] : m) {
        if (value == target1 || value == target2) {
            return true;
        }
    }
    return false;
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

bool doesLineIntersectWithSegment(const pf::Point& p1,
                                  const pf::Point& p2,
                                  const pf::Point& q1,
                                  const pf::Point& q2) {
    // Check orientations of q1 and q2 with respect to the line defined by p1 and p2
    Orientation o1 = orientation(p1, p2, q1);
    Orientation o2 = orientation(p1, p2, q2);
    
    // If q1 and q2 have different orientations with respect to line p1-p2, they lie on different sides of the line.
    if (o1 != o2) return true;

    // Check for the special case where one of the segment's endpoints lies on the line
    if (o1 == Orientation::COLLINEAR && onSegment(p1, q1, p2)) return true;
    if (o2 == Orientation::COLLINEAR && onSegment(p1, q2, p2)) return true;

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
            if (entry.second == "330") continue;
            const auto& doorSegment = entry.first;
            const auto& roomInOut = RoomData.at(entry.second);

            if (path[i] == std::get<0>(roomInOut) || path[i] == std::get<1>(roomInOut)) {
                resPath.push_back(path[i]);
                i++;
                foundInAnyRectangle = true;
                break;
            }

            if (isInPolygon(std::get<0>(doorSegment), std::get<1>(doorSegment), 
                            std::get<0>(roomInOut), std::get<1>(roomInOut), path[i])) {

                // if the segment connecting the previous point to path[i] does not intersect with the door segment
                // and the segment connecting path[i] and std::get<1>(roomInOut) does not intersect with the door segment
                // The robot is outside the room and does not need to go through the door
                // all we need to do is add the current point to the result and move on to the next point (continue)
                if (i>0) {
                    if (!doSegmentsIntersect(path[i-1], path[i], std::get<0>(doorSegment), std::get<1>(doorSegment)) &&
                        !doSegmentsIntersect(path[i], std::get<1>(roomInOut), std::get<0>(doorSegment), std::get<1>(doorSegment))) {
                        resPath.push_back(path[i]);
                        i++;
                        foundInAnyRectangle = true;
                        break;
                    }
                }
                foundInAnyRectangle = true;

                // Add the current point and the closer point from roomInOut to the result
                resPath.push_back(path[i]);
                pf::Point closerPoint = (path[i].distance(std::get<0>(roomInOut)) < path[i].distance(std::get<1>(roomInOut)))
                                        ? std::get<0>(roomInOut) : std::get<1>(roomInOut);
                resPath.push_back(closerPoint);

                // Skip the subsequent points which are still in the same rectangle
                while (i < path.size() && isInPolygon(std::get<0>(doorSegment), std::get<1>(doorSegment),
                                                      std::get<0>(roomInOut), std::get<1>(roomInOut), path[i])) {
                    i++;
                }

                // If we haven't reached the end, add the farther point to resPath
                if (i < path.size()) {
                    pf::Point fartherPoint = (closerPoint == std::get<0>(roomInOut)) 
                                             ? std::get<1>(roomInOut) : std::get<0>(roomInOut);
                    resPath.push_back(fartherPoint);
                }

                break;
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
