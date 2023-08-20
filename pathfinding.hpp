#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <map>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>

namespace Pathfinding {

    struct Point {
        int x, y;
        bool operator==(const Point& other) const;
        bool operator!=(const Point& other) const;
        bool operator<(const Point& other) const ;
        std::string toString() const;
        double distance(const Point& other) const;
    };

    struct Node {
        Point point;
        double cost; // Cost of reaching this node from the start.
        double priority;
        Node(Point point, double cost, double priority);
        std::string toString() const;
    };

    struct ComparePriority {
        bool operator()(const Node& a, const Node& b);
    };

} // namespace Pathfinding

namespace pf = Pathfinding;
// using nlohmann::json;

struct ComparePoints {
    bool operator()(const pf::Point& a, const pf::Point& b) const;
};

// double distance(const pf::Point& a, const pf::Point& b);
int countOnesInRadius(const Eigen::MatrixXd& matrix, int y, int x, int radius);
std::vector<pf::Point> reconstructPath(const std::map<pf::Point, pf::Point, ComparePoints>& cameFrom, const pf::Point& current);
std::vector<pf::Point> getNeighbors(const pf::Point& point, const Eigen::MatrixXd& matrix);
std::vector<pf::Point> astar(const Eigen::MatrixXd& matrix, const pf::Point& start, const pf::Point& target);
Eigen::MatrixXd loadMatrix(const std::string& filename);
bool isPathClear(const Eigen::MatrixXd& matrix, const pf::Point& start, const pf::Point& end, int b);
std::vector<pf::Point> getTruePath(const std::vector<pf::Point>& path, pf::Point start, const Eigen::MatrixXd& matrix, int b);

std::vector<pf::Point> computeOrderlyPath(
    std::vector<pf::Point>& path, 
                                            const std::map<std::tuple<pf::Point, pf::Point>, std::string>& RoomDoorMapping,
                                            const std::map<std::string, std::tuple<pf::Point, pf::Point>>& RoomData);

bool doSegmentsIntersect(const pf::Point& p1, const pf::Point& q1, const pf::Point& p2, const pf::Point& q2);
bool onSegment(const pf::Point& p, const pf::Point& q, const pf::Point& r);
#endif // PATHFINDING_HPP
