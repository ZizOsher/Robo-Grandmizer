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

// using namespace boost;
// using namespace std;

namespace Pathfinding {

    struct Point {
        int x, y;
        bool operator==(const Point& other) const;
        std::string toString() const;
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

struct ComparePoints {
    bool operator()(const pf::Point& a, const pf::Point& b) const;
};

std::vector<pf::Point> reconstructPath(const std::map<pf::Point, pf::Point, ComparePoints>& cameFrom, const pf::Point& current);
std::vector<pf::Point> getNeighbors(const pf::Point& point, const Eigen::MatrixXd& matrix);
std::vector<pf::Point> astar(const Eigen::MatrixXd& matrix, const pf::Point& start, const pf::Point& target, std::string direction);
Eigen::MatrixXd loadMatrix(const std::string& filename);

#endif // PATHFINDING_HPP
