#pragma once
#include "pathfinding.hpp"

using namespace Pathfinding;
namespace pf = Pathfinding;

namespace measurements {

    int world_to_pixel_x(double x, int image_width = 287, double world_width = 43.780);
    int world_to_pixel_y(double y, int image_height = 369, double world_height = 56.040);
    pf::Point world_to_pixel(double x, double y, int image_width = 287, int image_height = 369, double world_width = 43.780, double world_height = 56.040);

    double pixel_to_world_x(int px, int image_width = 287, double world_width = 43.780);
    double pixel_to_world_y(int py, int image_height = 369, double world_height = 56.040);
    void pixel_to_world(int px, int py, double& x, double& y, int image_width = 287, int image_height = 369, double world_width = 43.780, double world_height = 56.040);

    int world_distance_to_pixel_distance(double distance, int image_width = 287, double world_width = 43.780);
    double pixel_distance_to_world_distance(int pixel_distance, int image_width = 287, double world_width = 43.780);
    double calculate_pixel_distance(const pf::Point& a, const pf::Point& b);

} // namespace measurements
