#include "measurements.hpp"
#include <algorithm>
#include <cmath>
#include "pathfinding.hpp"

using namespace Pathfinding;
namespace pf = Pathfinding;

namespace measurements {

    inline double custom_round(double value) {
        return (value > 0.0) ? std::floor(value + 0.5) : std::ceil(value - 0.5);
    }

    int world_to_pixel_x(double x,
                            int image_width,
                            double world_width) {
        int px = static_cast<int>(custom_round((x / world_width + 0.5) * image_width));
        return std::max(0, std::min(px, image_width - 1));
    }

    int world_to_pixel_y(double y,
                            int image_height,
                            double world_height) {
        int py = static_cast<int>(custom_round((0.5 - y / world_height) * image_height));
        return std::max(0, std::min(py, image_height - 1));
    }

    pf::Point world_to_pixel(double x,
                                double y,
                                int image_width,
                                int image_height,
                                double world_width,
                                double world_height) {
        int px = world_to_pixel_x(x, image_width, world_width);
        int py = world_to_pixel_y(y, image_height, world_height);
        return {px, py};
    }

    double pixel_to_world_x(int px,
                            int image_width,
                            double world_width) {
        return custom_round((px / static_cast<double>(image_width) - 0.5) * world_width * 1000000) / 1000000;
    }

    double pixel_to_world_y(int py,
                            int image_height,
                            double world_height) {
        return custom_round((0.5 - py / static_cast<double>(image_height)) * world_height * 1000000) / 1000000;
    }

    void pixel_to_world(int px,
                        int py,
                        double& x,
                        double& y,
                        int image_width,
                        int image_height,
                        double world_width,
                        double world_height) {
        x = pixel_to_world_x(px, image_width, world_width);
        y = pixel_to_world_y(py, image_height, world_height);
    }

    int world_distance_to_pixel_distance(double distance,
                                            int image_width,
                                            double world_width) {
        return static_cast<int>(custom_round(distance * image_width / world_width));
    }

    double pixel_distance_to_world_distance(int pixel_distance,
                                            int image_width,
                                            double world_width) {
        return custom_round(pixel_distance * world_width / image_width * 1000000) / 1000000;
    }
} // namespace measurements
