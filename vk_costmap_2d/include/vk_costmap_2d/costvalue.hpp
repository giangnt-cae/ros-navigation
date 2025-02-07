#pragma once

namespace vk_costmap_2d {

static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;

inline double distance(double x0, double y0, double x1, double y1) {
    return hypot(x1 - x0, y1 - y0);
}

inline double sign(double x) {
    return x < 0.0 ? -1.0 : 1.0;
}

inline double sign0(double x) {
    return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

}   // namespace vk_costmap_2d