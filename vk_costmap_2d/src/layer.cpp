#include "vk_costmap_2d/layer.hpp"

namespace vk_costmap_2d {

const std::vector<geometry_msgs::Point>& Layer::getFootprint() const {
    return layered_costmap_->getFootprint();
}

}   // vk_costmap_2d namespace