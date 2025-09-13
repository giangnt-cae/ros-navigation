#include "vk_costmap_2d/layered_costmap.hpp"

namespace vk_costmap_2d {

LayeredCostmap::LayeredCostmap (std::string global_frame, bool rolling_window) :
    master_costmap_(),
    global_frame_(global_frame),
    rolling_window_(rolling_window),
    updated_(false),
    min_x_(0.0),
    min_y_(0.0),
    max_x_(0.0),
    max_y_(0.0),
    bx0_(0),
    bxn_(0),
    by0_(0),
    byn_(0),
    initialized_(false),
    size_locked_(false),
    circumscribed_radius_(1.0),
    inscribed_radius_(0.2)
{
    master_costmap_.setDefaultValue(NO_INFORMATION);
}

LayeredCostmap::~LayeredCostmap() {
    while(layers_.size() > 0) {
        layers_.pop_back();
    }
}

void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                               double origin_x, double origin_y, bool size_locked) {
    boost::unique_lock<boost::recursive_mutex> lock(master_costmap_.configuration_mutex_);
    size_locked_ = size_locked;
    master_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
    for(std::vector<boost::shared_ptr<Layer>>::iterator layer = layers_.begin();
        layer != layers_.end(); ++layer) {
        (*layer)->matchSize();
    }
}

#ifdef RESIZE_MAP
void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw) {
    boost::unique_lock<boost::recursive_mutex> lock(master_costmap_.configuration_mutex_);
    if(rolling_window_) {
        double new_origin_x = robot_x - master_costmap_.getSizeInMetersX() / 2;
        double new_origin_y = robot_y - master_costmap_.getSizeInMetersY() / 2;
        master_costmap_.updateOrigin(new_origin_x, new_origin_y);
    }
    if(layers_.size() == 0) return;
    min_x_ = min_y_ = 1e30;
    max_x_ = max_y_ = -1e30;
    for(std::vector<boost::shared_ptr<Layer>>::iterator layer = layers_.begin();
        layer != layers_.end(); ++layer) {
        if(!(*layer)->isEnabled()) continue;
        (*layer)->updateBounds(robot_x, robot_y, robot_yaw,
                               &min_x_, &min_y_, &max_x_, &max_y_);
    }
    int x0, xn, y0, yn;
    master_costmap_.worldToMapEnforceBounds(min_x_, min_y_, x0, y0);
    master_costmap_.worldToMapEnforceBounds(max_x_, max_y_, xn, yn);
    
    if (xn < x0 || yn < y0) return;
    
    master_costmap_.resetRegionOfMap(x0, y0, xn, yn);
    for(std::vector<boost::shared_ptr<Layer>>::iterator layer = layers_.begin();
        layer != layers_.end(); ++layer) {
        if((*layer)->isEnabled())
            (*layer)->updateCosts(master_costmap_, x0, y0, xn, yn);
    }
    bx0_ = x0;
    bxn_ = xn;
    by0_ = y0;
    byn_ = yn;
    initialized_ = true;
}
#endif

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw,
                               double width, double height) {
    boost::unique_lock<boost::recursive_mutex> lock(master_costmap_.configuration_mutex_);
    if(rolling_window_) {
        double resolution = master_costmap_.getResolution();
        double new_origin_x, new_origin_y;
        double kx = 0.5, ky = 0.5;
        double k = std::max(width, height) / 0.95;
        if(width > height) {
            if(cos(robot_yaw) > 0) {
                kx = 1 / k;
            }else {
                kx = (k - 1) / k;
            }
        } else {
            if(sin(robot_yaw) > 0) {
                ky = 1 / k;
            }else {
                ky = (k - 1) / k;
            }
        }
        new_origin_x = robot_x - width * kx;
        new_origin_y = robot_y - height * ky;
        unsigned int new_size_x = width / resolution;
        unsigned int new_size_y = height / resolution;
        master_costmap_.updateOriginAndResize(new_origin_x, new_origin_y, new_size_x, new_size_y);

        master_costmap_.resizeMap(new_size_x, new_size_y, resolution, new_origin_x, new_origin_y);
        for(std::vector<boost::shared_ptr<Layer>>::iterator layer = layers_.begin();
            layer != layers_.end(); ++layer) {
            (*layer)->matchSize();
        }
    }
    if(layers_.size() == 0) return;
    min_x_ = min_y_ = 1e30;
    max_x_ = max_y_ = -1e30;
    for(std::vector<boost::shared_ptr<Layer>>::iterator layer = layers_.begin();
        layer != layers_.end(); ++layer) {
        if(!(*layer)->isEnabled()) continue;
        (*layer)->updateBounds(robot_x, robot_y, robot_yaw,
                               &min_x_, &min_y_, &max_x_, &max_y_);
    }
    int x0, xn, y0, yn;
    master_costmap_.worldToMapEnforceBounds(min_x_, min_y_, x0, y0);
    master_costmap_.worldToMapEnforceBounds(max_x_, max_y_, xn, yn);
    
    if (xn < x0 || yn < y0) return;
    
    master_costmap_.resetRegionOfMap(x0, y0, xn, yn);
    for(std::vector<boost::shared_ptr<Layer>>::iterator layer = layers_.begin();
        layer != layers_.end(); ++layer) {
        if((*layer)->isEnabled())
            (*layer)->updateCosts(master_costmap_, x0, y0, xn, yn);
    }
    bx0_ = x0;
    bxn_ = xn;
    by0_ = y0;
    byn_ = yn;
    initialized_ = true;
}

bool LayeredCostmap::isUpdated() {
    updated_ = true;
    for(std::vector<boost::shared_ptr<Layer>>::iterator layer = layers_.begin();
        layer != layers_.end(); ++layer) {
        if((*layer)->isEnabled()) updated_ = updated_ && (*layer)->isUpdated();
    }
    return updated_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec) {
  footprint_ = footprint_spec;
  vk_costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);
  for(std::vector<boost::shared_ptr<Layer> >::iterator layer = layers_.begin();
      layer != layers_.end(); ++layer) {
      (*layer)->onFootprintChanged();
  }
}


}   // namespace vk_costmap_2d