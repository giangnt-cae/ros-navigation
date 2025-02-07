#include "vk_costmap_2d/static_layer.hpp"

using vk_costmap_2d::NO_INFORMATION;
using vk_costmap_2d::LETHAL_OBSTACLE;
using vk_costmap_2d::FREE_SPACE;

namespace vk_costmap_2d {

StaticLayer::StaticLayer() {}

StaticLayer::~StaticLayer() {}

void StaticLayer::onInitialize() {
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle nh;
    updated_ = true;

    global_frame_ = layered_costmap_->getGlobalFrameID();
    std::string map_topic;
    private_nh.param("map_topic", map_topic, std::string("map"));
    private_nh.param("first_map_only", first_map_only_, false);
    private_nh.param("use_maximum", use_maximum_, false);

    int temp_lethal_threshold, temp_unknown_cost_value;
    private_nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
    private_nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
    lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
    unknown_cost_value_ = temp_unknown_cost_value;
    
    map_sub_ = nh.subscribe(map_topic, 1, &StaticLayer::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;
    ros::Rate r(10);
    while(!map_received_ && nh.ok()) {
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("Received a %d X %d map at %f m/pix",
             getSizeInCellsX(), getSizeInCellsY(), getResolution());
}

unsigned char StaticLayer::interpretValue(unsigned char value) {
    if(value == unknown_cost_value_) {
        return NO_INFORMATION;
    }else if(value >= lethal_threshold_) {
        return LETHAL_OBSTACLE;
    }else {
        return FREE_SPACE;
    }
    double scale = (double) value / lethal_threshold_;
    return scale * LETHAL_OBSTACLE;
}

void StaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& msg) {
    unsigned int size_x = msg->info.width;
    unsigned int size_y = msg->info.height;
    Costmap2D* master = layered_costmap_->getCostmap();
    if (!layered_costmap_->isRolling() && 
        (master->getSizeInCellsX() != size_x ||
        master->getSizeInCellsY() != size_y ||
        master->getResolution() != msg->info.resolution ||
        master->getOriginX() != msg->info.origin.position.x ||
        master->getOriginY() != msg->info.origin.position.y)) {
        // Update the size of the layered costmap (and all layers, including this one)
        layered_costmap_->resizeMap(size_x, size_y, msg->info.resolution,
                                    msg->info.origin.position.x, msg->info.origin.position.y, true);   
    }else if (size_x_ != size_x || size_y_ != size_y ||
              resolution_ != msg->info.resolution ||
              origin_x_ != msg->info.origin.position.x ||
              origin_y_ != msg->info.origin.position.y) {
        
        // only update the size of the costmap stored locally in this layer
        ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, msg->info.resolution);
        resizeMap(size_x, size_y, msg->info.resolution,
                  msg->info.origin.position.x, msg->info.origin.position.y);
    }
    
    unsigned int index = 0;
    for (unsigned int i = 0; i < size_y; ++i) {
        for (unsigned int j = 0; j < size_x; ++j) {
            unsigned char value = msg->data[index];
            costmap_[index] = interpretValue(value);
            ++index;
        }
    }
    map_frame_ = msg->header.frame_id;
    x0_ = y0_ = 0;
    width_ = size_x_;
    height_ = size_y_;
    map_received_ = true;
    has_updated_data_ = true;

    if (first_map_only_) {
        ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
        map_sub_.shutdown();
    }
}

void StaticLayer::matchSize() {
    if(!layered_costmap_->isRolling()) {
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }
}

void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y) {
    if( !layered_costmap_->isRolling() ){
        if (!map_received_ || !(has_updated_data_))
            return;
    }
    double wx, wy;
    mapToWorld(x0_, y0_, wx, wy);
    *min_x = std::min(wx, *min_x);
    *min_y = std::min(wy, *min_y);

    mapToWorld(x0_ + width_, y0_ + height_, wx, wy);
    *max_x = std::max(wx, *max_x);
    *max_y = std::max(wy, *max_y);

    has_updated_data_ = false;
}

void StaticLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j,
                              int max_i, int max_j) {
    if(!map_received_)
        return;
    if(!layered_costmap_->isRolling()) {
        if(!use_maximum_)
            updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
        else
            updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }else {
        unsigned int mx, my;
        double wx, wy;
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0));
        } catch (tf2::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
        tf2::Transform tf2_transform;
        tf2::convert(transform.transform, tf2_transform);
        for(unsigned int i = min_i; i < max_i; ++i) {
            for(unsigned int j = min_j; j < max_j; ++j) {
                // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
                layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
                // Transform from global_frame_ to map_frame_
                tf2::Vector3 p(wx, wy, 0);
                p = tf2_transform*p;
                // Set master_grid with cell from map
                if (worldToMap(p.x(), p.y(), mx, my)) {
                    if (!use_maximum_)
                        master_grid.setCost(i, j, getCost(mx, my));
                    else
                        master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
                }
            }
        }
    }                          

}

}