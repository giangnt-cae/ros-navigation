#include "vk_costmap_2d/agv_layer.hpp"

namespace vk_costmap_2d {

AgvLayer::AgvLayer() :
    use_maximum_(true),
    agv_received_(false),
    min_x_(-std::numeric_limits<float>::max()),
    min_y_(-std::numeric_limits<float>::max()),
    max_x_(std::numeric_limits<float>::max()),
    max_y_(std::numeric_limits<float>::max()) {}

void AgvLayer::onInitialize() {
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle nh;
    rolling_window_ = layered_costmap_->isRolling();
    default_value_ = NO_INFORMATION;
    private_nh.param("id", id_, std::string("AGV-1709"));

    AgvLayer::matchSize();
    updated_ = true;
    agv_sub_ = nh.subscribe("agv_info", 10, &AgvLayer::errorAgvInfoCallback, this);
}

void AgvLayer::errorAgvInfoCallback(const AgvInfoArrayConstPtr& msg) {
    AgvInfo agv;
    std::vector<geometry_msgs::Point> polygon;
    std::vector<geometry_msgs::Point> footprint_spec;
    polygons_.clear();
    for(unsigned int i = 0; i < msg->agvs.size(); i++) {
        agv = msg->agvs[i];
        if(agv.id == id_) continue;
        ROS_INFO("%s at pose x: %.3f, y: %.3f, theta: %.3f!",
                agv.id.c_str(), agv.current_pose.position.x, agv.current_pose.position.y,
                tf2::getYaw(agv.current_pose.orientation));
        footprint_spec = makeFootprintFromWidthAndHeight(agv.length, agv.width);
        transformFootprint(agv.current_pose.position.x, agv.current_pose.position.y,
                           tf2::getYaw(agv.current_pose.orientation), footprint_spec, polygon);
        polygons_.push_back(polygon);
    }
    agv_received_ = true;
}

void AgvLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y,
                            double* max_x, double* max_y) {
    if(rolling_window_)
        updateOrigin(robot_x -getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    #ifdef updateBounds
    double tmp_min_x = min_x_;
    double tmp_min_y = min_y_;
    double tmp_max_x = max_x_;
    double tmp_max_y = max_y_;
    min_x_ = *min_x;
    min_y_ = *min_y;
    max_x_ = *max_x;
    max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
    #endif
    resetFullMap();
    for(const auto& polygon : polygons_) {
        if(!setConvexPolygonCost(polygon, LETHAL_OBSTACLE)) {
            continue;
        }
    }
    return;                             
}

void AgvLayer::updateCosts(Costmap2D& master_grid,
                           int min_i, int min_j,
                           int max_i, int max_j) {
    if(!agv_received_)
        return;
    if(!use_maximum_)
        updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}       // vk_costmap_2d namespace