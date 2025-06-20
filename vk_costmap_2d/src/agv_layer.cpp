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
    private_nh.param("id", id_, 952);
    lethal_threshold_ = 100;
    unknown_cost_value_ = -1;

    AgvLayer::matchSize();
    updated_ = true;
    agvs_sub_ = nh.subscribe("agvs_info", 10, &AgvLayer::errorAgvInfoCallback, this);
}

void AgvLayer::errorAgvInfoCallback(const AgvInfoArrayConstPtr& msg) {
    boost::recursive_mutex::scoped_lock lock(agv_access_);
    agv_arr.agvs.clear();
    for (const auto& agv : msg->agvs) {
        if(agv.id == id_) continue;
        agv_arr.agvs.push_back(agv);
    }
    agv_received_ = true;
}

void AgvLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y,
                            double* max_x, double* max_y) {
    if(rolling_window_)
        updateOrigin(robot_x -getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

    resetFullMap();
    double origin_x = origin_x_, origin_y = origin_y_;
    double map_end_x = origin_x + size_x_ * resolution_;
    double map_end_y = origin_y + size_y_ * resolution_;

    unsigned int x0, y0, other_x0, other_y0;

    for(const auto& agv : agv_arr.agvs) {
        double other_origin_x = agv.map.info.origin.position.x;
        double other_origin_y = agv.map.info.origin.position.y;
        double other_size_x   = agv.map.info.width;
        double other_size_y   = agv.map.info.height;
        double other_resolution = agv.map.info.resolution;

        if((other_resolution - resolution_) > 1e-3)
            ROS_WARN(" AGV %d has map resolution %f different with AGV %d has resolution %f", agv.id, other_resolution, id_, resolution_);

        double other_map_end_x = other_origin_x + other_size_x * other_resolution;
        double other_map_end_y = other_origin_y + other_size_y * other_resolution;

        double intersect_min_x = std::max(origin_x, other_origin_x);
        double intersect_max_x = std::min(map_end_x, other_map_end_x);
        double intersect_min_y = std::max(origin_y, other_origin_y);
        double intersect_max_y = std::min(map_end_y, other_map_end_y);

        if (intersect_min_x >= intersect_max_x || intersect_min_y >= intersect_max_y)
            continue;

        unsigned int width_cells = (intersect_max_x - intersect_min_x) / resolution_;
        unsigned int height_cells = (intersect_max_y - intersect_min_y) / resolution_;

        if(!worldToMap(intersect_min_x, intersect_min_y, x0, y0))
            continue;
        if(!worldToMapOther(intersect_min_x, intersect_min_y, other_x0, other_y0,
                            other_origin_x, other_origin_y, other_size_x, other_size_y, other_resolution))
            continue;

        for(unsigned int dx = 0; dx < width_cells; dx++) {
            for(unsigned int dy = 0; dy < height_cells; dy++) {
                unsigned int other_mx = other_x0 + dx, other_my = other_y0 + dy;
                unsigned int mx = x0 + dx, my = y0 + dy;
                if(other_mx >= other_size_x || other_my >= other_size_y ||
                   mx >= size_x_ || my >= size_y_)
                   continue;
                unsigned int other_index = other_my * other_size_x + other_mx;
                unsigned char temp_cost;
                if(agv.map.data[other_index] == unknown_cost_value_) {
                    continue;
                }else if(agv.map.data[other_index] == lethal_threshold_) {
                    temp_cost = LETHAL_OBSTACLE;
                }else {
                    temp_cost = FREE_SPACE;
                }
                
                unsigned int index = getIndex(mx, my);
                unsigned char old_cost = costmap_[index];
                if(old_cost == NO_INFORMATION || old_cost < temp_cost)
                    costmap_[index] = temp_cost;

                double wx = intersect_min_x + dx * resolution_;
                double wy = intersect_min_y + dy * resolution_;

                *min_x = std::min(*min_x, wx);
                *min_y = std::min(*min_y, wy);
                *max_x = std::max(*max_x, wx);
                *max_y = std::max(*max_y, wy);
            }
        }
    }                             
}

bool AgvLayer::worldToMapOther(double wx, double wy, unsigned int& mx, unsigned int& my,
                               double origin_x, double origin_y, double size_x, double size_y, double resolution) {
    if(wx < origin_x || wy < origin_y) return false;

    mx = (int)((wx - origin_x) / resolution);
    my = (int)((wy - origin_y) / resolution);

    if(mx < size_x && my < size_y) return true;

    return false;
}

void AgvLayer::updateCosts(Costmap2D& master_grid,
                           int min_i, int min_j,
                           int max_i, int max_j) {
    if(!agv_received_)
        return;

    for (const auto& agv : agv_arr.agvs) {
        if(agv.footprint.polygon.points.empty()) continue;
        std::vector<geometry_msgs::Point> polygon = getFootprint(agv.footprint);
        setConvexPolygonCost(polygon, LETHAL_OBSTACLE);
    }

    if(!use_maximum_)
        updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

std::vector<geometry_msgs::Point> AgvLayer::getFootprint(const geometry_msgs::PolygonStamped& footprint) {
    std::vector<geometry_msgs::Point> polygon;
    geometry_msgs::Point pt;
    for(const auto pt32 : footprint.polygon.points) {
        pt.x = pt32.x;
        pt.y = pt32.y;
        polygon.push_back(pt);
    }
    return polygon;
}

}       // vk_costmap_2d namespace