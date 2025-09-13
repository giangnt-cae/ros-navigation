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
    private_nh.param("id", id_, std::string("952"));
    private_nh.param("footprint_padding", footprint_padding_, 0.05);
    lethal_threshold_ = 100;
    unknown_cost_value_ = -1;

    AgvLayer::matchSize();
    updated_ = true;

    marker_.header.frame_id = "map";
    marker_.header.stamp = ros::Time::now();
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_.scale.x = 0.02;
    marker_.scale.y = 0.02;
    marker_.scale.z = 0.02;
    marker_.ns = "vk_costmap_2d";
    marker_.color.r = 1.0;
    marker_.color.g = 0.0;
    marker_.color.b = 0.0;
    marker_.color.a = 2.0;
    marker_.lifetime = ros::Duration(0);
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.frame_locked = true; 

    agvs_sub_ = nh.subscribe("agvs_info", 10, &AgvLayer::errorAgvInfoCallback, this);
    footprints_pub_ = nh.advertise<visualization_msgs::MarkerArray>("footprint_markers", 10);
}

void AgvLayer::errorAgvInfoCallback(const AgvInfoArrayConstPtr& msg) {
    boost::recursive_mutex::scoped_lock lock(agv_access_);

    visualization_msgs::Marker delete_all_marker;
    delete_all_marker.action = visualization_msgs::Marker::DELETEALL;
    visualization_msgs::MarkerArray footprint_markers;
    marker_.id = 0;

    if(msg->agvs.empty())
        return;
    agv_arr.agvs.clear();
    for (const auto& agv : msg->agvs) {
        agv_arr.agvs.push_back(agv);
        marker_.header.frame_id = agv.footprint.header.frame_id;
        marker_.points.clear();
        if(agv.footprint.polygon.points.empty()) continue;
        for(const auto& point : agv.footprint.polygon.points) {
            marker_.points.push_back(toPoint(point));
        }
        marker_.points.push_back(toPoint(agv.footprint.polygon.points.front()));
        footprint_markers.markers.push_back(marker_);
        ++marker_.id;
    }
    footprints_pub_.publish(footprint_markers);

    agv_received_ = true;
}

void AgvLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y,
                            double* max_x, double* max_y) {
    if(rolling_window_) {
        Costmap2D* master_costmap = layered_costmap_->getCostmap();
        updateOriginAndResize(master_costmap->getOriginX(), master_costmap->getOriginY(),
                              master_costmap->getSizeInCellsX(), master_costmap->getSizeInCellsY());
    }

    double origin_x = origin_x_, origin_y = origin_y_;
    double map_end_x = origin_x + size_x_ * resolution_;
    double map_end_y = origin_y + size_y_ * resolution_;

    unsigned int x0, y0, other_x0, other_y0;
    
    boost::recursive_mutex::scoped_lock lock(agv_access_);
    for(const auto& agv : agv_arr.agvs) {
        double other_origin_x = agv.map.info.origin.position.x;
        double other_origin_y = agv.map.info.origin.position.y;
        double other_size_x   = agv.map.info.width;
        double other_size_y   = agv.map.info.height;
        double other_resolution = agv.map.info.resolution;

        if((other_resolution - resolution_) > 1e-3)
            ROS_WARN(" AGV %s has map resolution %f different with AGV %s has resolution %f",
                     agv.id.c_str(), other_resolution, id_.c_str(), resolution_);

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
                if(agv.map.data[other_index] == unknown_cost_value_)
                    continue;
                else if(agv.map.data[other_index] == lethal_threshold_)
                    temp_cost = LETHAL_OBSTACLE;
                else
                    temp_cost = FREE_SPACE;
                
                unsigned int index = getIndex(mx, my);
                unsigned char old_cost = costmap_[index];
                if(old_cost == NO_INFORMATION || old_cost < temp_cost) {
                    costmap_[index] = temp_cost;
                }
                
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
    boost::recursive_mutex::scoped_lock lock(agv_access_);
    std::vector<geometry_msgs::Point> m_polygon;
    for (const auto& agv : agv_arr.agvs) {
        if(agv.footprint.polygon.points.empty()) continue;
        std::vector<geometry_msgs::Point> polygon = getFootprint(agv.footprint);
        padFootprint(polygon, footprint_padding_);
        if(agv.id == id_) {
            m_polygon = polygon;
            continue;
        }
        setConvexPolygonCost(polygon, LETHAL_OBSTACLE);
    }
    setConvexPolygonCost(m_polygon, FREE_SPACE);

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

void AgvLayer::padFootprint(std::vector<geometry_msgs::Point>& footprint, double padding) {
    if(footprint.empty()) return;
    double cx = 0.0, cy = 0.0;
    for(const auto& pt : footprint) {
        cx += pt.x;
        cy += pt.y;
    }
    cx /= footprint.size();
    cy /= footprint.size();

    for(auto& pt : footprint) {
        double dx = pt.x - cx;
        double dy = pt.y - cy;
        double len = std::sqrt(dx * dx + dy * dy);
        if(len < 1e-6) continue;

        double scale = (len + padding) / len;
        pt.x = cx + dx * scale;
        pt.y = cy + dy * scale;
    }
}

}       // vk_costmap_2d namespace