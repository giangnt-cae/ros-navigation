#include <vk_global_planner/rrt_star.hpp>

namespace vk_global_planner {

rrtstar::rrtstar(tf2_ros::Buffer& tf)
:   tf_(tf), delta_(0.5),
    transform_tolerance_(0.5),
    max_size_(5000),
    base_frame_("base_link"),
    global_frame_("map"),
    free_(0)
{   
    map_sub_ = nh_.subscribe("/vk_costmap_2d/costmap", 1, &rrtstar::mapCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &rrtstar::goalCallback, this);

    planner_pub_ = nh_.advertise<nav_msgs::Path>("/path", 1);
    graph_pub_ = nh_.advertise<visualization_msgs::Marker>("/rrtstar_graph", 1);

    marker_node_.header.frame_id = global_frame_;
    marker_node_.header.stamp = ros::Time::now();
    marker_node_.action = visualization_msgs::Marker::ADD;
    marker_node_.type = visualization_msgs::Marker::SPHERE;
    marker_node_.id = 0;
    marker_node_.ns = "rrtstar";
    marker_node_.scale.x = 0.02;
    marker_node_.scale.y = 0.02;
    marker_node_.scale.z = 0.02;
    marker_node_.color.r = 1.0;
    marker_node_.color.g = 0.0;
    marker_node_.color.b = 0.0;
    marker_node_.color.a = 2.0;
    marker_node_.lifetime = ros::Duration(0);
    marker_node_.pose.orientation.x = 0.0;
    marker_node_.pose.orientation.y = 0.0;
    marker_node_.pose.orientation.z = 0.0;
    marker_node_.pose.orientation.w = 1.0;

    marker_edge_.header.frame_id = global_frame_;
    marker_edge_.header.stamp = ros::Time::now();
    marker_edge_.action = visualization_msgs::Marker::ADD;
    marker_edge_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_edge_.id = 0;
    marker_edge_.scale.x = 0.02;
    marker_edge_.scale.y = 0.02;
    marker_edge_.scale.z = 0.02;
    marker_edge_.ns = "rrtstar";
    marker_edge_.color.r = 0.0;
    marker_edge_.color.g = 1.0;
    marker_edge_.color.b = 0.0;
    marker_edge_.color.a = 2.0;
    marker_edge_.lifetime = ros::Duration(0);
    marker_edge_.pose.orientation.x = 0.0;
    marker_edge_.pose.orientation.y = 0.0;
    marker_edge_.pose.orientation.z = 0.0;
    marker_edge_.pose.orientation.w = 1.0;

    ros::spin();
}

bool rrtstar::rapidlyRandomTreeAlgorithm(geometry_msgs::PoseStamped& init_pose,
                                         geometry_msgs::PoseStamped& goal_pose) {
    unsigned int n = 0;
    rrtstar::Cell* cell_init = new rrtstar::Cell();
    cell_init->x = init_pose.pose.position.x;
    cell_init->y = init_pose.pose.position.y;
    cell_init->id = n;
    cell_init->cost = 0.0;
    tree_.push_back(cell_init);
    addNodeMarker(cell_init);
    
    rrtstar::Cell* cell_goal = new rrtstar::Cell();
    cell_goal->x = goal_pose.pose.position.x;
    cell_goal->y = goal_pose.pose.position.y;
    cell_goal->id = max_size_;

    rrtstar::CellCloud cloud;
    cloud.pts = tree_;
    rrtstar::KDTree index(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    index.buildIndex();
    
    n = 1;
    while (n < max_size_)
    {   
        rrtstar::Cell* cell = getCellSample(n);
        rrtstar::Cell* cell_nearest = getCellNearest(cell, cloud, index);
        if(!extend(cell, cell_nearest) || collisionCheck(cell, cell_nearest)) {
            delete cell;
            continue;
        }
        std::vector<rrtstar::Cell*> neighbors = getNeighbors(cell, cloud, index);

        std::vector<rrtstar::Cell*> filter_neighbors;
        double cmin = std::numeric_limits<double>::max();
        for(auto& neighbor : neighbors) {
            if(collisionCheck(cell, neighbor)) continue;
            filter_neighbors.push_back(neighbor);
            double c = neighbor->cost + norm2d(cell, neighbor);
            if(c < cmin) {
                cmin = c;
                cell->cost = cmin;
                cell->parent = neighbor;
            }
        }
        if (cell->parent == nullptr) {
            delete cell;
            continue;
        }
        tree_.push_back(cell);
        
        for(auto& neighbor : filter_neighbors) {
            double c = cell->cost + norm2d(neighbor, cell);
            if(c < neighbor->cost) {
                deleteMarker(neighbor);
                neighbor->parent = cell;
                neighbor->cost = c;
                addEdgeMarker(neighbor);
            }
        }
        n += 1;

        cloud.pts.push_back(cell);
        index.buildIndex();

        addNodeMarker(cell);
        addEdgeMarker(cell);
    }


    std::vector<rrtstar::Cell*> goal_neighbors = getNeighbors(cell_goal, cloud, index);

    std::vector<rrtstar::Cell*> filter_neighbors;
    double cmin = std::numeric_limits<double>::max();
    for(auto& neighbor : goal_neighbors) {
        if(collisionCheck(cell_goal, neighbor)) continue;
        filter_neighbors.push_back(neighbor);
        double c = neighbor->cost + norm2d(cell_goal, neighbor);
        if(c < cmin) {
            cmin = c;
            cell_goal->cost = cmin;
            cell_goal->parent = neighbor;
        }
    }
    if(filter_neighbors.empty())
        return false;
    tree_.push_back(cell_goal);
    addNodeMarker(cell_goal);
    addEdgeMarker(cell_goal);
    return true;
}

std::vector<rrtstar::Cell*> rrtstar::smoothingPath() {
    std::vector<rrtstar::Cell*> cells;
    rrtstar::Cell* cell = tree_.back();
    while (cell != nullptr) {
        cells.push_back(cell);
        cell = cell->parent;
    }
    
    std::vector<rrtstar::Cell*> filtered_cells;
    cell = cells.front();
    filtered_cells.push_back(cell);
    unsigned int N = 0;
    while (cell->parent != nullptr) {
        for (int i = cells.size() - 1; i > N; i--) {
            if (!collisionCheck(cell, cells[i])) {
                cell = cells[i];
                filtered_cells.push_back(cell);
                N = i;
                break;
            }
        }
    }
    return filtered_cells;
}

void rrtstar::resetTree() {
    for(auto& cell : tree_) {
        delete cell;
    }
    tree_.clear();
}

void rrtstar::addNodeMarker(rrtstar::Cell* cell) {
    marker_node_.id = marker_id_;
    marker_node_.pose.position.x = cell->x;
    marker_node_.pose.position.y = cell->y;
    marker_node_.pose.position.z = 0.0;
    graph_pub_.publish(marker_node_);
    marker_id_ += 1;
}

void rrtstar::addEdgeMarker(rrtstar::Cell* cell) {
    if(cell->parent != nullptr) {
        marker_edge_.id = marker_id_;
        marker_edge_.points.clear();
        geometry_msgs::Point pt;
        pt.x = cell->parent->x;
        pt.y = cell->parent->y;
        marker_edge_.points.push_back(pt);
        pt.x = cell->x;
        pt.y = cell->y;
        marker_edge_.points.push_back(pt);
        graph_pub_.publish(marker_edge_);

        edge_marker_ids_[cell] = marker_id_;
        ++marker_id_;
    }
}

void rrtstar::deleteMarker(rrtstar::Cell* cell) {
    if (edge_marker_ids_.find(cell) != edge_marker_ids_.end()) {
        int old_id = edge_marker_ids_[cell];

        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = global_frame_;
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.ns = "rrtstar";
        delete_marker.id = old_id;
        delete_marker.action = visualization_msgs::Marker::DELETE;
        graph_pub_.publish(delete_marker);
    }
}

void rrtstar::resetMarkers() {
    visualization_msgs::Marker delete_all_marker;
    delete_all_marker.action = visualization_msgs::Marker::DELETEALL;
    graph_pub_.publish(delete_all_marker);
    marker_id_ = 0;
    edge_marker_ids_.clear();
}

void rrtstar::goalCallback(const geometry_msgs::PoseStamped& msg) {
    if(!map_received_) return;
    boost::recursive_mutex::scoped_lock ml(configuration_mutex_);
    int num_cells_free = 0;
    for(const auto& data : map_.data) {
        if(data == free_)   num_cells_free += 1;
    }
    double lebesgue_mesure = num_cells_free * pow(map_.info.resolution, 2);
    gama_ = 3 * sqrt(3 * lebesgue_mesure / (M_PI * pow(1, 2)));

    geometry_msgs::PoseStamped goal_pose = msg;
    ROS_INFO("[RRT*] Received global goal: (x = %.2f, y = %.2f, theta = %.2f)",
             msg.pose.position.x,
             msg.pose.position.y,
             tf2::getYaw(msg.pose.orientation));

    geometry_msgs::PoseStamped init_pose;
    if(!getCurrentPose(init_pose))
        return;
    
    resetMarkers();
    
    if(rapidlyRandomTreeAlgorithm(init_pose, goal_pose)) {
        ROS_INFO("The path planner is sucessful!");
        std::vector<rrtstar::Cell*> cells = smoothingPath();
        nav_msgs::Path path;
        getPathPlanner(path, cells);
        resetTree();
    } else
        ROS_WARN("The path planner is failed!");
}

void rrtstar::getPathPlanner(nav_msgs::Path& path, std::vector<rrtstar::Cell*>& cells) {
    path.poses.clear();
    path.header.frame_id = global_frame_;
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pt;
    for (const auto& cell : cells) {
        geometry_msgs::PoseStamped pt;
        pt.pose.position.x = cell->x;
        pt.pose.position.y = cell->y;
        pt.pose.position.z = 0.0;
        path.poses.push_back(pt);
    }
    std::reverse(path.poses.begin(), path.poses.end());
    planner_pub_.publish(path);
}

void rrtstar::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
    boost::recursive_mutex::scoped_lock ml(configuration_mutex_);
    map_ = *msg;
    map_received_ = true;
}

bool rrtstar::getCurrentPose(geometry_msgs::PoseStamped& global_pose) {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);

    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = base_frame_;
    robot_pose.header.stamp = ros::Time();
    ros::Time current_time = ros::Time::now();

    try {
        if(tf_.canTransform(global_frame_, base_frame_, current_time)) {
            geometry_msgs::TransformStamped transform = tf_.lookupTransform(global_frame_, base_frame_, current_time);
            tf2::doTransform(robot_pose, global_pose, transform);
        }else {
            tf_.transform(robot_pose, global_pose, global_frame_);
        }
    }catch (tf2::LookupException& ex) {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }catch (tf2::ConnectivityException& ex) {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }catch (tf2::ExtrapolationException& ex) {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    
    if (!global_pose.header.stamp.isZero() && current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_) {
        ROS_WARN_THROTTLE(1.0,
                      "Transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
        return false;
    }
    return true;
}

rrtstar::Cell* rrtstar::getCellSample(unsigned int n) {
    double xmin = map_.info.origin.position.x;
    double xmax = map_.info.width * map_.info.resolution + map_.info.origin.position.x; 
    double ymin = map_.info.origin.position.y;
    double ymax = map_.info.height * map_.info.resolution + map_.info.origin.position.y;
    rrtstar::Cell* cell = new rrtstar::Cell();
    std::random_device s;
    std::mt19937 rng(s());
    std::uniform_real_distribution<double> uni_x(xmin, xmax);
    std::uniform_real_distribution<double> uni_y(ymin, ymax);
    unsigned int index = 0;
    do {
        cell->x = uni_x(rng);
        cell->y = uni_y(rng);
        cell->id = n;
        index = getIndex(cell->x, cell->y);
        if(index >= map_.data.size()) continue;
    }while(map_.data[index] != free_);
    return cell;
}

rrtstar::Cell* rrtstar::getCellNearest(rrtstar::Cell* query_cell, 
                                       rrtstar::CellCloud& cloud, 
                                       rrtstar::KDTree& index) {
    double query[2] = { query_cell->x, query_cell->y };
    size_t ret_index;
    double out_dist_sqr;

    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index, &out_dist_sqr);
    index.findNeighbors(resultSet, query, nanoflann::SearchParameters());

    return cloud.pts[ret_index];
}

bool rrtstar::extend(rrtstar::Cell* cell, rrtstar::Cell* cell_nearest) {
    if(norm2d(cell, cell_nearest) > delta_) {
        double k = delta_ / norm2d(cell, cell_nearest);
        cell->x = k*cell->x + (1.0 - k) * cell_nearest->x;
        cell->y = k*cell->y + (1.0 - k) * cell_nearest->y;
    }
    unsigned int index = getIndex(cell->x, cell->y);
    if(map_.data[index] == free_)
        return true;
    else
        return false;
}

unsigned int rrtstar::getIndex(double& x, double& y) {
    unsigned int mx, my;
    mx = static_cast<unsigned int>((x - map_.info.origin.position.x) / map_.info.resolution);
    my = static_cast<unsigned int>((y - map_.info.origin.position.y) / map_.info.resolution);
    return my *map_.info.width + mx;
}

bool rrtstar::collisionCheck(rrtstar::Cell* cell, rrtstar::Cell* cell_nearest) {
    double dx = cell->x - cell_nearest->x;
    double dy = cell->y - cell_nearest->y;
    double dist = std::sqrt(dx*dx + dy*dy);

    int steps = static_cast<int>(dist / map_.info.resolution);
    if(steps == 0)
        return false;
    
    for (int i = 0; i <= steps; ++i) {
        double x = cell_nearest->x + i * dx / steps;
        double y = cell_nearest->y + i * dy / steps;

        unsigned int idx = getIndex(x, y);
        if (map_.data[idx] != free_)
            return true;
    }
    return false;
}

double rrtstar::getRadiusNeighbor(int n) {
    if (n <= 1) return delta_;
    return std::min(gama_ * std::sqrt(std::log(n) / n), delta_);
}

std::vector<rrtstar::Cell*> rrtstar::getNeighbors(rrtstar::Cell* query_cell,
                                                  rrtstar::CellCloud& cloud,
                                                  rrtstar::KDTree& index)
{
    std::vector<rrtstar::Cell*> neighbors;
    double radius = getRadiusNeighbor(tree_.size());
    double query[2] = { query_cell->x, query_cell->y };
    double radius_squared = radius * radius;

    std::vector<nanoflann::ResultItem<unsigned int, double>> matches;
    nanoflann::SearchParameters params;
    params.sorted = true;
    size_t num_found = index.radiusSearch(query, radius_squared, matches, params);

    neighbors.reserve(num_found);
    for (const auto& m : matches) {
        neighbors.push_back(cloud.pts[m.first]);
    }
    return neighbors;
}

}   // vk_global_planner namespace
