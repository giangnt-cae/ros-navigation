#include <graph_based_slam/graph_based_slam.hpp>
#include <graph_based_slam/optimization.hpp>
#include <graph_based_slam/detect_loop.hpp>

namespace slam2d {

GraphBasedSlam::GraphBasedSlam() : sm_(NULL),
                                   map_(NULL)
{
    init();
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
    graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("graph", 2);
    covariance_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("covariance", 1);

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_(nh_, "odom", 10);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_(nh_, "scan1", 10);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub_, laser_scan_sub_);
    sync.registerCallback(&GraphBasedSlam::dataCallback, this);

    sm_ = new ScanMatcher();
    map_ = new Map2D(map_size_x_, map_size_y_, resolution_, origin_x_, origin_y_, default_value_);
    f_init_ = true;

    ros::spin();
}

void GraphBasedSlam::init() {
    nh_.param("map_frame", map_frame_, std::string("map"));
    nh_.param("base_frame", base_frame_, std::string("vkbot"));
    nh_.param("odom_frame", odom_frame_, std::string("odom"));
    nh_.param("model_type", model_type_, std::string("omni"));

    nh_.param("min_trans", min_trans_, 0.3);
    nh_.param("min_rot", min_rot_, 0.2);
    nh_.param("min_range", min_range_, 0.05);
    nh_.param("max_range", max_range_, 15.0);
    nh_.param("angle_min", angle_min_, -M_PI / 2);
    nh_.param("angle_max", angle_max_, M_PI / 2);

    nh_.param("throttle_scan", throttle_scan_, 1);
    nh_.param("max_beams", max_beams_, 540);
    nh_.param("inverted_laser", inverted_laser_, true);

    nh_.param("max_distance_threshold", max_distance_threshold_, 0.5);
    int tmp;
    nh_.param("map_size_x", tmp, 1000);
    map_size_x_ = tmp;
    nh_.param("map_size_y", tmp, 500);
    map_size_y_ = tmp;
    nh_.param("resolution", resolution_, 0.05);
    nh_.param("origin_x", origin_x_, -0.5 * map_size_x_ * resolution_);
    nh_.param("origin_y", origin_y_, -0.5 * map_size_y_ * resolution_);
    nh_.param("delta", tmp, 100);
    delta_ = tmp;
    default_value_ = NO_INFORMATION;

    marker_node_.header.frame_id = map_frame_;
    marker_node_.header.stamp = ros::Time::now();
    marker_node_.action = visualization_msgs::Marker::ADD;
    marker_node_.type = visualization_msgs::Marker::SPHERE;
    marker_node_.id = 0;
    marker_node_.ns = "slam2d";
    marker_node_.scale.x = 0.1;
    marker_node_.scale.y = 0.1;
    marker_node_.scale.z = 0.1;
    marker_node_.color.r = 1.0;
    marker_node_.color.g = 0.0;
    marker_node_.color.b = 0.0;
    marker_node_.color.a = 2.0;
    marker_node_.lifetime = ros::Duration(0);
    marker_node_.pose.orientation.x = 0.0;
    marker_node_.pose.orientation.y = 0.0;
    marker_node_.pose.orientation.z = 0.0;
    marker_node_.pose.orientation.w = 1.0;

    marker_edge_.header.frame_id = map_frame_;
    marker_edge_.header.stamp = ros::Time::now();
    marker_edge_.action = visualization_msgs::Marker::ADD;
    marker_edge_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_edge_.id = 0;
    marker_edge_.scale.x = 0.02;
    marker_edge_.scale.y = 0.02;
    marker_edge_.scale.z = 0.02;
    marker_edge_.ns = "slam2d";
    marker_edge_.color.r = 0.0;
    marker_edge_.color.g = 1.0;
    marker_edge_.color.b = 0.0;
    marker_edge_.color.a = 2.0;
    marker_edge_.lifetime = ros::Duration(0);
    marker_edge_.pose.orientation.x = 0.0;
    marker_edge_.pose.orientation.y = 0.0;
    marker_edge_.pose.orientation.z = 0.0;
    marker_edge_.pose.orientation.w = 1.0;
}

void GraphBasedSlam::dataCallback(const nav_msgs::Odometry& msg1,
                                  const sensor_msgs::LaserScan& msg2) {
    odom_t_ = {msg1.pose.pose.position.x,
               msg1.pose.pose.position.y,
               tf2::getYaw(msg1.pose.pose.orientation)};
    
    scan_.clear();
    int inv_scan =  sign(inverted_laser_);
    int ibegin = (angle_min_ - msg2.angle_min) / msg2.angle_increment;
    int iend   = (angle_max_ - msg2.angle_min) / msg2.angle_increment;
    
    std::vector<Eigen::Vector2d> points;
    Eigen::Vector2d beam;
    for(int i = ibegin; i < iend; i++) {
        if(msg2.ranges[i] < min_range_ || msg2.ranges[i] > max_range_) continue;
        beam[0] = msg2.ranges[i];
        beam[1] = inv_scan * (i * msg2.angle_increment + msg2.angle_min);
        scan_.push_back(beam);

        if((i - ibegin) % (throttle_scan_ + 1)) {
            double angle = pose_lidar[2] + beam[1];
            Eigen::Vector2d point = pose_lidar.head<2>() + beam[0] * Eigen::Vector2d(cos(angle), sin(angle));
            points.push_back(point);
        }
    }
    
    if(f_init_) {
        robot_pose_ = initial_pose;
        node_  = new Node();
        edge_  = new Edge();
        graph_ = new Graph();

        node_->pose = robot_pose_;
        node_->scan = scan_;
        node_->points = points;
        node_->id   = 0;
        node_->omega << 1e3,   0,   0,
                        0  , 1e3,   0,
                        0  ,   0, 1e3;
        graph_->nodes.push_back(*node_);
        
        rayCasting(node_, map_);
        visualization();
        
        odom_t_1_ = odom_t_;
        f_odom_pose_ = odom_t_;
        f_init_ = false;
    }

    updateMotion(odom_t_1_, odom_t_, robot_pose_);
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    delta.head(2) = odom_t_.head(2) - f_odom_pose_.head(2);
    delta[2] = angle_diff(odom_t_[2], f_odom_pose_[2]);
    
    // See if we should update the filter
    bool update = fabs(delta[0]) > min_trans_ ||
                  fabs(delta[1]) > min_trans_ ||
                  fabs(delta[2]) > min_rot_;
    if(update) {
        /* Scan to map */
        // Eigen::Affine2d initial_guess = XYZEulertoAffineMatrix(robot_pose_);
        // Eigen::Affine2d T_t = sm_->registration(points, map_, initial_guess, max_distance_threshold_);

        /* Scan to scan */
        bool successful;
        Eigen::Affine2d Ti = XYZEulertoAffineMatrix(node_->pose);
        Eigen::Affine2d Tj = XYZEulertoAffineMatrix(robot_pose_);
        Eigen::Affine2d initial_guess = Ti.inverse() * Tj;
        Eigen::Matrix3d H;
        Eigen::Affine2d Tij = sm_->registration(points, node_->points, initial_guess, max_distance_threshold_, H, successful);
        Eigen::Affine2d T_t = Ti * Tij;
        
        /*-------------------------------------------------*/
        robot_pose_.head(2) = T_t.translation();
        robot_pose_[2] = atan2(T_t(1, 0), T_t(0, 0));
        
        node_->pose = robot_pose_;
        node_->scan = scan_;
        node_->points = points;
        node_->id++;
        graph_->nodes.push_back(*node_);

        edge_->z  = Tij;
        edge_->omega = H;
        edge_->ni = node_->id - 1;
        edge_->nj = node_->id;
        graph_->edges.push_back(*edge_);
        
        /* Expand map */
        // unsigned int mx, my;
        // if (map_->worldToMap(robot_pose_[0], robot_pose_[1], mx, my)) {
        //     unsigned int new_size_x = map_->getSizeInCellsX();
        //     unsigned int new_size_y = map_->getSizeInCellsY();
    
        //     bool expand_x = std::min(mx, new_size_x - mx - 1) < delta_;
        //     bool expand_y = std::min(my, new_size_y - my - 1) < delta_;
            
        //     if (expand_x || expand_y) {
        //         if (expand_x) new_size_x += delta_;
        //         if (expand_y) new_size_y += delta_;
        
        //         map_->resizeMap(new_size_x, new_size_y, map_->getResolution(), map_->getOriginX(), map_->getOriginY());
        //     }
        // }

        // rayCasting(node_, map_);
        bool loop_closure = false;
        if(computeInformationMatrix(graph_)) {
            for(int i = 0; i < graph_->nodes.size() - 1; i++) {
                double d = mahalanobisDistance(graph_->nodes[i], graph_->nodes.back());
                if(d <= MAX_MALAHANOBIS_DISTANCE) {
                    std::cout << "d: " << d << "; N: " << graph_->nodes.size() << std::endl;
                    Tij = sm_->registration(graph_->nodes[i].points, graph_->nodes.back().points, initial_guess, max_distance_threshold_, H, successful);
                    if(successful) {
                        edge_->z  = Tij;
                        edge_->omega = H;
                        edge_->ni = graph_->nodes[i].id;
                        edge_->nj = graph_->nodes.back().id;
                        graph_->edges.push_back(*edge_);
                        loop_closure = true;
                    }
                }
            }
        }
        if(loop_closure)
            graphOptimization(graph_);
        visualization();

        f_odom_pose_ = odom_t_;
    }
    odom_t_1_ = odom_t_;
}

void GraphBasedSlam::visualization() {
    #ifdef UpdateMap2D
    nav_msgs::OccupancyGrid ros_map;
    ros_map.header.stamp = ros::Time::now();
    ros_map.header.frame_id = map_frame_;
    ros_map.info.resolution = map_->getResolution();
    ros_map.info.width = map_->getSizeInCellsX();
    ros_map.info.height = map_->getSizeInCellsY();
    ros_map.info.origin.position.x = map_->getOriginX();
    ros_map.info.origin.position.y = map_->getOriginY();
    
    int num_cells = ros_map.info.width * ros_map.info.height;
    ros_map.data.resize(num_cells);
    for(int i = 0; i < num_cells; i++) {
        int logodd = map_->getIntMap()[i];
        if(logodd < FREE_SPACE)
            ros_map.data[i] = 0;
        else if(logodd > OCCUPIED)
            ros_map.data[i] = 100;
        else
            ros_map.data[i] = -1;
    }
    map_pub_.publish(ros_map);
    #endif
    
    int num_nodes = graph_->nodes.size();
    int id = 0;
    marker_graph_.markers.clear();
    for(const auto& node : graph_->nodes) {
        marker_node_.id = id;
        marker_node_.pose.position.x = node.pose[0];
        marker_node_.pose.position.y = node.pose[1];
        marker_node_.pose.position.z = 0.0;
        marker_graph_.markers.push_back(marker_node_);
        id++;
    }
    
    for(const auto& edge : graph_->edges) {
        marker_edge_.id = id;
        marker_edge_.points.clear();
        geometry_msgs::Point p;

        int i = edge.ni, j = edge.nj;
        p.x = graph_->nodes[i].pose[0];
        p.y = graph_->nodes[i].pose[1];
        marker_edge_.points.push_back(p);

        p.x = graph_->nodes[j].pose[0];
        p.y = graph_->nodes[j].pose[1];
        marker_edge_.points.push_back(p);

        marker_graph_.markers.push_back(marker_edge_);
        id++;
    }

    #ifdef Elipse
    if(!computeInformationMatrix(graph_))
        return;

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Elipse";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    for(const auto& node : graph_->nodes) {
        marker.id = id;
        marker.pose.position.x = node.pose[0];
        marker.pose.position.y = node.pose[1];
        marker.pose.position.z = 0;
        Eigen::Matrix2d P = node.omega.block<2,2>(0,0);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(P.inverse());
        Eigen::Vector2d eigenvalues = solver.eigenvalues();
        Eigen::Matrix2d eigenvectors = solver.eigenvectors();

        double axis_length_1 = 2 * std::max(0.01, sqrt(std::max(0.0, eigenvalues(1))));
        double axis_length_2 = 2 * std::max(0.01, sqrt(std::max(0.0, eigenvalues(0))));

        double angle = atan2(eigenvectors(1, 1), eigenvectors(0, 1));

        if (fabs(eigenvectors(0, 1)) > fabs(eigenvectors(1, 1))) {
            marker.scale.x = axis_length_1;
            marker.scale.y = axis_length_2;
        } else {
            marker.scale.x = axis_length_2;
            marker.scale.y = axis_length_1;
        }
        marker.scale.z = 0.1;

        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker_array.markers.push_back(marker);
        id++;
    }
    covariance_pub_.publish(marker_array);
    #endif
    
    #ifdef DetectLoop
    for(int i = 0; i < num_nodes - 1; i++) {
        double d = mahalanobisDistance(graph_->nodes[i], graph_->nodes.back());
        if(d <= MAX_MALAHANOBIS_DISTANCE) {
            marker_edge_.id = id;
            marker_edge_.points.clear();
            geometry_msgs::Point p;

            int k = graph_->nodes.back().id;
            p.x = graph_->nodes[i].pose[0];
            p.y = graph_->nodes[i].pose[1];
            marker_edge_.points.push_back(p);

            p.x = graph_->nodes[k].pose[0];
            p.y = graph_->nodes[k].pose[1];
            marker_edge_.points.push_back(p);

            marker_graph_.markers.push_back(marker_edge_);
            id++;
        }
    }
    #endif
    graph_pub_.publish(marker_graph_);
}

double normalize(double z) {
    return atan2(sin(z), cos(z));
}

double angle_diff(double a, double b) {
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a - b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0) d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
        return d1;
    else
        return d2;
}

bool GraphBasedSlam::updateMotion(Eigen::Vector3d& u_t_1,
                                  Eigen::Vector3d& u_t,
                                  Eigen::Vector3d& x) {
    if(model_type_ == "omni") {
        double delta_trans, delta_rot, delta_bearing;
        delta_trans = (u_t.head(2) - u_t_1.head(2)).norm();
        delta_rot = angle_diff(u_t[2], u_t_1[2]);
        delta_bearing = angle_diff(atan2(u_t[1] - u_t_1[1], u_t[0] - u_t_1[0]), u_t[2]) + x[2];

        Eigen::Vector3d delta_x = {delta_trans*cos(delta_bearing),
                                   delta_trans*sin(delta_bearing),
                                   delta_rot};
        x += delta_x;
        x[2] = normalize(x[2]);
        return true;
    } else if (model_type_ == "diff") {
        double delta_trans, delta_rot1, delta_rot2;
        delta_trans = (u_t.head(2) - u_t_1.head(2)).norm();

        if(delta_trans < 0.01)
            delta_rot1 = 0.0;
        else
            delta_rot1 = angle_diff(atan2(u_t[1] - u_t_1[1], u_t[0] - u_t_1[0]), u_t[2]);
        
        delta_rot2 = angle_diff(angle_diff(u_t[2], u_t_1[2]), delta_rot1);
        
        Eigen::Vector3d delta_x = {delta_trans*cos(x[2] + delta_rot1),
                                   delta_trans*sin(x[2] + delta_rot1),
                                   delta_rot1 + delta_rot2};
        x += delta_x;
        x[2] = normalize(x[2]);
        return true;
    } else {
        ROS_ERROR("\"model_type\" parameter is unvalid, \"omni\" or \"diff\" !");
        return false;
    }               
}

/** Occupancy grid map - using Bresenham's Line Drawing Algorithm
    -1: un-known    0: free     100: occupied */
void GraphBasedSlam::rayCasting(Node *node, Map2D *map) {
    /** Lidar pose in map frame */
    double x, y, theta;
    x = node->pose[0] + pose_lidar[0] * cos(node->pose[2]) - pose_lidar[1] * sin(node->pose[2]);
    y = node->pose[1] + pose_lidar[0] * sin(node->pose[2]) + pose_lidar[1] * cos(node->pose[2]);
    theta = normalize(node->pose[2] + pose_lidar[2]);

    int num_beam = node->scan.size();
    const int kernel = std::min(40, num_beam / 2);
    
    int del_x, del_y, e;
    unsigned int id_x1, id_x2, id_y1, id_y2, id_x, id_y;
    int x_step, y_step, p1, p2;

    if(!map->worldToMap(x, y, id_x1, id_y1))
        return;
    
    double anpha, d, phi;
    int id_cell, log_inv;
    double x_endpoint, y_endpoint, x_cell, y_cell;
    double half_angle_increment = 0.5 * fabs(node->scan[0][1] - node->scan[1][1]);
    double half_map_resolution = 0.5 * map->getResolution();
    bool outlier;
    for(int k = 0; k < num_beam; k++) {
        anpha = node->scan[k][1] + pose_lidar[2] + node->pose[2];
        x_endpoint = x + node->scan[k][0] * cos(anpha);
        y_endpoint = y + node->scan[k][0] * sin(anpha);
        outlier = false;
        if (!map->worldToMap(x_endpoint, y_endpoint, id_x2, id_y2)) {
            id_x2 = std::max(0u, std::min(id_x2, map->getSizeInCellsX() - 1));
            id_y2 = std::max(0u, std::min(id_y2, map->getSizeInCellsY() - 1));
            outlier = true;
        }
        
        del_x = id_x2 - id_x1;
        del_y = id_y2 - id_y1;
        if (del_x < 0) del_x = -del_x;
        if (del_y < 0) del_y = -del_y;
        x_step = 1;
        if (id_x2 < id_x1) x_step = -1;
        y_step = 1;
        if (id_y2 < id_y1) y_step = -1;
        id_x = id_x1; id_y = id_y1;

        if(del_x > del_y) {
            e = 2*del_y - del_x;
            p1 = 2*(del_y - del_x);
            p2 = 2*del_y;
            for(int i = 0; i < del_x; i++) {
                if(e >= 0) {
                    id_y += y_step;
                    e += p1;
                } else {
                    e += p2;
                }
                id_x += x_step;
                map->mapToWorld(id_x, id_y, x_cell, y_cell);
                d = sqrt((x_cell - x) * (x_cell - x) + (y_cell - y) * (y_cell - y));
                phi = atan2(y_cell - y, x_cell - x) - theta;

                int k_min = k;
                int k_start = std::max(0, k - kernel);
                int k_end = std::min(num_beam - 1, k + kernel);
                for (int j = k_start; j <= k_end; j++) {
                    if (fabs(phi - node->scan[j][1]) < fabs(phi - node->scan[k_min][1])) {
                        k_min = j;
                    }
                }

                if(d > std::min(max_range_, node->scan[k_min][0] + half_map_resolution) ||
                   fabs(phi - node->scan[k_min][1]) > half_angle_increment)
                {
                    log_inv = NO_INFORMATION;
                }
                else if (node->scan[k_min][0] < max_range_ && 
                          fabs(d - node->scan[k_min][0]) < half_map_resolution)
                {
                    log_inv = OCCUPIED;
                }
                else if (d <= node->scan[k_min][0])
                {
                    log_inv = FREE_SPACE;
                }
                map->updateCost(id_x, id_y, log_inv - NO_INFORMATION);
            }
        }else {
            e  = 2 * del_x - del_y;
            p1 = 2 * (del_x - del_y);
            p2 = 2 * del_x;
            for(int i = 0; i < del_y; i++) {
                if (e >= 0) {
                    id_x += x_step;
                    e += p1;
                } else {
                    e += p2;
                }
                id_y += y_step;
                map->mapToWorld(id_x, id_y, x_cell, y_cell);
                d = sqrt((x_cell - x) * (x_cell - x) + (y_cell - y) * (y_cell - y));
                phi = atan2(y_cell - y, x_cell - x) - theta;

                int k_min = k;
                int k_start = std::max(0, k - kernel);
                int k_end = std::min(num_beam - 1, k + kernel);
                for (int j = k_start; j <= k_end; j++) {
                    if (fabs(phi - node->scan[j][1]) < fabs(phi - node->scan[k_min][1])) {
                        k_min = j;
                    }
                }

                if(d > std::min(max_range_, node->scan[k_min][0] + half_map_resolution) ||
                   fabs(phi - node->scan[k_min][1]) > half_angle_increment)
                {
                    log_inv = NO_INFORMATION;
                }
                else if (node->scan[k_min][0] < max_range_ && 
                          fabs(d - node->scan[k_min][0]) < half_map_resolution)
                {
                    log_inv = OCCUPIED;
                }
                else if (d <= node->scan[k_min][0])
                {
                    log_inv = FREE_SPACE;
                }
                map->updateCost(id_x, id_y, log_inv - NO_INFORMATION);
            }
        }
        if(outlier)
            map->updateCost(id_x2, id_y2, FREE_SPACE - OCCUPIED);
    }
}

};  // slam2d namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "graph_based_slam");
    ros::NodeHandle nh_;
    slam2d::GraphBasedSlam mapping;

    return 0;
}