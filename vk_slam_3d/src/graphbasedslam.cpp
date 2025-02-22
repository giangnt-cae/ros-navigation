#include <vk_slam_3d/graphbasedslam.hpp>

namespace vk_slam_3d {

GraphBasedSlam::GraphBasedSlam() : odom_(NULL),
                                 sm_(NULL),
                                 adp_thresh_(NULL)
{
    ros::NodeHandle nh_;
    init();
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud>("map", 1);
    pose_graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("graph", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_(nh_, "odom", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> laser_scan_sub_(nh_, "velodyne_points", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub_, laser_scan_sub_);
    sync.registerCallback(&GraphBasedSlam::dataCallback, this);
    
    VoxelHashMap::Ptr voxel_map (new VoxelHashMap(voxel_size_, max_points_per_voxel_));
    
    odom_ = new Odom();
    odom_->model_type = model_type_;
    
    sm_ = new ScanMatcher();
    
    adp_thresh_ = new AdaptiveThreshold(initial_threshold_, min_motion_, max_range_);
    Graph graph;
    
    Eigen::Vector6d odom_t_1;
    Eigen::Vector6d odom_t;
    Eigen::Vector6d f_odom_pose_;
    Eigen::Vector6d robot_pose;
    Node node_t;
    Edge edge_t;
    bool first_time = true;
    double sigma;
    
    Eigen::Vector6d tf_base_frame_to_lidar_frame = {-0.058, 0.0, 0.394, 0.0, 0.0, 0.0};
    Eigen::Affine3d tf_base_frame_to_lidar_frame_matrix = XYZEulertoAffineMatrix({tf_base_frame_to_lidar_frame});

    ros::Rate rate(frequency_);
    while(ros::ok()) {
        ros::spinOnce();
        if(received_data_) {
            odom_t = odom_->pose;
            if(first_time) {
                robot_pose = {20.0, -20.0, 0.0, 0.0, 0.0, odom_t[5]};
                node_t.pose = robot_pose;
                node_t.scan = scan_;
                node_t.idx = 0;
                graph.nodes.push_back(node_t);
                Eigen::Affine3d T_t = XYZEulertoAffineMatrix(node_t.pose);
                voxel_map->update(scan_, T_t);
                visualization(graph, voxel_map);

                odom_t_1 = odom_t;
                f_odom_pose_ = odom_t;
                sigma = initial_threshold_;
                first_time = false;
            }
            odom_->UpdateMotion(odom_t_1, odom_t, robot_pose);
            Eigen::Vector3d delta = Eigen::Vector3d::Zero();
            // Compute change in pose
            delta.head(2) = odom_t.head(2) - f_odom_pose_.head(2);
            delta[2] = angle_diff(odom_t[5], f_odom_pose_[5]);

            // See if we should update the filter
            bool update = fabs(delta[0]) > min_trans_ ||
                          fabs(delta[1]) > min_trans_ ||
                          fabs(delta[2]) > min_rot_;
            if(update) {
                Eigen::Affine3d initial_guess = XYZEulertoAffineMatrix(robot_pose);

                Eigen::Affine3d T_t = sm_->registration(icpscan_, voxel_map, initial_guess, 3.0 * sigma, sigma / 3.0);

                const Eigen::Affine3d current_deviation = initial_guess.inverse() * T_t;     

                adp_thresh_->UpdateModelDeviation(current_deviation);

                sigma = adp_thresh_->ComputeThreshold();
                
                robot_pose.head(3) = T_t.translation();
                robot_pose[3] = atan2(T_t(2 , 1), T_t(2, 2));
                robot_pose[5] = atan2(T_t(1 , 0), T_t(0, 0));
                robot_pose[4] = atan2(-T_t(2, 0), T_t(0, 0) / cos(robot_pose[5]));

                node_t.pose = robot_pose;
                node_t.scan = scan_;
                node_t.idx ++;
                graph.nodes.push_back(node_t);

                edge_t.i = node_t.idx - 1;
                edge_t.j = node_t.idx;
                graph.edges.push_back(edge_t);

                voxel_map->update(scan_, T_t);
                visualization(graph, voxel_map);
                f_odom_pose_ = odom_t;
            }
            odom_t_1 = odom_t;
        }
        rate.sleep();
    }
}

void GraphBasedSlam::init() {
    if(!ros::param::get("~map_frame", map_frame_))
        map_frame_ = "map";
    if(!ros::param::get("~base_frame", base_frame_))
        base_frame_ = "vkbot";
    if(!ros::param::get("~odom_frame", odom_frame_))
        odom_frame_ = "odom";
    if(!ros::param::get("~model_type", model_type_))
        model_type_ = "omni";

    if(!ros::param::get("~frequency", frequency_))
        frequency_ = 10;
    if(!ros::param::get("~anpha", anpha_))
        anpha_ = 0.5;
    if(!ros::param::get("~beta", beta_))
        beta_ = 1.5;
    if(!ros::param::get("~voxel_size", voxel_size_))
        voxel_size_ = 0.3;
    if(!ros::param::get("~max_points_per_voxel", max_points_per_voxel_))
        max_points_per_voxel_ = 20;
    if(!ros::param::get("~min_trans", min_trans_))
        min_trans_ = 1.0;
    if(!ros::param::get("~min_rot", min_rot_))
        min_rot_ = 0.5;
    if(!ros::param::get("~min_range", min_range_))
        min_range_ = 2.0;
    if(!ros::param::get("~max_range", max_range_))
        max_range_ = 30.0;
    if(!ros::param::get("~min_z", min_z_))
        min_z_ = -0.4;
    if(!ros::param::get("~max_z", max_z_))
        max_z_ = 2.0;
    if(!ros::param::get("~initial_threshold", initial_threshold_))
        initial_threshold_ = 1.0;
    if(!ros::param::get("~min_motion", min_motion_))
        min_motion_ = 0.1;
    
    received_data_ = false;

    marker_node.header.frame_id = map_frame_;
    marker_node.header.stamp = ros::Time::now();
    marker_node.action = visualization_msgs::Marker::ADD;
    marker_node.type = visualization_msgs::Marker::SPHERE;
    marker_node.id = 0;
    marker_node.ns = "vk_slam_3d_node";
    marker_node.pose.orientation.x = 0;
    marker_node.pose.orientation.y = 0;
    marker_node.pose.orientation.z = 0.0;
    marker_node.pose.orientation.w = 1.0;
    marker_node.scale.x = 0.1;
    marker_node.scale.y = 0.1;
    marker_node.scale.z = 0.1;
    marker_node.color.r = 1.0;
    marker_node.color.g = 0.0;
    marker_node.color.b = 0.0;
    marker_node.color.a = 2.0;
    marker_node.lifetime = ros::Duration(0);

    marker_edge.header.frame_id = map_frame_;
    marker_edge.header.stamp = ros::Time::now();
    marker_edge.action = visualization_msgs::Marker::ADD;
    marker_edge.type = visualization_msgs::Marker::LINE_STRIP;
    marker_edge.id = 0;
    marker_edge.scale.x = 0.02;
    marker_edge.scale.y = 0.02;
    marker_edge.scale.z = 0.02;
    marker_edge.ns = "vk_slam_3d_edge";
    marker_edge.pose.orientation.x = 0;
    marker_edge.pose.orientation.y = 0;
    marker_edge.pose.orientation.z = 0.0;
    marker_edge.pose.orientation.w = 1.0;
    marker_edge.color.r = 0.0;
    marker_edge.color.g = 1.0;
    marker_edge.color.b = 0.0;
    marker_edge.color.a = 2.0;
    marker_edge.lifetime = ros::Duration(0);
}

void GraphBasedSlam::dataCallback(const nav_msgs::Odometry& msg1, const sensor_msgs::PointCloud2& msg2) {
    odom_->pose = {msg1.pose.pose.position.x, 
                   msg1.pose.pose.position.y,
                   msg1.pose.pose.position.z,
                   0.0,
                   0.0,
                   tf::getYaw(msg1.pose.pose.orientation)};
    
    Eigen::Vector6d pose_lidar = {-0.058, 0.0, 0.394, 0.0, 0.0, 0.0};         // transform base_frame to lidar_frame
    Eigen::Affine3d T = XYZEulertoAffineMatrix({pose_lidar});
    sensor_msgs::PointCloud pcl;
    sensor_msgs::convertPointCloud2ToPointCloud(msg2, pcl);
    
    std::vector<Eigen::Vector3d> points;
    Eigen::Vector3d point;
    for(int i = 0; i < pcl.points.size(); i++) {
        if(!pcl.channels[0].values[i]) continue;
        if(pcl.points[i].z < min_z_ || pcl.points[i].z > max_z_) continue;
        point = {pcl.points[i].x, pcl.points[i].y, pcl.points[i].z};
        auto range = point.norm();
        if(range > min_range_ && range < max_range_) {
            point = T.rotation() * point + T.translation();
            points.push_back(point);
        }
    }

    scan_ = downSamplingFilter(points, anpha_ * voxel_size_);
    icpscan_ = downSamplingFilter(scan_, beta_ * voxel_size_);
    received_data_ = true;
}


std::vector<Eigen::Vector3d> GraphBasedSlam::downSamplingFilter(const std::vector<Eigen::Vector3d>& points, double voxel_size) {
    VoxelGrid* voxelgrid = new VoxelGrid();
    for(auto& point : points) {
        Eigen::Vector3i voxel = (point / voxel_size).cast<int>();
        voxelgrid->insert(point, voxel);
    }
    std::vector<Eigen::Vector3d> points_downsampled;
    Voxel* temp;
    for(int i = 0; i < voxelgrid->gethashsize(); i++) {
        temp = voxelgrid->hash_table[i];
        while(temp != NULL) {
            points_downsampled.push_back(temp->points[0]);
            temp = temp->next;
        }
    }
    delete [] voxelgrid;
    return points_downsampled;
}

std::vector<Eigen::Vector3d> GraphBasedSlam::preProcessing(const std::vector<Eigen::Vector3d>& points) {
    std::vector<Eigen::Vector3d> inliers;
    for(auto& pt : points) {
        double range = pt.norm();
        if(range > min_range_ && range < max_range_) inliers.push_back(pt);
    }
    return inliers;
}

void GraphBasedSlam::visualization(Graph& graph, VoxelHashMap::Ptr voxel_map) {
    std::vector<Eigen::Vector3d> eigen_points = voxel_map->extractPoints();
    sensor_msgs::PointCloud ros_map = voxel_map->eigenToPointCloudMsg(eigen_points, map_frame_);
    map_pub_.publish(ros_map);

    int num_nodes = graph.nodes.size();
    int num_edges = graph.edges.size();
    int id = 0;
    marker_graph.markers.clear();
    for(int i = 0; i < num_nodes; i++) {
        marker_node.id = id;
        marker_node.pose.position.x = graph.nodes[i].pose(0);
        marker_node.pose.position.y = graph.nodes[i].pose(1);
        marker_node.pose.position.z = graph.nodes[i].pose(2);
        marker_graph.markers.push_back(marker_node);
        id++;
    }
    
    for(int j = 0; j < num_edges; j++) {
        marker_edge.id = id;
        marker_edge.points.clear();
        geometry_msgs::Point p;

        int id_i = graph.edges[j].i;
        int id_j = graph.edges[j].j;

        p.x = graph.nodes[id_i].pose(0);
        p.y = graph.nodes[id_i].pose(1);
        p.z = graph.nodes[id_i].pose(2);
        marker_edge.points.push_back(p);

        p.x = graph.nodes[id_j].pose(0);
        p.y = graph.nodes[id_j].pose(1);
        p.z = graph.nodes[id_j].pose(2);
        marker_edge.points.push_back(p);

        marker_graph.markers.push_back(marker_edge);
        id++;
    }
    pose_graph_pub_.publish(marker_graph);
}

}   // namespace vk_slam_3d