#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <vk_slam_3d/adaptive_threshold.hpp>
#include <vk_slam_3d/scan_matcher.hpp>
#include <vk_slam_3d/odometry_model.hpp>
#include <vk_slam_3d/voxel_map.hpp>

namespace vk_slam_3d {

struct Node {
    Eigen::Vector6d pose;
    std::vector<Eigen::Vector3d> scan;
    Eigen::Matrix6d inverse_covariance;
    int idx;
};

struct Edge {
    Eigen::Vector6d z;
    Eigen::Matrix6d inverse_covariance;
    int i;
    int j;
};

struct Graph {
    std::vector<Edge> edges;
    std::vector<Node> nodes;
};

class GraphBasedSlam {
    private:
        ros::Publisher map_pub_, pose_graph_pub_;
        ros::Subscriber laser_scan_sub_, odom_sub_;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
        void dataCallback(const nav_msgs::Odometry& msg1, const sensor_msgs::PointCloud2& msg2);

        std::vector<Eigen::Vector3d> downSamplingFilter(const std::vector<Eigen::Vector3d>& points,
                                                        double voxel_size);

        std::vector<Eigen::Vector3d> preProcessing(const std::vector<Eigen::Vector3d>& points);

        std::string odom_frame_, map_frame_, base_frame_, model_type_;
        double frequency_;
        double anpha_, beta_;       // anpha = [0; 1.0], beta = [1.0; 2.0]
        double voxel_size_;
        int max_points_per_voxel_;
        double min_trans_, min_rot_;
        double min_range_, max_range_;
        double min_z_, max_z_;
        bool received_data_;

        double initial_threshold_;
        double min_motion_;

        Odom* odom_;
        ScanMatcher* sm_;
        AdaptiveThreshold* adp_thresh_;

        std::vector<Eigen::Vector3d> scan_;
        std::vector<Eigen::Vector3d> icpscan_;

        visualization_msgs::Marker marker_node;
        visualization_msgs::Marker marker_edge;
        visualization_msgs::MarkerArray marker_graph;

        void init();
        void visualization(Graph& graph, VoxelHashMap::Ptr voxel_map);

    public:
        GraphBasedSlam();

        ~GraphBasedSlam() {
            delete odom_;
            delete sm_;
            delete adp_thresh_;
        }
};


}   // namespace vk_slam_3d