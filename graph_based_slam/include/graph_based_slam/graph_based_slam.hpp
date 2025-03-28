#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <graph_based_slam/map.hpp>
#include <graph_based_slam/scan_matcher.hpp>

namespace slam2d {

struct Node {
    Eigen::Vector3d pose;
    std::vector<Eigen::Vector2d> scan;
    std::vector<Eigen::Vector2d> points;
    Eigen::Matrix3d omega;
    unsigned int id;
};

struct Edge {
    Eigen::Affine2d z;
    Eigen::Matrix3d omega;
    unsigned int ni;
    unsigned int nj;
};

struct Graph {
    std::vector<Node> nodes;
    std::vector<Edge> edges;
};

double normalize(double z);

double angle_diff(double a, double b);

inline double sign(bool x) { return (x == true ? -1.0 : 1.0); }

static const Eigen::Vector3d pose_lidar = {0.289, 0.0, 0.0};     // transform base_frame to lidar_frame
static const Eigen::Vector3d initial_pose = {-20.0, 0.0, -9.6 * M_PI / 180};
// static const Eigen::Vector3d initial_pose = {0.0, 0.0, 0.0};

class GraphBasedSlam {
    private:
        ros::NodeHandle nh_;
        ros::Publisher map_pub_, graph_pub_, covariance_pub_;
        ros::Subscriber laser_scan_sub_, odom_sub_;

        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
        void dataCallback(const nav_msgs::Odometry& msg1, const sensor_msgs::LaserScan& msg2);

        std::string odom_frame_, map_frame_, base_frame_, model_type_;
        double min_trans_, min_rot_;
        double min_range_, max_range_;
        double angle_min_, angle_max_;
        int throttle_scan_;
        int max_beams_;
        bool inverted_laser_;

        bool f_init_;
        bool received_data_;
        double max_distance_threshold_;

        unsigned int map_size_x_, map_size_y_, delta_;
        double resolution_;
        double origin_x_, origin_y_;
        unsigned char default_value_;

        ScanMatcher* sm_;
        Map2D* map_;

        std::vector<Eigen::Vector2d> scan_;

        visualization_msgs::Marker marker_node_, marker_edge_;
        visualization_msgs::MarkerArray marker_graph_;

        void init();
        void visualization();
        bool updateMotion(Eigen::Vector3d& u_t_1, Eigen::Vector3d& u_t, Eigen::Vector3d& x);
        void rayCasting(Node *node, Map2D *map);

        Eigen::Vector3d odom_t_, odom_t_1_, f_odom_pose_;
        Eigen::Vector3d robot_pose_;

        Node* node_;
        Edge* edge_;
        Graph* graph_;

    public:
        GraphBasedSlam();

        ~GraphBasedSlam() {
            delete sm_;
            delete map_;
            delete node_;
            delete edge_;
            delete graph_;
        }

};  // class GraphBasedSlam

}   // slam2d namespace


