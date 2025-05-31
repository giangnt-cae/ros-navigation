#pragma once

#include "header.hpp"
#include "map.hpp"
#include "scan_matcher.hpp"
#include "motion_model.hpp"
#include "adaptivethreshold.hpp"

static const std::string scan_topic = "scan_multi";

class LocalizationNode {
    public:
        LocalizationNode();
        ~LocalizationNode();
        void savePoseToServer();
    private:
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
        std::shared_ptr<tf2_ros::TransformListener> tfl_;
        std::shared_ptr<tf2_ros::Buffer> tf_;

        bool sent_first_transform_;

        tf2::Transform latest_tf_;
        bool latest_tf_valid_;

        void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
        void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
        void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);

        void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
        void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
        map_t* convertMap(const nav_msgs::OccupancyGrid& msg);

        void updatePoseFromServer();

        std::string odom_frame_id_;
        geometry_msgs::PoseStamped latest_odom_pose_;

        std::string base_frame_id_;
        std::string map_frame_id_;

        bool use_map_topic_;
        bool first_map_only_;

        ros::Duration gui_publish_period;
        ros::Time save_pose_last_time;
        ros::Duration save_pose_period;

        geometry_msgs::PoseWithCovarianceStamped last_published_pose;

        map_t* map_;
        char* mapdata;
        int sx, sy;
        double resolution;

        message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
        tf2_ros::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;

        ros::Subscriber initial_pose_sub_;
        std::vector<Laser*> lasers_;
        std::vector<bool> lasers_update_;
        std::map<std::string, int> frame_to_laser_;

        Eigen::Vector3d robot_pose;
        AdaptiveThreshold* adp_thresh_;
        ScanMatcher* sm_;
        Eigen::Vector3d f_odom_pose_;
        bool f_init_;

        // Laser range data (range, bearing tuples)
        double laser_min_range_;
        double laser_max_range_;

        double initial_threshold_;
        double min_motion_;
        double min_trans_;
        double min_rot_;
        double sigma;

        // Nomotion update control
        bool m_force_update;    // used to temporarily let localization update robot pose even when no motion occurs ...

        Odom* odom_;
        Laser* laser_;

        void requestMap();

        // Get odometric pose from transform system
        bool getOdomPose(geometry_msgs::PoseStamped& pose,
                         double& x, double& y, double& yaw,
                         const ros::Time& t, const std::string& f);
        
        // Time for tolerance on the published transform,
        // basically defines how long a map->odom transform is good for
        ros::Duration transform_tolerance_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher pose_pub_;
        ros::Subscriber initial_pose_sub_old_;
        ros::Subscriber map_sub_;

        bool first_map_received_;
        bool first_reconfigure_call_;

        boost::recursive_mutex configuration_mutex_;
        ros::Timer check_laser_timer_;

        std::string odom_model_type_;
        int max_beams_;

        double init_pose_[3];
        bool tf_broadcast_;
        bool force_update_after_initialpose_;

        ros::Time last_laser_received_ts_;
        ros::Duration laser_check_interval_;
        void checkLaserReceived(const ros::TimerEvent& event);
};

boost::shared_ptr<LocalizationNode> localization_node_ptr;
void sigintHandler(int sig);