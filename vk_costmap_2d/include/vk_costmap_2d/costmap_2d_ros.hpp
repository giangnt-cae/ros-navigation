#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <vk_costmap_2d/layered_costmap.hpp>
#include <vk_costmap_2d/costmap_layer.hpp>
#include <vk_costmap_2d/static_layer.hpp>
#include <vk_costmap_2d/inflation_layer.hpp>
#include <vk_costmap_2d/footprint.hpp>
#include <vk_costmap_2d/agv_layer.hpp>
#include <vk_costmap_2d/obstacle_layer.hpp>

namespace vk_costmap_2d {

class Costmap2DROS {
    private:
        ros::Publisher costmap_pub_;
        nav_msgs::OccupancyGrid grid_;
        static char* cost_translation_table_;
        unsigned int x0_, xn_, y0_, yn_;

        double width_, height_, resolution_, origin_x_,  origin_y_;

        LayeredCostmap* layered_costmap_;
        std::string name_;
        tf2_ros::Buffer& tf_;               // Used for transforming point clouds
        std::string global_frame_;          // The global frame for the costmap
        std::string base_frame_;            // The frame_id of the robot base
        double transform_tolerance_;        // Timeout before transform errors
        double publish_rate;

        float footprint_padding_X_;
        float footprint_padding_Y_;
        float footprint_padding_;
        
        ros::Publisher footprint_pub_;
        std::vector<geometry_msgs::Point> unpadded_footprint_;
        std::vector<geometry_msgs::Point> padded_footprint_;
        double robot_radius_;
        
        ros::Time last_publish_;
        ros::Duration publish_cycle;
        bool map_update_thread_shutdown_;
        boost::thread* map_update_thread_;
        ros::NodeHandle private_nh;
        ros::NodeHandle nh;

        boost::recursive_mutex configuration_mutex_;
    public:
        Costmap2DROS(const std::string &name, tf2_ros::Buffer& tf);
        ~Costmap2DROS();
        void mapUpdateLoop(double frequency);
        void publishCostmap();

        void updateMap();
        bool getRobotPose(geometry_msgs::PoseStamped& global_pose) const;
        double getTransformTolerance() const { return transform_tolerance_; }

        bool loadFootprintFromParam(ros::NodeHandle& nh, const std::string& param_name, std::vector<geometry_msgs::Point>& footprint);

};  // Costmap2DROS class


}   // namespace vk_costmap_2d