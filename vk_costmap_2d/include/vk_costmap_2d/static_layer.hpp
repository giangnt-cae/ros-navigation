#pragma once
#include <ros/ros.h>
#include "costmap_layer.hpp"
#include <nav_msgs/OccupancyGrid.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


namespace vk_costmap_2d {

class StaticLayer : public CostmapLayer {
    private:
        ros::Subscriber map_sub_;

        std::string global_frame_;
        std::string map_frame_;
        unsigned int x0_, y0_;
        unsigned int width_, height_;
        bool map_received_;
        bool first_map_only_;
        bool use_maximum_;
        bool has_updated_data_;
        unsigned char lethal_threshold_, unknown_cost_value_;

        unsigned char interpretValue(unsigned char value);
        void incomingMap(const nav_msgs::OccupancyGridConstPtr& msg);

    public:
        StaticLayer();
        virtual ~StaticLayer();

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double* min_x, double* min_y,
                                  double* max_x, double* max_y);
        virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j,
                                 int max_i, int max_j);
        virtual void matchSize();

};  // StaticLayer class

}   // namespace vk_costmap_2d 

