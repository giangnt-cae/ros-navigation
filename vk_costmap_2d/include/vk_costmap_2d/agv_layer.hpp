#pragma once
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "costvalue.hpp"
#include "costmap_layer.hpp"
#include "footprint.hpp"

#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <vk_costmap_2d/AgvInfo.h>
#include <vk_costmap_2d/AgvInfoArray.h>

using vk_costmap_2d::NO_INFORMATION;
using vk_costmap_2d::LETHAL_OBSTACLE;
using vk_costmap_2d::FREE_SPACE;

namespace vk_costmap_2d {

class AgvLayer : public CostmapLayer {
    public:
        AgvLayer();
        ~AgvLayer() {}

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double* min_x, double* min_y,
                                  double* max_x, double* max_y);
        
        virtual void updateCosts(Costmap2D& master_grid,
                                 int min_i, int min_j,
                                 int max_i, int max_j);
        
        boost::recursive_mutex agv_access_;

    private:
        void errorAgvInfoCallback(const AgvInfoArrayConstPtr& msg);
        ros::Subscriber agv_sub_;
        double min_x_, min_y_, max_x_, max_y_;
        bool use_maximum_;
        bool agv_received_;
        bool rolling_window_;

};  // AgvLayer class

}   // vk_costmap_2d namespace