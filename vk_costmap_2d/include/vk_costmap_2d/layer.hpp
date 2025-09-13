#pragma once
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Point.h>
#include <string.h>

#include <vk_costmap_2d/costmap_2d.hpp>
#include <vk_costmap_2d/layered_costmap.hpp>

namespace vk_costmap_2d {
    
class LayeredCostmap;

class Layer {
    private:
        std::vector<geometry_msgs::Point> footprint_spec_;

    protected:
        virtual void onInitialize() {}

        LayeredCostmap* layered_costmap_;
        bool updated_;
        bool enabled_;
        std::string name_;
        tf2_ros::Buffer *tf_;
    public:
        Layer() : layered_costmap_(NULL), updated_(false), enabled_(false), name_(), tf_(NULL) {}

        void initialize(LayeredCostmap* layered_costmap, std::string name, tf2_ros::Buffer *tf) {
            layered_costmap_ = layered_costmap;
            name_ = name;
            tf_ = tf;
            enabled_ = true;
            onInitialize();
        }

        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double* min_x, double* min_y,
                                  double* max_x, double* max_y) {};
        virtual void updateCosts(Costmap2D& master_grid,
                                 int min_i, int min_j,
                                 int max_i, int max_j) {}
        virtual void matchSize() {}

        bool isUpdated() const { return updated_; }
        bool isEnabled() const noexcept { return enabled_; }

        std::string getName() const { return name_; }

        const std::vector<geometry_msgs::Point>& getFootprint() const;
        virtual void onFootprintChanged() {};
        virtual std::string getID() const { return std::string(""); }

        virtual ~Layer() {}

};  // class Layer

}   // namespace vk_cost_map_2d