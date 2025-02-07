#pragma once
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <vk_costmap_2d/layer.hpp>
#include <vk_costmap_2d/costmap_2d.hpp>
#include <vk_costmap_2d/costvalue.hpp>
#include <vk_costmap_2d/footprint.hpp>

namespace vk_costmap_2d {

class Layer;

class LayeredCostmap {
    private:
        Costmap2D master_costmap_;
        std::string global_frame_;
        bool rolling_window_;
        bool updated_;
        double min_x_, min_y_, max_x_, max_y_;
        unsigned int bx0_, bxn_, by0_, byn_;

        double circumscribed_radius_;
        double inscribed_radius_;
        std::vector<geometry_msgs::Point> footprint_;

        std::vector<boost::shared_ptr<Layer>> layers_;
        bool initialized_;
        bool size_locked_;

    public:
        LayeredCostmap(std::string global_frame, bool rolling_window);
        ~LayeredCostmap();

        void updateMap(double robot_x, double robot_y, double robot_yaw);

        void resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                       double origin_x, double origin_y, bool size_locked = false);

        Costmap2D* getCostmap() { return &master_costmap_; }

        std::string getGlobalFrameID() const { return global_frame_; }

        bool isUpdated();

         bool isSizeLocked() { return size_locked_; }

        bool isRolling() { return rolling_window_; }

        std::vector<boost::shared_ptr<Layer>>* getLayers() { return &layers_; }
        void addLayer(boost::shared_ptr<Layer> layer) { layers_.push_back(layer); }

        /* Updates the stored footprint, updates the circumscribed
        * and inscribed radii, and calls onFootprintChanged() in all
        * layers. */
        void setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec);

        /* Returns the latest footprint stored with setFootprint(). */
        const std::vector<geometry_msgs::Point>& getFootprint() { return footprint_; }

        void getBounds(unsigned int* x0, unsigned int* xn,
                       unsigned int* y0, unsigned int* yn) {
            *x0 = bx0_;
            *y0 = by0_;
            *xn = bxn_;
            *yn = byn_;
        }

        /* This is updated by setFootprint(). */
        double getCircumscribedRadius() { return circumscribed_radius_; }
        double getInscribedRadius() { return inscribed_radius_; }

        bool isInitialized() { return initialized_; }

};  // LayeredCostmap class


}   // vk_costmap_2d namespace