#pragma once
#include <ros/ros.h>
#include "costmap_layer.hpp"
#include "costmap_2d.hpp"

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

class CellData {
    public:
        CellData(double i, unsigned int x, unsigned int y,
                 unsigned int sx, unsigned int sy) :
            index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
        {}
        unsigned int index_;
        unsigned int x_, y_;
        unsigned int src_x_, src_y_;
};

class InflationLayer : public Layer {
    public:
        InflationLayer();
        virtual ~InflationLayer() { deleteKernels(); }

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double* min_x, double* min_y,
                                  double* max_x, double* max_y);
        virtual void updateCosts(Costmap2D& master_grid,
                                 int min_i, int min_j,
                                 int max_i, int max_j);

        virtual void matchSize();

        virtual inline unsigned char computeCost(double distance) const {
            unsigned char cost = 0;
            if (distance == 0)
                cost = LETHAL_OBSTACLE;
            else if (distance * resolution_ <= inscribed_radius_)
                cost = INSCRIBED_INFLATED_OBSTACLE;
            else {
                double euclidean_distance = distance * resolution_;
                double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
                cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
            }
            return cost;
        }
    
    private:
        virtual void onFootprintChanged();
        boost::recursive_mutex inflation_access_;

        double resolution_;
        double inflation_radius_;
        double inscribed_radius_;
        double weight_;
        bool inflate_unknown_;

        inline double distanceLookup(int mx, int my, int src_x, int src_y) {
            unsigned int dx = abs(mx - src_x);
            unsigned int dy = abs(my - src_y);
            return cached_distances_[dx][dy];
        }

        inline unsigned char costLookup(int mx, int my, int src_x, int src_y) {
            unsigned int dx = abs(mx - src_x);
            unsigned int dy = abs(my - src_y);
            return cached_costs_[dx][dy];
        }

        void deleteKernels();
        void computeCaches();

        unsigned int cellDistance(double world_dist) {
            return layered_costmap_->getCostmap()->cellDistance(world_dist);
        }

        inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y);

        unsigned int cell_inflation_radius_;
        unsigned int cached_cell_inflation_radius_;
        std::map<double, std::vector<CellData> > inflation_cells_;

        bool* seen_;
        int seen_size_;

        unsigned char** cached_costs_;
        double** cached_distances_;
        double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

};  // InflationLayer class

}   // vk_costmap_2d namespace