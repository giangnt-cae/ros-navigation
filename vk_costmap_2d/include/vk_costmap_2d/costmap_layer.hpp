#pragma once
#include <ros/ros.h>
#include "costvalue.hpp"
#include "layer.hpp"
#include "costmap_2d.hpp"
#include "layered_costmap.hpp"

namespace vk_costmap_2d {

class CostmapLayer : public Layer, public Costmap2D {
    public:
        CostmapLayer() {}
        virtual void matchSize();
        virtual void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert_area=false);
    protected:
        /* Updates the master_grid within the specified bounding box using this layer's values.
        TrueOverwrite means every value from this layer is written into the master grid. */
        void updateWithTrueOverwrite(vk_costmap_2d::Costmap2D& master_grid,
                                     int min_i, int min_j, 
                                     int max_i, int max_j);

        /* Updates the master_grid within the specified bounding box using this layer's values.
        TrueOverwrite means every value from this layer is written into the master grid (does not copy NO_INFORMATION) */
        void updateWithOverwrite(vk_costmap_2d::Costmap2D& master_grid,
                                 int min_i, int min_j, 
                                 int max_i, int max_j);

        /* Updates the master_grid within the specified bounding box using this layer's values.
        Sets the new value to the maximum of the master_grid's value and this layer's value. 
        If the master value is NO_INFORMATION, it is overwritten. 
        If the layer's value is NO_INFORMATION, the master value does not change. */
        void updateWithMax(vk_costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, 
                           int max_i, int max_j);

        /* Updates the master_grid within the specified bounding box using this layer's values.
        Sets the new value to the sum of the master grid's value and this layer's value.
        If the master value is NO_INFORMATION, it is overwritten with the layer's value.
        If the layer's value is NO_INFORMATION, then the master value does not change.

        If the sum value is larger than INSCRIBED_INFLATED_OBSTACLE,
        the master value is set to (INSCRIBED_INFLATED_OBSTACLE - 1) */
        void updateWithMaxupdateWithAddition(vk_costmap_2d::Costmap2D& master_grid,
                                             int min_i, int min_j, 
                                             int max_i, int max_j);
        
        void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

};



}   // namespace vk_costmap_2d