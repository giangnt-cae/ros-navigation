#pragma once
#include <ros/ros.h>
#include <string.h>
#include <tf2_ros/message_filter.h>

namespace slam2d {

// Log-odds: log [px / (1 - px)]
static const int NO_INFORMATION = 0;
static const int OCCUPIED = 1;
static const int FREE_SPACE = -1;

static const int MAX_OCCUPIED = round(log(0.999 / (1.0 - 0.999)));
static const int MIN_FREE = round(log((1.0 - 0.999) / 0.999));

class Map2D {
    public:
        Map2D();

        Map2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
            double origin_x, double origin_y, int default_value);
        
        ~Map2D();

        void initMaps(unsigned int size_x, unsigned int size_y);
        void resetFullMap();
        void deleteMaps();

        void resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                       double origin_x, double origin_y);

        unsigned int getSizeInCellsX() const { return size_x_; }
        unsigned int getSizeInCellsY() const { return size_y_; }
        double getResolution() const { return resolution_; }
        double getOriginX() const { return origin_x_; }
        double getOriginY() const { return origin_y_; }

        double getSizeInMetersX() const { return (size_x_ - 1 + 0.5) * resolution_; }
        double getSizeInMetersY() const { return (size_y_ - 1 + 0.5) * resolution_; }

        int* getIntMap() const { return occmap_; }

        int getDefaultValue() { return default_value_; }
        void setDefaultValue(int c) { default_value_ = c; }

        int getCost(unsigned int mx, unsigned int my) const;
        void setCost(unsigned int mx, unsigned int my, int cost);
        void updateCost(unsigned int mx, unsigned int my, int value);

        inline unsigned int getIndex(unsigned int mx, unsigned int my) const { return my * size_x_ + mx; }

        inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const {
            my = index / size_x_;
            mx = index - (my * size_x_);
        }

        void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

        bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;

        bool mapValid(int mx, int my) const;

        // Copy a region of a source map into a destination map
    template<typename data_type>
        void copyMapRegion(data_type* source_map, unsigned int sm_x0, unsigned int sm_y0, unsigned int sm_size_x,
                           data_type* dest_map, unsigned int dm_x0, unsigned int dm_y0, unsigned int dm_size_x,
                           unsigned int region_size_x, unsigned int region_size_y) {
            // we'll first need to compute the starting points for each map
            data_type* sm_index = source_map + (sm_y0 * sm_size_x + sm_x0);
            data_type* dm_index = dest_map + (dm_y0 * dm_size_x + dm_x0);
              
            // now, we'll copy the source map into the destination map
            for (unsigned int i = 0; i < region_size_y; ++i) {
            memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
            sm_index += sm_size_x;
            dm_index += dm_size_x;
        }
    }

    private:
        unsigned int size_x_;
        unsigned int size_y_;
        double resolution_;
        double origin_x_;
        double origin_y_;
        int* occmap_;
        int default_value_;

        boost::recursive_mutex configuration_mutex_;


};  // Map2D class


}   // slam2d namespace