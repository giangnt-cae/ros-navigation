#pragma once
#include <ros/ros.h>
#include <string.h>
#include <tf2_ros/message_filter.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

namespace vk_costmap_2d {

struct MapLocation {
  unsigned int x;
  unsigned int y;
};

void bresenham2D(std::vector<MapLocation>& cells, 
                 MapLocation pt_begin, MapLocation pt_end,
                 double max_length = UINT_MAX);

class Costmap2D {
  protected:
    unsigned int size_x_;
    unsigned int size_y_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    unsigned char* costmap_;
    unsigned char default_value_;

    virtual void initMaps(unsigned int size_x, unsigned int size_y);
    virtual void resetFullMap();
    virtual void deleteMaps();

  public:
    Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
              double origin_x, double origin_y, unsigned char default_value);
    Costmap2D();
    virtual ~Costmap2D();
    
    void resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                   double origin_x, double origin_y);
    
    void resetRegionOfMap(unsigned int x0, unsigned int y0,
                          unsigned int xn, unsigned int yn);
    
    virtual void updateOrigin(double new_origin_x, double new_origin_y);

    unsigned int getSizeInCellsX() const { return size_x_; }
    unsigned int getSizeInCellsY() const { return size_y_; }
    double getResolution() const { return resolution_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }

    double getSizeInMetersX() const { return (size_x_ - 1 + 0.5) * resolution_; }
    double getSizeInMetersY() const { return (size_y_ - 1 + 0.5) * resolution_; }

    unsigned char* getCharMap() const { return costmap_; }

    unsigned char getDefaultValue() { return default_value_; }
    void setDefaultValue(unsigned char c) { default_value_ = c; }

    unsigned char getCost(unsigned int mx, unsigned int my) const;
    void setCost(unsigned int mx, unsigned int my, unsigned char cost);

    inline unsigned int getIndex(unsigned int mx, unsigned int my) const {
      return my * size_x_ + mx;
    }

    inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const {
      my = index / size_x_;
      mx = index - (my * size_x_);
    }

    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;

    void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const;
    void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const;

    unsigned int cellDistance(double world_dist);

    bool setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value);
    void polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);
    void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);
    void raytraceLine(unsigned char value, unsigned int x0, unsigned int y0,
                      unsigned int x1, unsigned int y1, unsigned int max_length = UINT_MAX);
    boost::recursive_mutex configuration_mutex_;
    
  protected:
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

};  // Costmap2D class

}   // namespace vk_cost_map_2d