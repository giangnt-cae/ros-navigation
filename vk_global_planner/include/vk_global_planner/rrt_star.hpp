#pragma once

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <random>
#include <iostream>
#include <math.h>
#include <vector>
#include <vk_global_planner/nanoflann.hpp>

namespace vk_global_planner {

template <typename Point1, typename Point2>  
inline double norm2d(const Point1* pt1, const Point2* pt2) {
    return std::sqrt((pt2->x - pt1->x) * (pt2->x - pt1->x) +
                     (pt2->y - pt1->y) * (pt2->y - pt1->y));
}

class rrtstar {
    public:
        rrtstar(tf2_ros::Buffer& tf);

        ~rrtstar() {};

        struct Cell {
            double x, y;
            unsigned int id;
            double cost;
            Cell* parent = nullptr;
        };

        struct CellCloud {
            std::vector<Cell*> pts;

            inline size_t kdtree_get_point_count() const { return pts.size(); }

            inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
                if(dim == 0) return pts[idx]->x;
                else return pts[idx]->y;
            }

            // Dont use bounding box
            template <class BBOX>
            bool kdtree_get_bbox(BBOX&) const { return false; }
        };

        using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<double, CellCloud>,
            CellCloud,
            2
        >;

        void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

        void goalCallback(const geometry_msgs::PoseStamped& msg);

        bool getCurrentPose(geometry_msgs::PoseStamped& global_pose);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber goal_sub_, map_sub_;
        ros::Publisher graph_pub_, planner_pub_;

        tf2_ros::Buffer& tf_;
        double transform_tolerance_;
        boost::recursive_mutex configuration_mutex_;
        std::string base_frame_, global_frame_;
        bool map_received_;

        unsigned int max_size_;
        double delta_, gama_;
        int free_;

        std::vector<Cell*> tree_;
        nav_msgs::OccupancyGrid map_;

        visualization_msgs::Marker marker_node_, marker_edge_;
        unsigned int marker_id_;
        std::map<Cell*, int> edge_marker_ids_;

        Cell* getCellSample(unsigned int n);

        bool collisionCheck(Cell* cell, Cell* cell_nearest);

        unsigned int getIndex(double& x, double& y);

        bool extend(Cell* cell, Cell* cell_nearest);

        Cell* getCellNearest(Cell* query_cell, CellCloud& cloud, KDTree& index);

        std::vector<Cell*> getNeighbors(Cell* query_cell,
                                        CellCloud& cloud,
                                        KDTree& index);

        double getRadiusNeighbor(int n);

        bool rapidlyRandomTreeAlgorithm(geometry_msgs::PoseStamped& init_pose,
                                        geometry_msgs::PoseStamped& goal_pose);
        void getPathPlanner(nav_msgs::Path& path, std::vector<Cell*>& cells);
        std::vector<Cell*> smoothingPath();

        void resetTree();

        void deleteMarker(Cell* cell);
        void addNodeMarker(Cell* cell);
        void addEdgeMarker(Cell* cell);
        void resetMarkers();
};

}   // vk_global_planner namespace