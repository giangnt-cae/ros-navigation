#pragma once

#include <ros/ros.h>
#include <vk_costmap_2d/costmap_layer.hpp>
#include <vk_costmap_2d/layered_costmap.hpp>
#include <vk_costmap_2d/observation.hpp>
#include <vk_costmap_2d/footprint.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

namespace vk_costmap_2d{

class ObstacleLayer : public CostmapLayer {
    public:
        ObstacleLayer() { costmap_ = NULL; }

        virtual ~ObstacleLayer() {}
        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double* min_x, double* min_y, double* max_x, double* max_y);
        virtual void updateCosts(vk_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

        void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg,
                               const boost::shared_ptr<vk_costmap_2d::ObservationBuffer>& buffer);
        // Xu ly du lieu laserscan inf to range_max
        void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& msg,
                                       const boost::shared_ptr<vk_costmap_2d::ObservationBuffer>& buffer);
        
        void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& msg,
                                const boost::shared_ptr<vk_costmap_2d::ObservationBuffer>& buffer);

        void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg,
                                 const boost::shared_ptr<vk_costmap_2d::ObservationBuffer>& buffer);
    protected:
        // Get the observations used to mark space
        bool getMarkingObservations(std::vector<vk_costmap_2d::Observation>& marking_observations) const;
        
        // Get the observations used to clear space
        bool getClearingObservations(std::vector<vk_costmap_2d::Observation>& clearing_observations) const;

        // Clear freespace based on one observation
        virtual void raytraceFreespace(const vk_costmap_2d::Observation& clearing_observation,
                                       double* min_x, double* min_y,
                                       double* max_x, double* max_y);
        // Tính toán và cập nhật các ranh giới của vùng mà raytrace 
        void updateRaytraceBounds(double ox, double oy, double wx, double wy,
                                  double range, double* min_x, double* min_y,
                                  double* max_x, double* max_y);
        
        std::vector<geometry_msgs::Point> transformed_footprint_;
        bool footprint_clearing_enabled_;
        void updateFootprint(double robot_x, double robot_y, double robot_yaw,
                             double* min_x, double* min_y,
                             double* max_x, double* max_y);

        std::string global_frame_;
        double max_obstacle_height_;

        laser_geometry::LaserProjection projector_; // Used to project laser scans into point clouds

        // Used for the observation message filters
        std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;

        // Used to make sure that transforms are available for each sensor
        std::vector<boost::shared_ptr<tf2_ros::MessageFilterBase> > observation_notifiers_;

        // Used to store observations from various sensors
        std::vector<boost::shared_ptr<vk_costmap_2d::ObservationBuffer> > observation_buffers_;

        // Used to store observation buffers used for marking obstacles
        std::vector<boost::shared_ptr<vk_costmap_2d::ObservationBuffer> > marking_buffers_;

        // Used to store observation buffers used for clearing obstacles
        std::vector<boost::shared_ptr<vk_costmap_2d::ObservationBuffer> > clearing_buffers_;

        bool rolling_window_;

        int combination_method_;

};  // ObstacleLayer class

}   // vk_costmap_2d namespace