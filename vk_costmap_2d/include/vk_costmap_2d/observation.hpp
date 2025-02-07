#pragma once
#include <ros/ros.h>
#include <vector>
#include <list>
#include <string>
#include <ros/time.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread.hpp>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace vk_costmap_2d {

class Observation {
    public:
        geometry_msgs::Point origin_;
        sensor_msgs::PointCloud2* cloud_;
        double obstacle_range_, raytrace_range_;

        Observation() : 
            cloud_(new sensor_msgs::PointCloud2()),
            obstacle_range_(0.0), raytrace_range_(0.0) {}

        Observation(geometry_msgs::Point& origin, const sensor_msgs::PointCloud2& cloud,
                    double obstacle_range, double raytrace_range) :
            origin_(origin),
            cloud_(new sensor_msgs::PointCloud2(cloud)),
            obstacle_range_(obstacle_range),
            raytrace_range_(raytrace_range) {}

        Observation(const Observation& obs) : origin_(obs.origin_), cloud_(new sensor_msgs::PointCloud2(*(obs.cloud_))),
                                              obstacle_range_(obs.obstacle_range_), raytrace_range_(obs.raytrace_range_) {}
        
        virtual ~Observation() { delete cloud_; }

};  // Observation class


class ObservationBuffer {
    public:
        ObservationBuffer(std::string topic_name, double observation_keep_time, double expected_update_rate,
                    double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                    double raytrace_range, tf2_ros::Buffer& tf2_buffer, std::string global_frame,
                    std::string sensor_frame, double tf_tolerance);
        ~ObservationBuffer();

        // Transforms a PointCloud to the global frame and buffers it
        void bufferCloud(const sensor_msgs::PointCloud2& cloud);

        void getObservations(std::vector<Observation>& observations);

        bool isUpdated() const;
        boost::recursive_mutex lock_mutex_;

    private:
        void removeStaleObservations();

        tf2_ros::Buffer& tf2_buffer_;
        const ros::Duration observation_keep_time_;
        const ros::Duration expected_update_rate_;

        ros::Time last_updated_;
        std::string global_frame_;
        std::string sensor_frame_;
        std::list<Observation> observation_list_;
        std::string topic_name_;
        double min_obstacle_height_, max_obstacle_height_;
        double obstacle_range_, raytrace_range_;
        double tf_tolerance_;
};  // ObservationBuffer class

}   // namespace