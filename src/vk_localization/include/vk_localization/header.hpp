#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/SetMap.h>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <queue>
#include <cmath>
#include <limits>
#include <signal.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

inline std::string stripSlash(const std::string& in) {
    std::string out = in;
    if(( !in.empty() ) && (in[0] == '/')) {
        out.erase(0, 1);
    }
    return out;
}
