#include "ros/ros.h"
#include "vk_costmap_2d/AgvInfo.h"
#include "vk_costmap_2d/AgvInfoArray.h"

#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // ros::Publisher pub = nh.advertise<vk_costmap_2d::AgvInfoArray>("agv_info", 10);

    // ros::Rate loop_rate(10);

    // while (ros::ok()) {
    //     vk_costmap_2d::AgvInfoArray msgarr;

    //     vk_costmap_2d::AgvInfo msg1;
    //     msg1.id = "AGV001";
    //     msg1.manufacturer = "Robotics Inc.";
    //     msg1.model = "X200";
    //     msg1.serial_number = "12345";
    //     msg1.state = "moving";
    //     msg1.battery_level = 85.5;
    //     msg1.current_pose.position.x = 1.0;
    //     msg1.current_pose.position.y = 0.5;
    //     msg1.current_pose.position.z = 0.0;
    //     msg1.width = 0.5;
    //     msg1.length = 1.0;

    //     msgarr.agvs.push_back(msg1);
    
    //     pub.publish(msgarr);

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("global_path", 10);
    while (!ros::Time::now().toSec()) {
        ROS_WARN("Waiting for ROS Time to be initialized...");
        ros::Duration(0.1).sleep();
    }
    bool first = false;
    ros::Rate loop_rate(10);
    while (ros::ok() && !first) {
        ros::spinOnce();
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.poses.resize(4);
    
        path_msg.poses[0].pose.position.x = 0.0;
        path_msg.poses[0].pose.position.y = 0.0;
        path_msg.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    
        path_msg.poses[1].pose.position.x = 1.0;
        path_msg.poses[1].pose.position.y = 0.0;
        
        path_msg.poses[2].pose.position.x = 2.0;
        path_msg.poses[2].pose.position.y = 0.0;

        path_msg.poses[3].pose.position.x = 2.5;
        path_msg.poses[3].pose.position.y = -1.0;
        path_msg.poses[3].pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);

        first = true;
        pub.publish(path_msg);
        
        loop_rate.sleep();
    }

    return 0;
}