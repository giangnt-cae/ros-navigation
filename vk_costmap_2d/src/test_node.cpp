#include "ros/ros.h"
#include "vk_costmap_2d/AgvInfo.h"
#include "vk_costmap_2d/AgvInfoArray.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<vk_costmap_2d::AgvInfoArray>("agv_info", 10);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        vk_costmap_2d::AgvInfoArray msgarr;

        vk_costmap_2d::AgvInfo msg1;
        msg1.id = "AGV001";
        msg1.manufacturer = "Robotics Inc.";
        msg1.model = "X200";
        msg1.serial_number = "12345";
        msg1.state = "moving";
        msg1.battery_level = 85.5;
        msg1.current_pose.position.x = 1.0;
        msg1.current_pose.position.y = 0.5;
        msg1.current_pose.position.z = 0.0;
        msg1.width = 0.5;
        msg1.length = 1.0;

        msgarr.agvs.push_back(msg1);
    
        pub.publish(msgarr);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}