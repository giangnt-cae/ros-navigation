#include "ros/ros.h"
#include "vk_costmap_2d/AgvInfo.h"
#include "vk_costmap_2d/AgvInfoArray.h"

ros::Publisher agvs_pub_;
ros::Subscriber agv_sub_;
void agvInfoCallback(const vk_costmap_2d::AgvInfo& msg) {
    vk_costmap_2d::AgvInfoArray agvs_msg;
    agvs_msg.agvs.clear();
    agvs_msg.agvs.push_back(msg);
    agvs_pub_.publish(agvs_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh_;
    agvs_pub_ = nh_.advertise<vk_costmap_2d::AgvInfoArray>("agvs_info", 10);
    agv_sub_ = nh_.subscribe("/vk_costmap_2d/agv_info", 10, agvInfoCallback);
    ros::spin();
    return 0;
}