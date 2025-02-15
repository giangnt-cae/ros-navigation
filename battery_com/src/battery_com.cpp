#include "battery_com/battery_com.hpp"

int main(int argc,char **argv) {
    ROS_INFO("Running battery communication node!");
    ros::init(argc, argv, "battery_com");
    ros::NodeHandle nh;
    BATTERY battery_com(&nh);
    return 0;
}