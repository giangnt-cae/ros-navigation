#include <vk_slam_3d/graphbasedslam.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_slam_3d");
    ROS_INFO("Running vk_slam_3d node!");
    vk_slam_3d::GraphBasedSlam map3d_;
    return 0;
}