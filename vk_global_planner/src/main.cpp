#include <vk_global_planner/rrt_star.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_global_planner");
    ros::NodeHandle nh;
    while (!ros::Time::now().toSec()) {
        ros::Duration(0.1).sleep();
    }
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    vk_global_planner::rrtstar planner(buffer);
    ros::waitForShutdown();
    return 0;
}