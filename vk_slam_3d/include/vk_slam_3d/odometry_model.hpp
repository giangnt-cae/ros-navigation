#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Core>

namespace vk_slam_3d {

double normalize(double z);

double angle_diff(double a, double b);

class Odom {
    public:
        Odom();

        bool UpdateMotion(Eigen::Matrix<double, 6, 1>& u_t_1,
                          Eigen::Matrix<double, 6, 1>& u_t,
                          Eigen::Matrix<double, 6, 1>& x);
        
        // Odometric pose
        Eigen::Matrix<double, 6, 1> pose;

        // Change in odometric pose
        Eigen::Matrix<double, 6, 1> delta;

        // Model type: "omni" or "diff"
        std::string model_type;
    private:
        double time;
};  // class Odom

}   // namespace vk_slam_3d