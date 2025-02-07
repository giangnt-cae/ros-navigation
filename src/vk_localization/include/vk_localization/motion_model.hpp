#pragma once

#include "header.hpp"

double normalize(double z);

double angle_diff(double a, double b);

class Odom {
    public:
        Odom();
        bool UpdateMotion(Eigen::Vector3d& u_t_1,
                          Eigen::Vector3d& u_t,
                          Eigen::Vector3d& x);
        // Odometric pose
        Eigen::Vector3d pose;
        // Change in odometric pose
        Eigen::Vector3d delta;
        std::string model_type;
    private:
        double time;
};

class Laser {
    public:
        Laser(int max_beams);
        void SetLaserPose(const Eigen::Vector3d& laser_pose) {
            this->laser_pose = laser_pose;
        }
        void computePointCloud(std::vector<Eigen::Vector2d>& points);
        int range_count;
        double range_min;
        double range_max;
        double (*ranges)[2];

        ~Laser() {delete [] ranges;};
    private:
        double time;
        int max_beams;
        Eigen::Vector3d laser_pose;
};

