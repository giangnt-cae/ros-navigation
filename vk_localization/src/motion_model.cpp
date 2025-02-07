#include "vk_localization/motion_model.hpp"

double normalize(double z) {
    return atan2(sin(z), cos(z));
}

double angle_diff(double a, double b) {
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a - b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0) d2 *= -1.0;
    if(fabs(d1) < fabs(d2)) {
        return d1;
    }else {
        return d2;
    }
}

bool Odom::UpdateMotion(Eigen::Vector3d& u_t_1,
                        Eigen::Vector3d& u_t,
                        Eigen::Vector3d& x) {
    if(this->model_type == "omni") {
        double delta_trans, delta_rot, delta_bearing;
        delta_trans = (u_t.head(2) - u_t_1.head(2)).norm();
        delta_rot = angle_diff(u_t[2], u_t_1[2]);
        delta_bearing = angle_diff(atan2(u_t[1] - u_t_1[1], u_t[0] - u_t_1[0]), u_t[2]) + x[2];

        Eigen::Vector3d delta_x = {delta_trans*cos(delta_bearing),
                                   delta_trans*sin(delta_bearing),
                                   delta_rot};
        x += delta_x;
        x[2] = normalize(x[2]);
        return true;
    } else if (this->model_type == "diff") {
        double delta_trans, delta_rot1, delta_rot2;
        delta_trans = (u_t.head(2) - u_t_1.head(2)).norm();
        if(delta_trans < 0.01) {
            delta_rot1 = 0.0;
        }else {
            delta_rot1 = angle_diff(atan2(u_t[1] - u_t_1[1], u_t[0] - u_t_1[0]), u_t[2]);
        }
        delta_rot2 = angle_diff(angle_diff(u_t[2], u_t_1[2]), delta_rot1);
        Eigen::Vector3d delta_x = {delta_trans*cos(x[2] + delta_rot1),
                                   delta_trans*sin(x[2] + delta_rot1),
                                   delta_rot1 + delta_rot2};
        x += delta_x;
        x[2] = normalize(x[2]);
        return true;
    } else {
        ROS_ERROR("\"model_type\" parameter is unvalid, \"omni\" or \"diff\" !");
        return false;
    }               
}

Odom::Odom() {
    this->time = 0.0;
}

Laser::Laser(int max_beams) {
    this->time = 0.0;
    this->max_beams = max_beams;
}

void Laser::computePointCloud(std::vector<Eigen::Vector2d>& points) {
    Eigen::Vector2d p;
    int step = (range_count - 1) / (max_beams - 1);
    for(int i = 0; i < range_count; i += step) {
        if(ranges[i][0] >= range_max) continue;
        if(std::isnan(ranges[i][0])) continue;
        p[0] = laser_pose[0] + ranges[i][0] * cos(ranges[i][1]);
        p[1] = laser_pose[1] + ranges[i][0] * sin(ranges[i][1]);
        points.push_back(p);
    }
}