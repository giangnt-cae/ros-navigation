#pragma once

#include <graph_based_slam/graph_based_slam.hpp>
#include <eigen3/Eigen/Dense>

namespace slam2d {

double mahalanobisDistance(const Node& ni, const Node& nj) {
    Eigen::Vector3d u = nj.pose - ni.pose;
    u[2] = angle_diff(nj.pose[2], ni.pose[2]);
    return std::sqrt(u.transpose() * ni.omega * u);
}

}   // slam2d namespace