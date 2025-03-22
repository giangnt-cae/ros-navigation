#pragma once

#include <eigen3/Eigen/Dense>

double mahalanobisDistance(const Node& ni, const Node& nj) {
    Eigen::Vector3d u = nj.pose - ni.pose;
    u[2] = angle_diff(nj.pose[2], ni.pose[2]);

    // Mahalanobis distance: d = sqrt(u^T * Cov_inv * u)
    return std::sqrt(std::fabs(u.transpose() * ni.inv_covar * u));
}