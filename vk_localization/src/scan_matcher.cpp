#include "vk_localization/scan_matcher.hpp"

Eigen::Matrix3d ConvertToHomogeneous(const Eigen::Vector3d& x) {
    Eigen::Matrix3d T;
    double cos_theta = cos(x[2]);
    double sin_theta = sin(x[2]);
    T << cos_theta,     -sin_theta,     x[0],
         sin_theta,     cos_theta,      x[1],
         0,             0,              1;
    return T;
}

void ScanMatcher::TransformPoints(const Eigen::Matrix3d& T,
                                  std::vector<Eigen::Vector2d>& points) {
    Eigen::Matrix2d R = T.block<2, 2>(0, 0);
    Eigen::Vector2d t = T.block<2, 1>(0, 2);
    for(auto& point : points) {
        point = R * point + t;
    }                                
}

void ScanMatcher::GetCorrespondences(const std::vector<Eigen::Vector2d>& source,
                                     map_t *map,
                                     double max_correspondence_distance,
                                     std::vector<Correspondence>& Corres) {
    Corres.clear();
    Correspondence core;
    int i, j;
    map_cell_t *cell;
    map_cell_t *cell_;
    for(auto& point : source) {
        cell = map_get_cell(map, point);
        if(cell != NULL) {
            if(cell->occ_dist < max_correspondence_distance) {
                core.p = point;
                core.q = cell->nearest_cell;
                Corres.push_back(core);
            }
        }
    }                                  
}

void ScanMatcher::BuildLinearSystem(const std::vector<Correspondence>& Corres,
                                    double kernel,
                                    Eigen::Matrix3d& JTJ,
                                    Eigen::Vector3d& JTr) {
    JTJ.setZero();
    JTr.setZero();                          
    Eigen::Matrix<double, 2, 3> J_r;
    Eigen::Vector2d residual;
    int N = Corres.size();
    double w;
    for(int i = 0; i < N; i++) {
        residual = Corres[i].p - Corres[i].q;
        J_r.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
        J_r(0, 2) = -Corres[i].p[1];
        J_r(1, 2) = Corres[i].p[0];
        w = square(kernel)/ square(kernel + residual.squaredNorm());
        JTJ.noalias() += J_r.transpose() * w * J_r;
        JTr.noalias() += J_r.transpose() * w * residual;
    }                                 
}

Eigen::Matrix3d ScanMatcher::Registration(const std::vector<Eigen::Vector2d>& scan,
                                          map_t *map,
                                          const Eigen::Matrix3d& initial_guess,
                                          double max_correspondence_distance,
                                          double kernel) {
    std::vector<Eigen::Vector2d> source = scan;
    TransformPoints(initial_guess, source);
    Eigen::Matrix3d T_icp = Eigen::Matrix3d::Identity();
    std::vector<Correspondence> Corres;
    Eigen::Matrix3d JTJ;
    Eigen::Vector3d JTr;
    Eigen::Matrix3d deltaT;
    Eigen::Vector3d dx;
    for(int i = 0; i < max_iterations; i++) {
        GetCorrespondences(source, map, max_correspondence_distance, Corres);
        if (Corres.empty()) {
            ROS_WARN("No correspondences found in iteration %d", i);
            break;
        }
        BuildLinearSystem(Corres, kernel, JTJ, JTr);
        dx = JTJ.ldlt().solve(-JTr);

        if (!dx.allFinite()) {
            ROS_WARN("dx contains NaN or Inf! ICP terminated early.");
            break;
        }
        
        deltaT = ConvertToHomogeneous(dx);
        TransformPoints(deltaT, source);
        T_icp = deltaT * T_icp;
        if(dx.norm() < epsilon) { break; }
    }
    return T_icp *initial_guess;                                                        
}

