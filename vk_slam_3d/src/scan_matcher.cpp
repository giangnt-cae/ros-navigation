#include <vk_slam_3d/scan_matcher.hpp>

namespace vk_slam_3d {

void ScanMatcher::transformPoints(const Eigen::Affine3d& T, std::vector<Eigen::Vector3d>& points) {
    for(auto& point : points) {
        point = T.rotation() * point + T.translation();
    }
}

void ScanMatcher::getCorrespondences(const std::vector<Eigen::Vector3d>& source,
                                     const VoxelHashMap::Ptr voxel_map,
                                     double max_correspondance_distance,
                                     std::vector<Correspondence>& Corres) {
    Corres.clear();
    Eigen::Vector3i voxel;
    std::vector<Eigen::Vector3d> neighbors;
    int wsize = 1;
    Correspondence corre;
    double voxel_size = voxel_map->getVoxelSize();
    for(auto& point : source) {
        auto kx = static_cast<int>(point[0] / voxel_size);
        auto ky = static_cast<int>(point[1] / voxel_size);
        auto kz = static_cast<int>(point[2] / voxel_size);

        Eigen::Vector3d closest_point;
        double closest_dist2 = std::numeric_limits<double>::max();
        std::vector<Eigen::Vector3i> voxels;
        for(int i = kx - wsize; i <= kx + wsize; i++) {
            for(int j = ky - wsize; j <= ky + wsize; j++) {
                for(int k = kz - wsize; k <= kz + wsize; k++) {
                    voxel = {i, j, k};
                    neighbors.clear();
                    neighbors = voxel_map->getPoints(voxel);
                    for(const auto& point_neighbor : neighbors) {
                        double dist2 = (point_neighbor - point).squaredNorm();
                        if(dist2 < closest_dist2) {
                            closest_point = point_neighbor;
                            closest_dist2 = dist2;
                        }
                    }
                }
            }
        }
        if(sqrt(closest_dist2) < max_correspondance_distance) {
            corre.p = point;
            corre.q = closest_point;
            corre.dist2 = closest_dist2;
            Corres.push_back(corre);
        }
    }
}

void ScanMatcher::buildLinearSystem(const std::vector<Correspondence>& Corres,
                                    double kernel,
                                    Eigen::Matrix6d& JTJ,
                                    Eigen::Vector6d& JTr) {
    JTJ.setZero();
    JTr.setZero();                          
    Eigen::Matrix3x6d J_r;
    Eigen::Vector3d residual;
    int N = Corres.size();
    for(int i = 0; i < N; i++) {
        residual = Corres[i].p - Corres[i].q;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * SO3intolie(Corres[i].p);
        double w = square(kernel) / square(kernel + residual.squaredNorm());
        JTJ += J_r.transpose() * w * J_r;
        JTr += J_r.transpose() * w * residual;
    }
}

Eigen::Affine3d ScanMatcher::registration(const std::vector<Eigen::Vector3d>& currentScan,
                                          const VoxelHashMap::Ptr voxel_map,
                                          const Eigen::Affine3d& initial_guess,
                                          double max_correspondence_distance,
                                          double kernel) {
    std::vector<Eigen::Vector3d> source = currentScan;
    transformPoints(initial_guess, source);
    Eigen::Affine3d T_icp = Eigen::Affine3d::Identity();
    std::vector<Correspondence> Corres;
    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
    for(int i = 0; i < max_iterations; i++) {
        getCorrespondences(source, voxel_map, max_correspondence_distance, Corres);
        buildLinearSystem(Corres, kernel, JTJ, JTr);
        Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        Eigen::Affine3d deltaT = XYZEulertoAffineMatrix(dx);
        transformPoints(deltaT, source);
        T_icp = deltaT * T_icp;
        if(dx.norm() < epsilon) { break; }
    }
    return T_icp *initial_guess;                                                        
}

}   // namespace vk_slam_3d