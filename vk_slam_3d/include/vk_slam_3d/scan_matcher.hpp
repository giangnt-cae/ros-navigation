#pragma once
#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tuple>
#include <thread>
#include <mutex>
#include <boost/thread.hpp>

#include "vk_slam_3d/voxel_map.hpp"

namespace Eigen {
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix3x6d = Eigen::Matrix<double, 3, 6>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
}

namespace vk_slam_3d {

/* Convert 3D vector into the correspondening matrix of a lie algebra element */
inline Eigen::Matrix3d SO3intolie(const Eigen::Vector3d& p) {
    Eigen::Matrix3d m;
    m << 0.0, -p[2], p[1],
         p[2], 0.0, -p[0],
         -p[1], p[0], 0.0;
    return m;
}

inline Eigen::Affine3d XYZEulertoAffineMatrix(const Eigen::Vector6d& x) {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();

    T.translation() = x.head<3>();

    T.rotate(Eigen::AngleAxisd(x[5], Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(x[4], Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(x[3], Eigen::Vector3d::UnitX()));

    return T;
}

inline double square(double x) { return x * x; }

struct Correspondence {
    Eigen::Vector3d p;
    Eigen::Vector3d q;
    double dist2;
};

class ScanMatcher {
    private:
        void transformPoints(const Eigen::Affine3d& T,
                             std::vector<Eigen::Vector3d>& points);

        void getCorrespondences(const std::vector<Eigen::Vector3d>& source,
                                const VoxelHashMap::Ptr voxel_map,
                                double max_correspondance_distance,
                                std::vector<Correspondence>& Corres);

        void buildLinearSystem(const std::vector<Correspondence>& Corres,
                               double kernel,
                               Eigen::Matrix6d& JTJ,
                               Eigen::Vector6d& JTr);
        
        int max_iterations = 500;
        double epsilon = 1e-4;
    public:
        Eigen::Affine3d registration(const std::vector<Eigen::Vector3d>& currentScan,
                                     const VoxelHashMap::Ptr voxel_map,
                                     const Eigen::Affine3d& initial_guess,
                                     double max_correspondence_distance,
                                     double kernel);
        
};  // class ScanMatcher

}   // namespace vk_slam_3d