#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include <graph_based_slam/map.hpp>

namespace slam2d {

inline double square(double x) { return x * x; }

inline Eigen::Affine2d XYZEulertoAffineMatrix(const Eigen::Vector3d& x) {
    Eigen::Affine2d T = Eigen::Affine2d::Identity();
    T.translation() = x.head<2>();
    T.rotate(x[2]);
    return T;
}

struct Correspondence {
    Eigen::Vector2d p;
    Eigen::Vector2d q;
};

class ScanMatcher {
    public:
        Eigen::Affine2d registration(const std::vector<Eigen::Vector2d>& scan,
                                     Map2D* map,
                                     const Eigen::Affine2d& initial_guess,
                                     double max_correspondence_distance);

        Eigen::Affine2d registration(const std::vector<Eigen::Vector2d>& scan,
                                     const std::vector<Eigen::Vector2d>& target,
                                     const Eigen::Affine2d& initial_guess,
                                     double max_correspondence_distance,
                                     Eigen::Matrix3d& Hess);

        void computeCovarianceAndMean(std::vector<Correspondence>& corres,
                                      Eigen::Vector2d& mean_ref,
                                      Eigen::Vector2d& mean_cur,
                                      Eigen::Matrix2d& H);
        
        double computeSumError(std::vector<Correspondence>& corres);
    private:
        void transformPoints(const Eigen::Affine2d& T,
                             std::vector<Eigen::Vector2d>& points);
        
        // Scan to map
        void getCorrespondences(const std::vector<Eigen::Vector2d>& source,
                                Map2D *map,
                                double max_correspondence_distance,
                                std::vector<Correspondence>& corres);
        
        // Scan to scan
        void getCorrespondences(const std::vector<Eigen::Vector2d>& source,
                                const std::vector<Eigen::Vector2d>& target,
                                double max_correspondence_distance,
                                std::vector<Correspondence>& corres);

        int max_iterations_ = 500;
        double epsilon_ = 1e-4;

};


}   // slam2d namespace