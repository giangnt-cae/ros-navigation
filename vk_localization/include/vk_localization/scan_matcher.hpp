#pragma once

#include "header.hpp"
#include "map.hpp"

inline double square(double x) { return x * x; }

Eigen::Matrix3d ConvertToHomogeneous(const Eigen::Vector3d& x);

struct Correspondence {
    Eigen::Vector2d p;
    Eigen::Vector2d q;
};

class ScanMatcher {
    private:
        void TransformPoints(const Eigen::Matrix3d& T,
                             std::vector<Eigen::Vector2d>& points);

        void GetCorrespondences(const std::vector<Eigen::Vector2d>& source,
                                map_t *map,
                                double max_correspondence_distance,
                                std::vector<Correspondence>& Corres);

        void BuildLinearSystem(const std::vector<Correspondence>& Corres,
                               double kernel,
                               Eigen::Matrix3d& JTJ,
                               Eigen::Vector3d& JTr);
        int max_iterations = 50;
        double epsilon = 1e-3;
    public:
        Eigen::Matrix3d Registration(const std::vector<Eigen::Vector2d>& scan,
                                     map_t *map,
                                     const Eigen::Matrix3d& initial_guess,
                                     double max_correspondence_distance,
                                     double kernel);
};