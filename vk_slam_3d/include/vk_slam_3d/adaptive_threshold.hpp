#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace vk_slam_3d {

class AdaptiveThreshold {
    private:
        double initial_threshold_;
        double min_motion_;
        double max_range_;
        Eigen::Affine3d model_deviation_;

        double model_error_sse2_ = 0;
        int num_samples_ = 0;

    public:
        AdaptiveThreshold(double initial_threshold, double min_motion, double max_range)
            : initial_threshold_(initial_threshold),
            min_motion_(min_motion),
            max_range_(max_range) {}

        double ComputeModelError() {
            const double theta = acos((model_deviation_.rotation().trace() - 1.0) / 2.0);
            const double delta_rot = 2.0 * max_range_ * sin(theta / 2.0);
            const double delta_trans = model_deviation_.translation().norm();
            return delta_trans + delta_rot;
        }

        double ComputeThreshold() {
            double model_error = ComputeModelError();
            if(model_error > min_motion_) {
                model_error_sse2_ += model_error * model_error;
                num_samples_++;
            }
            if(num_samples_ < 1)
                return initial_threshold_;
            return sqrt(model_error_sse2_ / num_samples_);
        }

        void UpdateModelDeviation(const Eigen::Affine3d& current_deviation) {
            model_deviation_ = current_deviation;
        }

};  // class AdaptiveThreshold

}   // namespce vk_slam_3d