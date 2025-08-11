#pragma once

#include "header.hpp"

class AdaptiveThreshold {
    private:
        double initial_threshold;
        double min_motion;
        double max_range;
        Eigen::Matrix3d model_deviation;

        double model_error_sse2 = 0;
        int num_samples = 0;
        int max_samples_ = 1e3;
    public:
        AdaptiveThreshold() {};
        AdaptiveThreshold(double initial_threshold_, double min_motion_, double max_range_)
            : initial_threshold(initial_threshold_),
              min_motion(min_motion_),
              max_range(max_range_) {}
        
        double ComputeModelError(Eigen::Matrix3d& model_deviation_, double max_range_) {
            double trace_val = model_deviation_.block<2, 2>(0, 0).trace() / 2.0;
            trace_val = std::max(-1.0, std::min(1.0, trace_val));
            double theta = std::acos(trace_val);
            double delta_rot = 2.0 * max_range_ * std::sin(theta / 2.0);
            double delta_trans = model_deviation_.block<2, 1>(0, 2).norm();
            return delta_trans + delta_rot;
        }

        double ComputeThreshold() {
            double model_error = ComputeModelError(model_deviation, max_range);
            if(model_error > min_motion) {
                if(num_samples > max_samples_) {
                    model_error_sse2 = 0.0;
                    num_samples = 0;
                    return initial_threshold;
                }
                model_error_sse2 += model_error * model_error;
                num_samples++;
            }
            if(num_samples < 1) {
                return initial_threshold;
            }
            return sqrt(model_error_sse2 / num_samples);
        }

        void UpdateModelDeviation(const Eigen::Matrix3d& current_deviation) {
            model_deviation = current_deviation;
        }
};