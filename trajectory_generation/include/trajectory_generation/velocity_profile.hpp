#pragma once
#include <ros/ros.h>

#include <algorithm>
#include <trajectory_generation/quintic_bezier_splines.hpp>

struct RefPoint {
    double u;
    double t;
    unsigned int index;
};

class NumericalVelocityProfile {
    public:
        NumericalVelocityProfile(double v_start, double v_end,
                                 double v_maxtrans, double v_maxrot,
                                 double a_maxtrans, double a_maxrot,
                                 double centrforce_max, double mass,
                                 double scalingCoefficient, bool use_orientation_robot);

        ~NumericalVelocityProfile() { std::vector<RefPoint>().swap(uts_); }

        double getDeltaS() { return deltaS_; }
        int getNumOfPoints() { return num_points_; }
        void setVelocityProfile();

        QuinticBezierSpline* getSpline() { return &spline_; }
        std::vector<RefPoint>* getVelocityProfile() { return &uts_; }

    private:
        QuinticBezierSpline spline_;
        double v_start_, v_end_;
        double deltaS_;             // Arc length between planning points

        double v_maxtrans_, v_maxrot_;  
        double a_maxtrans_, a_maxrot_;
        double centrforce_max_;     // F_centr = m * R * w^2
        double mass_;

        std::vector<RefPoint> uts_;
        int num_points_;

        void computeDeltaS();
        RefPoint computeU(int index);
        double getCurve(double u, unsigned int k);
        double maxVelocityCurve(double& c);
        double maxVelocityForce(double& c);

        double isolatedConstraints(double& c);
        void translationAccelerationConstraints(double& vi_1, double& vi);
        void rotationAccelerationConstraints();
        void computeTime(double& vi_1, double& vi, double& Ti_1, double& Ti);   
};