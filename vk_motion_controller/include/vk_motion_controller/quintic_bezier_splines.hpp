#pragma once
#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <Eigen/Core>
#include <Eigen/Dense>

class QuinticBezierSpline {
    public:
        QuinticBezierSpline(double scalingCoefficient, bool use_orientation_robot) :
            scalingCoefficient_(scalingCoefficient),
            use_orientation_robot_(use_orientation_robot) {}

        virtual void reset();
        
        virtual ~QuinticBezierSpline() {
            reset();
        }

        bool setWaypoints(const nav_msgs::Path& path);
        void generationSpline();
        
        std::vector<std::array<Eigen::Vector2d, 6>>& getSpline() { return segments_; }
        std::vector<double>& getArcLengths() { return arc_lengths_; }
        double getSumLength() { return sum_length_; }
        unsigned int getNumOfWaypoints() { return num_waypoints_; }

        double computeArcLength(double lower_limit, double upper_limit, int index_segment);
        double getCurvatureAtU(double u, int index_segment);
        
        void getState(double& x, double& y, double& theta, double u, int index_segment);
        void getState(double (&x)[5], double u, double u_dot, int index_segment);
    
    private:
        std::vector<Eigen::Vector2d> waypoints_;
        std::vector<double> waypoint_orientations_;                    // Waypoints from path planner
        std::vector<std::array<Eigen::Vector2d, 6>> segments_;      // The quintic polynomial coefficients of the segments 
        std::vector<double> arc_lengths_;
        double sum_length_;
        double scalingCoefficient_;
        unsigned int num_waypoints_;
        bool use_orientation_robot_;
        double theta_start_;                                        // Orientation of robot at current
        double theta_goal_;                                         //  Orientation of robot at goal

        std::vector<double> computeDistanceOfSegments();
        std::vector<Eigen::Vector2d> computeFirstDerivativeHeuristics(std::vector<double>& distanceOfSegments);
        std::vector<Eigen::Vector2d> computeSecondDerivativeHeuristics(std::vector<double>& distanceOfSegments,
                                                                       std::vector<Eigen::Vector2d>& first_derivatives);

        // Compute second derivative at point if point is end point of segment
        Eigen::Vector2d computeSecondDerivativeAtEndPoint(Eigen::Vector2d& A, Eigen::Vector2d& tA,
                                                          Eigen::Vector2d& B, Eigen::Vector2d& tB);
        // Compute second derivative at point if point is begin point of segment
        Eigen::Vector2d computeSecondDerivativeAtBeginPoint(Eigen::Vector2d& B, Eigen::Vector2d& tB,
                                                            Eigen::Vector2d& C, Eigen::Vector2d& tC);
        // Compute the control points of the segment
        std::array<Eigen::Vector2d, 6> computeControlPointsOfSegment(Eigen::Vector2d& A, Eigen::Vector2d& tA, Eigen::Vector2d& aA,
                                                                     Eigen::Vector2d& B, Eigen::Vector2d& tB, Eigen::Vector2d& aB);
        // Compute the quintic polynomial coefficients of the segment
        std::array<Eigen::Vector2d, 6> computeCoefficientOfSegment(std::array<Eigen::Vector2d, 6>& control_points);

        double computeSpeedAtU(double u, int index_segment);
        double rombergIntegrator(double a, double b, int index_segment);
        void computeArcLengths();
};