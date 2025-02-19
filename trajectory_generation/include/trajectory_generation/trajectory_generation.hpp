#pragma once
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <trajectory_generation/quintic_bezier_splines.hpp>
#include <trajectory_generation/velocity_profile.hpp>

class Trajectory {
    public:
        Trajectory();
        
        ~Trajectory();
        
    private:
        NumericalVelocityProfile* vel_profile_;
        QuinticBezierSpline* spline_;

        void pathCallback(const nav_msgs::PathConstPtr& msg);

        // Reference pose at time t
        void getPose(geometry_msgs::Pose& pose, double t);

        ros::NodeHandle private_nh_, nh_;
        ros::Publisher traj_pub_;
        ros::Subscriber path_sub_;

        double v_start_, v_end_;
        double v_maxtrans_, v_maxrot_;
        double a_maxtrans_, a_maxrot_;
        double centrforce_max_, mass_;
        double scalingCoefficient_;
        bool use_orientation_robot_;

        std::string global_frame_;
};