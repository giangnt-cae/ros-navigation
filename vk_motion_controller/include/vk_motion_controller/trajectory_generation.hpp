#pragma once
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vk_motion_controller/quintic_bezier_splines.hpp>
#include <vk_motion_controller/velocity_profile.hpp>

class Trajectory {
    public:
        Trajectory(ros::NodeHandle& nh_);
        std::string getGlobalFrame() { return global_frame_; }
        std::string getBaseFrame() { return base_frame_; }

        double getMaxVelocityLinear() { return v_maxtrans_; }
        double getMaxvelocityAngular() { return v_maxrot_; }

        // Reference pose at time t (x, y, theta)
        void getPose(geometry_msgs::Pose& pose, double t);

        // Reference state at time t (x, y, theta, v, w)
        void getPoseAndVelocity(double (&x)[5], double t);

        bool getUpdated() { return updated_; }

        void setUpdated(bool updated) { updated_ = updated; }

        geometry_msgs::Pose getGlobalGoal() { return nav_goal_; }
        
        ~Trajectory();
        
    private:
        NumericalVelocityProfile* vel_profile_;
        QuinticBezierSpline* spline_;

        void pathCallback(const nav_msgs::PathConstPtr& msg);
        
        ros::Publisher traj_pub_;
        ros::Subscriber path_sub_;

        double v_start_, v_end_, anpha_;
        double v_maxtrans_, v_maxrot_;
        double a_maxtrans_, a_maxrot_;
        double centrforce_max_, mass_;
        double scalingCoefficient_;
        bool use_orientation_robot_;
        std::string global_frame_, base_frame_;
        bool updated_;
        geometry_msgs::Pose nav_goal_;

        std::mutex mtx_;
};