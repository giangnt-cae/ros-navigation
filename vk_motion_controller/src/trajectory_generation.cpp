#include <vk_motion_controller/trajectory_generation.hpp>

Trajectory::Trajectory(ros::NodeHandle& nh_) {
    nh_.param("v_start", v_start_, 0.0);
    nh_.param("v_end", v_end_, 0.0);
    nh_.param("anpha", anpha_, 0.5);
    nh_.param("v_maxtrans", v_maxtrans_, 0.5);
    nh_.param("v_maxrot", v_maxrot_, 0.5);
    nh_.param("a_maxtrans", a_maxtrans_, 0.25);
    nh_.param("a_maxrot", a_maxrot_, 0.25);
    nh_.param("centrforce_max", centrforce_max_, 500.0);
    nh_.param("mass", mass_, 100.0);
    nh_.param("scalingCoefficient", scalingCoefficient_, 0.5);
    nh_.param("use_orientation_robot", use_orientation_robot_, false);

    nh_.param("global_frame", global_frame_, std::string("map"));
    nh_.param("base_frame", base_frame_, std::string("base_link"));

    updated_ = false;

    traj_pub_ = nh_.advertise<nav_msgs::Path>("trajectory", 1);
    path_sub_ = nh_.subscribe<nav_msgs::Path>("global_path", 1, &Trajectory::pathCallback, this);

    vel_profile_ = new NumericalVelocityProfile(v_start_, v_end_, anpha_, v_maxtrans_, v_maxrot_,
                                                a_maxtrans_, a_maxrot_,
                                                centrforce_max_, mass_, scalingCoefficient_, use_orientation_robot_);
    spline_ = vel_profile_->getSpline();
}

void Trajectory::pathCallback(const nav_msgs::PathConstPtr& msg) {
    ROS_INFO("Received new path.");
    std::lock_guard<std::mutex> lock(mtx_);
    if(spline_->setWaypoints(*msg)) {
        spline_->generationSpline();
        vel_profile_->setVelocityProfile();
        std::vector<RefPoint>* uts = vel_profile_->getVelocityProfile();

        nav_msgs::Path traj;
        geometry_msgs::PoseStamped pose_stamped;
        double t_end = uts->back().t;
        double t_start = uts->front().t;
        double dt = 0.05;
        int N = (t_end - t_start) / dt;
        traj.header.frame_id = global_frame_;
        for(int i = 0; i <= N; i++) {
            double t = i * dt;
            getPose(pose_stamped.pose, t);
            traj.poses.push_back(pose_stamped);
        }
        traj_pub_.publish(traj);

        // Set global goal
        getPose(nav_goal_, t_end);
        updated_ = true;
    }
}

void Trajectory::getPose(geometry_msgs::Pose& pose, double t) {
    std::vector<RefPoint>* uts = vel_profile_->getVelocityProfile();
    double x, y, theta;
    if(t > uts->back().t)
        t = uts->back().t;
    else if(t < uts->front().t)
        t = uts->front().t;

    int left = 0, right = vel_profile_->getNumOfPoints();
    while(left < right) {
        int mid = left + (right - left) / 2;
        if((*uts)[mid].t > t) {
            right = mid;
        }else {
            left = mid + 1;
        }
    }
    if(left == vel_profile_->getNumOfPoints())
        left -= 1;

    // Noi suy t trong khoang [ti, tj]
    double ui = (*uts)[left - 1].u, uj = (*uts)[left].u;
    double ti = (*uts)[left - 1].t, tj = (*uts)[left].t;
    unsigned int k = (*uts)[left -1].index;
    double u;
    if(uj > ui) {
        u = ui + (t - ti) * (uj - ui) / (tj - ti);
    }else {
        double t0 = ti + (tj - ti) * (1 - ui) / (1 - ui + uj);
        if(t <= t0) {
            u = ui + (t - ti) * (1 - ui) / (t0 - ti);
        }else {
            u = (t - t0) * (uj) / (tj - t0);
            k = (*uts)[left].index;
        }
    }
    spline_->getState(x, y, theta, u, k);
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation = tf::createQuaternionMsgFromYaw(theta);
}

void Trajectory::getPoseAndVelocity(double (&x)[5], double t) {
    std::vector<RefPoint>* uts = vel_profile_->getVelocityProfile();
    if(t > uts->back().t)
        t = uts->back().t;
    else if(t < uts->front().t)
        t = uts->front().t;

    int left = 0, right = vel_profile_->getNumOfPoints();
    while(left < right) {
        int mid = left + (right - left) / 2;
        if((*uts)[mid].t > t) {
            right = mid;
        }else {
            left = mid + 1;
        }
    }
    if(left == vel_profile_->getNumOfPoints())
        left -= 1;

    // Noi suy t trong khoang [ti, tj]
    double ui = (*uts)[left - 1].u, uj = (*uts)[left].u;
    double ti = (*uts)[left - 1].t, tj = (*uts)[left].t;
    unsigned int k = (*uts)[left -1].index;
    
    double u, u_dot;
    if(uj > ui) {
        u = ui + (t - ti) * (uj - ui) / (tj - ti);
        u_dot = (uj - ui) / (tj - ti);
    }else {
        double t0 = ti + (tj - ti) * (1 - ui) / (1 - ui + uj);
        if(t <= t0) {
            u = ui + (t - ti) * (1 - ui) / (t0 - ti);
            u_dot = (1 - ui) / (t0 - ti);
        }else {
            u = (t - t0) * (uj) / (tj - t0);
            u_dot = uj/ (tj - t0);
            k = (*uts)[left].index;
        }
    }
    spline_->getState(x, u, u_dot, k);
}

Trajectory::~Trajectory() {
    delete vel_profile_;
    delete spline_;
}