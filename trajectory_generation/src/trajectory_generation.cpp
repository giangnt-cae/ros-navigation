#include <trajectory_generation/trajectory_generation.hpp>

Trajectory::Trajectory() {
    private_nh_.param("v_start", v_start_, 0.0);
    private_nh_.param("v_end", v_end_, 0.0);
    private_nh_.param("v_maxtrans", v_maxtrans_, 0.3);
    private_nh_.param("v_maxrot", v_maxrot_, 1.0);
    private_nh_.param("a_maxtrans", a_maxtrans_, 0.3);
    private_nh_.param("a_maxrot", a_maxrot_, 1.0);
    private_nh_.param("mass", mass_, 100.0);
    private_nh_.param("centrforce_max", centrforce_max_, 500.0);
    private_nh_.param("scalingCoefficient", scalingCoefficient_, 0.5);
    private_nh_.param("use_orientation_robot", use_orientation_robot_, true);

    private_nh_.param("global_frame", global_frame_, std::string("map"));

    traj_pub_ = private_nh_.advertise<geometry_msgs::PoseArray>("trajectory", 1);
    path_sub_ = private_nh_.subscribe<nav_msgs::Path>("global_path", 1, &Trajectory::pathCallback, this);

    vel_profile_ = new NumericalVelocityProfile(v_start_, v_end_, v_maxtrans_, v_maxrot_,
                                                a_maxtrans_, a_maxrot_,
                                                centrforce_max_, mass_, scalingCoefficient_, use_orientation_robot_);
    spline_ = vel_profile_->getSpline();

    #ifdef test
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.poses.resize(5);

    path_msg.poses[0].pose.position.x = -4.0;
    path_msg.poses[0].pose.position.y = -4.0;
    path_msg.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);

    path_msg.poses[1].pose.position.x = -3.0;
    path_msg.poses[1].pose.position.y = -1.0;

    path_msg.poses[2].pose.position.x = 0.0;
    path_msg.poses[2].pose.position.y = -3.0;

    path_msg.poses[3].pose.position.x = 1.0;
    path_msg.poses[3].pose.position.y = 2.0;

    path_msg.poses[4].pose.position.x = 4.0;
    path_msg.poses[4].pose.position.y = 4.0;
    path_msg.poses[4].pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 6);

    nav_msgs::PathConstPtr path_ptr = boost::make_shared<nav_msgs::Path>(path_msg);
    ros::Rate rate(10);
    while(ros::ok()) {
        pathCallback(path_ptr);
        rate.sleep();
    }
    #endif

    while(nh_.ok()) {
        ros::spinOnce();
    }
}

Trajectory::~Trajectory() {
    delete vel_profile_;
    delete spline_;
}

void Trajectory::pathCallback(const nav_msgs::PathConstPtr& msg) {
    if(spline_->setWaypoints(*msg)) {
        // Generate quintic bezier spline
        spline_->generationSpline();
        // Numerical Velocity Profile
        vel_profile_->setVelocityProfile();
        
        std::vector<RefPoint>* uts = vel_profile_->getVelocityProfile();
        double t_end = uts->back().t;
        double t_start = uts->front().t;
        double dt = 0.5;
        int N = (t_end - t_start) / dt;
        geometry_msgs::PoseArray traj;
        traj.header.frame_id = global_frame_;
        for(int i = 0; i <= N; i++) {
            double t = i * dt;
            geometry_msgs::Pose pose;
            getPose(pose, t);
            traj.poses.push_back(pose);
        }
        traj_pub_.publish(traj);
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_generation");
    Trajectory trajectory;
    return 0;
}