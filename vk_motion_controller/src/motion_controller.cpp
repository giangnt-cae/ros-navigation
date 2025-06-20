#include <vk_motion_controller/motion_controller.hpp>
MotionController::MotionController(tf2_ros::Buffer& tf)
    : Q_(DM::diagcat({5, 5, 2})),
    R_(DM::diagcat({0.1, 0.1})),
    W_(DM::diagcat({0.5, 0.5})),
    Q_N_(DM::diagcat({10, 10, 5})),
    tf_(tf),
    transform_tolerance_(0.5),
    ref_traj_(NULL),
    private_nh_("~")
{   
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    local_planner_pub_ = nh_.advertise<geometry_msgs::PoseArray>("local_planner", 10);
    reference_pub_ = nh_.advertise<geometry_msgs::PoseArray>("references", 10);
    obstacle_sub_ = nh_.subscribe("obstacles", 10, &MotionController::obstacleCallback, this);

    init();

    run();
}

void MotionController::run() {
    int numX = X.numel(), numU = U.numel(), numP = P.numel();
    int num_cst_collision = (N_horizon_ + 1)*sx_*sy_*N_maxobs_, num_cst_keep_lane = N_horizon_ - 1;
    DM p    = DM::zeros(numP, 1);
    DM x0_  = DM::zeros(numX, 1);
    DM u0_  = DM::zeros(numU, 1);
    ROS_INFO_STREAM("Using CasADi version " << casadi::CasadiMeta::version() 
                    << " with solver: IPOPT.");

    casadi::Function solver = nlpsol("solver", "ipopt", nlp_, opts_);

    ros::Time current_time = ros::Time::now(), last_time = ros::Time::now();

    double robot_pose[3];
    double x_ref[5];
    double dt;
    ros::Rate rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        current_time = ros::Time::now();
        if(ref_traj_->getUpdated() && getRobotPose(robot_pose) && !ref_traj_->getRobotIsGoal() && !has_collision_) {
            if(checkIsGoal(robot_pose)) {
                ref_traj_->setRobotIsGoal(true);
                continue;
            }
            
            dt = (current_time - last_time).toSec();

            p(Slice(0, num_states_)) = {robot_pose[0], robot_pose[1], robot_pose[2]};
            
            for(int i = 0; i < N_horizon_; i++) {
                ref_traj_->getPoseAndVelocity(x_ref, ref_traj_->getTimeStamp() + (i + 1) * T_s_);
                p(Slice(num_states_ * (i + 1), num_states_ * (i + 2))) = {x_ref[0], x_ref[1], x_ref[2]};
                p(Slice(numX + num_controls_ * i, numX + num_controls_ * (i + 1))) = {x_ref[3], x_ref[4]};
            }

            DM x_first = p(Slice(0, num_states_)), x_end = p(Slice(num_states_, 2 * num_states_));
            DM delta = diffStateVector(x_first, x_end);
            if((double)norm_2(delta(Slice(0, 2))) < max_error_position_)
                ref_traj_->updateTimeStamp(dt);

            double x_min = robot_pose[0] - width_map_ / 2.0;
            double x_max = robot_pose[0] + width_map_ / 2.0;
            double y_min = robot_pose[1] - height_map_ / 2.0;
            double y_max = robot_pose[1] + height_map_ / 2.0;

            casadi::DM lbx = DM::zeros(numX + numU, 1);
            lbx(Slice(0, numX, num_states_)) = x_min;
            lbx(Slice(1, numX, num_states_)) = y_min;
            lbx(Slice(2, numX, num_states_)) = -M_PI;

            lbx(Slice(numX, numX + numU, num_controls_))     = -ref_traj_->getMaxVelocityLinear();
            lbx(Slice(numX + 1, numX + numU, num_controls_)) = -ref_traj_->getMaxvelocityAngular();

            casadi::DM ubx = DM::zeros(numX + numU, 1);
            ubx(Slice(0, numX, num_states_)) = x_max;
            ubx(Slice(1, numX, num_states_)) = y_max;
            ubx(Slice(2, numX, num_states_)) = M_PI;

            ubx(Slice(numX, numX + numU, num_controls_))     = ref_traj_->getMaxVelocityLinear();
            ubx(Slice(numX + 1, numX + numU, num_controls_)) = ref_traj_->getMaxvelocityAngular();

            args_["lbx"] = lbx;
            args_["ubx"] = ubx;

            casadi::DM lbg = DM::zeros(cst_.numel(), 1);
            casadi::DM ubg = DM::zeros(cst_.numel(), 1);
            #ifdef MaxVerticesFilter
            for(auto& polygon : obstacles_) {
                polygon.filter(N_maxvertices_, robot_pose);
            }
            #endif
            if(avoidance_enable_ && !obstacles_.empty()) {
                filterObstacles(robot_pose);

                int n = numX + numU - 1;
                for(auto& polygon : obstacles_) {
                    p(++n) = polygon.centroid.x;
                    p(++n) = polygon.centroid.y;
                    p(++n) = polygon.radius;
                }
                ubg(Slice(numX, numX + num_cst_collision)) = casadi::inf;
            }else if(avoidance_enable_ && obstacles_.empty()) {
                lbg(Slice(numX, numX + num_cst_collision)) = -casadi::inf;
                ubg(Slice(numX, numX + num_cst_collision)) = casadi::inf;
            }

            if(keep_lane_)
                ubg(Slice(cst_.numel() - num_cst_keep_lane, cst_.numel())) = casadi::inf;

            args_["lbg"] = lbg;
            args_["ubg"] = ubg;

            args_["p"]   = p;
            args_["x0" ] = DM::vertcat({reshape(x0_, numX, 1),
                                        reshape(u0_, numU, 1)});
            
            casadi::DMDict res = solver(DMDict{{"x0",  args_["x0"]},
                                               {"lbx", args_["lbx"]},
                                               {"ubx", args_["ubx"]},
                                               {"lbg", args_["lbg"]},
                                               {"ubg", args_["ubg"]},
                                               {"p",   args_["p"]}});
            
            // std::map<std::string, casadi::GenericType> solver_stats = solver.stats();
            // double total_wall_time = 0.0;
            // for (const auto& kv : solver_stats) {
            //     if (kv.first.find("t_wall_") == 0) {
            //         total_wall_time += kv.second.as_double();
            //     }
            // }
            // ROS_INFO("---> Total t_wall_* time: %.6f seconds", total_wall_time);

            DM x_opt = (DM)res["x"];
            x0_      = x_opt(Slice(0, numX));
            u0_      = x_opt(Slice(numX, numX + numU));
            
            // Velocity publish
            geometry_msgs::Twist ctrl_vel;
            ctrl_vel.linear.x =  (double)u0_(0);
            ctrl_vel.angular.z = (double)u0_(1);
            vel_pub_.publish(ctrl_vel);

            // Local planner publish
            geometry_msgs::PoseArray local_path;
            local_path.header.frame_id = global_frame_;
            local_path.header.stamp = ros::Time::now();
            for(int i = 0; i <= N_horizon_; i++) {
                geometry_msgs::Pose pose;
                pose.position.x = (double)x0_(num_states_ * i);
                pose.position.y = (double)x0_(num_states_ * i + 1);
                pose.orientation = tf::createQuaternionMsgFromYaw((double)x0_(num_states_ * i + 2));
                local_path.poses.push_back(pose);
            }
            local_planner_pub_.publish(local_path);

            // Reference trajectory publish
            geometry_msgs::PoseArray ref_path;
            ref_path.header.frame_id = global_frame_;
            ref_path.header.stamp = ros::Time::now();
            for(int i = 0; i < N_horizon_; i++) {
                geometry_msgs::Pose pose;
                DM p_ref = p(Slice(num_states_ * (i+1), num_states_ * (i + 2)));
                pose.position.x = (double)p_ref(0);
                pose.position.y = (double)p_ref(1);
                pose.orientation = tf::createQuaternionMsgFromYaw((double)p_ref(2));
                ref_path.poses.push_back(pose);
            }
            reference_pub_.publish(ref_path);
        }
        last_time = current_time;
        rate.sleep();
    }
}

void MotionController::init() {
    private_nh_.param("horizon_size", N_horizon_, 15);
    private_nh_.param("horizon_control", N_controls_, 10);
    private_nh_.param("max_obstacles", N_maxobs_, 10);
    private_nh_.param("max_vertices", N_maxvertices_, 4);
    private_nh_.param("T_sample", T_s_, 0.5);
    private_nh_.param("min_obstacle_distance", min_obstacle_distance_, 0.05);
    private_nh_.param("cir_radius", cir_radius_, 0.25);
    private_nh_.param("max_error_position", max_error_position_, 0.5);
    private_nh_.param("max_error_angle", max_error_angle_, M_PI / 6);
    private_nh_.param("avoidance_enable", avoidance_enable_, true);
    private_nh_.param("keep_lane", keep_lane_, false);
    private_nh_.param("width_lane", width_lane_, 1.0);
    private_nh_.param("width_map", width_map_, 10.0);
    private_nh_.param("height_map", height_map_, 10.0);
    private_nh_.param("sx", sx_, 2);
    private_nh_.param("sy", sy_, 2);
    private_nh_.param("footprint_padding_X", footprint_padding_X_, (float)0.1);
    private_nh_.param("footprint_padding_Y", footprint_padding_Y_, (float)0.0);
    private_nh_.param("eps_x", eps_x_, 5e-2);
    private_nh_.param("eps_y", eps_y_, 5e-2);
    private_nh_.param("eps_theta", eps_theta_, 1e-2);

    if (loadFootprintFromParam("unpadded_footprint", unpadded_footprint_)) {
        ROS_INFO("Successfully loaded unpadded footprint.");
        padded_footprint_ = unpadded_footprint_;
        if(footprint_padding_X_ > 0)
            padFootprintX(padded_footprint_, footprint_padding_X_);
        if(footprint_padding_Y_ > 0)
            padFootprintY(padded_footprint_, footprint_padding_Y_);
    } else {
        ROS_WARN("Failed to load unpadded footprint. Using default.");
    }

    ROS_INFO("Circle raidus of disc: %.3f [m], min_obstacle_distance: %.3f [m]", cir_radius_, min_obstacle_distance_);

    X = SX::sym("X", num_states_, N_horizon_+1);
    U = SX::sym("U", num_controls_, N_horizon_);
    P = SX::sym("P", num_states_ * (N_horizon_ + 1) + num_controls_ * N_horizon_ + 3 * N_maxobs_);
    
    obj_ = setObjective();
    
    cst_ = setStateConstraints();
    
    if(avoidance_enable_)
        setCollisionConstraints(cst_);

    if(keep_lane_)
        setKeepLaneConstraints(cst_);

    SX x_ = SX::vertcat({SX::reshape(X, -1, 1), SX::reshape(U, -1, 1)});
    nlp_["x"] = x_;
    nlp_["f"] = obj_;
    nlp_["p"] = P;
    nlp_["g"] = cst_;

    opts_ = casadi::Dict();
    opts_["print_time"] = 0;                                
    opts_["ipopt.print_level"] = 0;                            
    opts_["ipopt.acceptable_tol"] = 1e-8;                       
    opts_["ipopt.max_iter"] = 100;                                                   
    opts_["ipopt.acceptable_obj_change_tol"] = 1e-6;            

    ref_traj_     = new Trajectory(nh_);
    global_frame_ = ref_traj_->getGlobalFrame();
    base_frame_   = ref_traj_->getBaseFrame();

    laser_scan_sub_ = boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>(
                        new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan1", 50));

    filter_ = boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(
                new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, tf_, base_frame_, 50, nh_));

    filter_->registerCallback(boost::bind(&MotionController::laserScanCallback, this, _1));
}

SX MotionController::setObjective() {
    SX x = X(Slice(), N_horizon_);
    SX x_ref = P(Slice(num_states_ * N_horizon_, num_states_ * (N_horizon_ + 1)));
    SX obj = SX::mtimes(SX::mtimes(diffStateVector(x, x_ref).T(), Q_N_), diffStateVector(x, x_ref));
    for(int k = 0; k < N_horizon_; k++) {
        x = X(Slice(), k);
        x_ref = P(Slice(num_states_ * k, num_states_ * (k + 1)));
        obj = obj + SX::mtimes(SX::mtimes(diffStateVector(x, x_ref).T(), Q_), diffStateVector(x, x_ref));
    }

    int start = num_states_ * (N_horizon_ + 1);
    int end = start + num_controls_;
    for(int k = 0; k < N_controls_; k++) {
        SX u = U(Slice(), k);
        SX u_next = U(Slice(), (k+1) % N_controls_);
        SX u_ref = P(Slice(start, end));

        obj = obj + SX::mtimes(SX::mtimes((u - u_ref).T(), R_), (u - u_ref))
            + SX::mtimes(SX::mtimes((u_next - u).T(), W_), (u_next - u));
        
        start = end;
        end += num_controls_;
    }
    return obj;
}

SX MotionController::setStateConstraints() {
    SX cst = X(Slice(), 0) - P(Slice(0, num_states_));
    for(int k = 0; k < N_horizon_; k++) {
        SX x = X(Slice(), k);
        SX x_next = X(Slice(), k+1);
        SX u = U(Slice(), k);

        SX anpha = cos(x(2, 0)) + 4 * cos(x(2, 0) + 0.5*T_s_*u(1, 0)) + cos(x(2, 0) + T_s_*u(1, 0));
        SX beta  = sin(x(2, 0)) + 4 * sin(x(2, 0) + 0.5*T_s_*u(1, 0)) + sin(x(2, 0) + T_s_*u(1, 0));
    
        SX H = SX::vertcat({SX::horzcat({anpha / 6, 0}),
                            SX::horzcat({beta / 6 , 0}),
                            SX::horzcat({0,         1})});
                            
        SX x_next_RK4 = x + T_s_ * SX::mtimes(H, u);
        cst = vertcat(cst, diffStateVector(x_next, x_next_RK4));
    }
    return cst;
}

void MotionController::setCollisionConstraints(SX& cst) {
    std::vector<geometry_msgs::Point> decomposed_footprint = deCompositionFootprint(unpadded_footprint_, sx_, sy_);
    SX oriented_footprint = SX::zeros(2, decomposed_footprint.size());
    int numXU = num_states_ * (N_horizon_ + 1) + num_controls_ * N_horizon_;
    int s = 3;
    for(int i = 0; i < N_horizon_ + 1; i++) {
        SX x = X(Slice(), i);
        transformFootprint(x, oriented_footprint, decomposed_footprint);
        for(int n = 0; n < oriented_footprint.size2(); n++) {
            for(int j = 0; j < N_maxobs_; j++) {
                SX polygon = P(Slice(numXU + (j) * s, numXU + (j + 1) * s));
                SX u = oriented_footprint(Slice(), n) - polygon(Slice(0,2,1));
                SX d_k = SX::dot(u, u) - (polygon(2) + min_obstacle_distance_ + cir_radius_) * (polygon(2) + min_obstacle_distance_ + cir_radius_);
                cst = vertcat(cst, d_k);
            }  
        }
    }
}

void MotionController::setKeepLaneConstraints(casadi::SX& cst) {
    for(int k = 1; k < N_horizon_; k++) {
        SX x = X(Slice(), k);
        SX x_ref = P(Slice(num_states_ * k, num_states_ * (k + 1)));
        SX d = (x(0) - x_ref(0)) * sin(x_ref(2)) - (x(1) - x_ref(1)) * cos(x_ref(2));
        cst = vertcat(cst, width_lane_ - fabs(d));
    }
}

void MotionController::obstacleCallback(const convert_polygon::ObstacleArrayMsg& msg) {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    if(!avoidance_enable_)
        return;
    obstacles_.clear();
    Polygon p;
    for(auto& obs : msg.obstacles) {
        if(obs.polygon.points.size() < 3)
            continue;
        p.points.assign(obs.polygon.points.begin(), obs.polygon.points.begin() + obs.polygon.points.size() - 1);
        p.centroid = obs.centroid;
        p.radius   = obs.radius;
        obstacles_.push_back(p);
    }
    updated_obstacles_ = true;
}

void MotionController::filterObstacles(double (&robot_pose)[3]) {
    int num_obstacles = obstacles_.size(); 
    if(num_obstacles < N_maxobs_ && num_obstacles > 0) {
        Polygon last_polygon = obstacles_.back();
        obstacles_.insert(obstacles_.end(), N_maxobs_ - num_obstacles, last_polygon);

    }else if (num_obstacles > N_maxobs_) {
        geometry_msgs::Point temp;
        temp.x = robot_pose[0]; temp.y = robot_pose[1];
        std::nth_element(obstacles_.begin(), obstacles_.begin() + N_maxobs_, obstacles_.end(),
                        [&](const Polygon& p1, const Polygon& p2) {
            return norm2d(p1.centroid, temp) < norm2d(p2.centroid, temp); });
        obstacles_.resize(N_maxobs_);
    }
}

bool MotionController::getRobotPose(double (&robot_pose_)[3]) {
    geometry_msgs::PoseStamped global_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);

    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = base_frame_;
    robot_pose.header.stamp = ros::Time();
    ros::Time current_time = ros::Time::now();

    try {
        if(tf_.canTransform(global_frame_, base_frame_, current_time)) {
            geometry_msgs::TransformStamped transform = tf_.lookupTransform(global_frame_, base_frame_, current_time);
            tf2::doTransform(robot_pose, global_pose, transform);
        }else {
            tf_.transform(robot_pose, global_pose, global_frame_);
        }
    }catch (tf2::LookupException& ex) {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }catch (tf2::ConnectivityException& ex) {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }catch (tf2::ExtrapolationException& ex) {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    
    if (!global_pose.header.stamp.isZero() && current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_) {
        ROS_WARN_THROTTLE(1.0,
                      "Transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
        return false;
    }
    robot_pose_[0] = global_pose.pose.position.x;
    robot_pose_[1] = global_pose.pose.position.y;
    robot_pose_[2] = tf2::getYaw(global_pose.pose.orientation);
    return true;
}

bool MotionController::checkIsGoal(double (&robot_pose)[3]) {
    geometry_msgs::Pose goal = ref_traj_->getGlobalGoal();
    double delta_angle;
    double d1 = robot_pose[2] - tf::getYaw(goal.orientation);
    double d2 = 2 * M_PI - fabs(d1);
    if(d1 > 0)
        d2 *= -1;
    if(fabs(d1) < fabs(d2))
        delta_angle = d1;
    else
        delta_angle = d2;
    
    if (fabs(robot_pose[0] - goal.position.x) < eps_x_ &&
        fabs(robot_pose[1] - goal.position.y) < eps_y_ && fabs(delta_angle) < eps_theta_) {

        geometry_msgs::Twist ctrl_vel;
        ctrl_vel.linear.x  = 0.0;
        ctrl_vel.angular.z = 0.0;
        vel_pub_.publish(ctrl_vel);
        ROS_INFO("Robot has successfully reached the goal position at (x: %.2f, y: %.2f) with heading: %.2f radians.",
                robot_pose[0], robot_pose[1], robot_pose[2]);
        return true;
    };
    return false;
}

bool MotionController::loadFootprintFromParam(const std::string& param_name, std::vector<geometry_msgs::Point>& footprint) {
    XmlRpc::XmlRpcValue footprint_list;
    if(!private_nh_.getParam(param_name, footprint_list) || footprint_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Parameter %s is not a valid list!", param_name.c_str());
        return false;
    }
    try {
        for (int i = 0; i < footprint_list.size(); ++i) {
            if (footprint_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
                !footprint_list[i].hasMember("x") || !footprint_list[i].hasMember("y")) {
                    ROS_ERROR("Invalid footprint element at index %d!", i);
                    return false;
            }
            geometry_msgs::Point pt;
            pt.x = static_cast<double>(footprint_list[i]["x"]);
            pt.y = static_cast<double>(footprint_list[i]["y"]);
            pt.z = 0.0;
            footprint.push_back(pt);
        }
    }catch (const XmlRpc::XmlRpcException& ex) {
        ROS_ERROR("Error parsing footprint: %s", ex.getMessage().c_str());
        return false;
    }
    return true;
}

std::vector<geometry_msgs::Point> MotionController::deCompositionFootprint(std::vector<geometry_msgs::Point>& footprint, int sx, int sy) {
    double xmin, ymin, xmax, ymax;
    xmin = std::numeric_limits<double>::max();
    ymin = std::numeric_limits<double>::max();
    xmax = -xmin; ymax = -ymin;
    for(int i = 0; i < (int)footprint.size(); i++) {
        if(footprint[i].x < xmin)
            xmin = footprint[i].x;
        if(footprint[i].x > xmax)
            xmax = footprint[i].x;
        
        if(footprint[i].y < ymin)
            ymin = footprint[i].y;
        if(footprint[i].y > ymax)
            ymax = footprint[i].y;
    }
    
    std::vector<geometry_msgs::Point> decomposed_footprint;
    double scale_x = (xmax - xmin) / sx;
    double scale_y = (ymax - ymin) / sy;
    geometry_msgs::Point pt;
    for(int i = 0; i < sx; i++) {
        for(int j = 0; j < sy; j++) {
            pt.x = xmin + i * scale_x + scale_x / 2;
            pt.y = ymin + j * scale_y + scale_y / 2;
            pt.z = 0;
            decomposed_footprint.push_back(pt);
        }
    }
    cir_radius_ = sqrt(scale_x * scale_x + scale_y * scale_y) / 2;
    ROS_INFO("Footprint decomposed with size (sx: %d, sy: %d); resulting safety margins: deltax = %.3f m, deltay = %.3f m.",
            sx, sy, cir_radius_ - 0.5 * scale_x, cir_radius_ - 0.5 * scale_y);
    return decomposed_footprint;
}

void MotionController::transformFootprint(const casadi::SX& x, casadi::SX& oriented_footprint,
                                          const std::vector<geometry_msgs::Point>& footprint) {
    SX cos_th = cos(x(2));
    SX sin_th = sin(x(2));
    for(int i = 0; i < (int)footprint.size(); i++) {
        oriented_footprint(0, i) = x(0) + (footprint[i].x * cos_th - footprint[i].y * sin_th);
        oriented_footprint(1, i) = x(1) + (footprint[i].x * sin_th + footprint[i].y * cos_th);
    }
}

void MotionController::padFootprintX(std::vector<geometry_msgs::Point>& footprint, double padding) {
    for (unsigned int i = 0; i < footprint.size(); i++) {
        geometry_msgs::Point& pt = footprint[i];
        pt.x += sign0(pt.x) * padding;
    }
}

void MotionController::padFootprintY(std::vector<geometry_msgs::Point>& footprint, double padding) {
    for (unsigned int i = 0; i < footprint.size(); i++) {
        geometry_msgs::Point& pt = footprint[i];
        pt.y += sign0(pt.y) * padding;
    }
}

void MotionController::laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg) {
    sensor_msgs::PointCloud2 cloud;
    cloud.header = msg->header; 
    try{
        projector_.transformLaserScanToPointCloud(msg->header.frame_id, *msg, cloud, tf_);
    }catch (tf2::TransformException &ex) {
        ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", base_frame_.c_str(),
             ex.what());
        projector_.projectLaser(*msg, cloud);
    }

    sensor_msgs::PointCloud2 base_frame_cloud;
    tf_.transform(cloud, base_frame_cloud, base_frame_);
    base_frame_cloud.header.frame_id = base_frame_;
    base_frame_cloud.header.stamp = cloud.header.stamp;

    double xmin, ymin, xmax, ymax;
    xmin = std::numeric_limits<double>::max();
    ymin = std::numeric_limits<double>::max();
    xmax = -xmin; ymax = -ymin;
    for(int i = 0; i < (int)unpadded_footprint_.size(); i++) {
        if(unpadded_footprint_[i].x < xmin)
            xmin = unpadded_footprint_[i].x;
        if(unpadded_footprint_[i].x > xmax)
            xmax = unpadded_footprint_[i].x;
        
        if(unpadded_footprint_[i].y < ymin)
            ymin = unpadded_footprint_[i].y;
        if(unpadded_footprint_[i].y > ymax)
            ymax = unpadded_footprint_[i].y;
    }

    double x_upper_bound, y_upper_bound, x_lower_bound, y_lower_bound;
    x_upper_bound = xmax + footprint_padding_X_;
    x_lower_bound = xmax;
    y_upper_bound = ymax + footprint_padding_Y_;
    y_lower_bound = ymin - footprint_padding_Y_;

    has_collision_ = false;
    for (sensor_msgs::PointCloud2Iterator<float> iter_x(base_frame_cloud, "x"),
                                                 iter_y(base_frame_cloud, "y");
         iter_x != iter_x.end();
         ++iter_x, ++iter_y)
    {
        if(*iter_x < x_upper_bound && *iter_x > x_lower_bound && 
           *iter_y < y_upper_bound && *iter_y > y_lower_bound) {
            has_collision_ = true;
            geometry_msgs::Twist ctrl_vel;
            ctrl_vel.linear.x  = 0.0;
            ctrl_vel.angular.z = 0.0;
            vel_pub_.publish(ctrl_vel);
            ROS_WARN("[Collision Warning] Potential obstacle detected ahead. Immediate attention required.");
            return;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_motion_controller");
    ros::NodeHandle nh;
    while (!ros::Time::now().toSec()) {
        ROS_WARN("Waiting for ROS Time to be initialized...");
        ros::Duration(0.1).sleep();
    }
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    MotionController ctrl(buffer);
    ros::waitForShutdown();
    return 0;
}