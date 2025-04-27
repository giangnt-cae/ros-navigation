#include <vk_motion_controller/motion_controller.hpp>
MotionController::MotionController(tf2_ros::Buffer& tf)
    : Q_(DM::diagcat({5, 5, 2})),
    R_(DM::diagcat({0.1, 0.1})),
    W_(DM::diagcat({0.5, 0.5})),
    Q_N_(DM::diagcat({10, 10, 2})),
    tf_(tf),
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
    DM p    = DM::zeros(numP, 1);
    DM x0_  = DM::zeros(numX, 1);
    DM u0_  = DM::zeros(numU, 1);

    casadi::Function solver = nlpsol("solver", "ipopt", nlp_, opts_);
    ros::Time current_time = ros::Time::now(), last_time = ros::Time::now();

    double robot_pose[3];
    double x_ref[5];        // [x, y, theta, v, w]
    double dt, t = 0;
    ros::Rate rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        current_time = ros::Time::now();
        if(ref_traj_->getUpdated() && getRobotPose(robot_pose)) {    
            dt = (current_time - last_time).toSec();

            p(Slice(0, num_states_)) = {robot_pose[0], robot_pose[1], robot_pose[2]};
            
            for(int i = 0; i < N_horizon_; i++) {
                ref_traj_->getPoseAndVelocity(x_ref, t + (i + 1) * T_s_);
                p(Slice(num_states_ * (i + 1), num_states_ * (i + 2))) = {x_ref[0], x_ref[1], x_ref[2]};
                p(Slice(numX + num_controls_ * i, numX + num_controls_ * (i + 1))) = {x_ref[3], x_ref[4]};
            }

            DM x_first = p(Slice(0, num_states_)), x_end = p(Slice(num_states_, 2 * num_states_));
            DM delta = diffStateVector(x_first, x_end);
            if((double)norm_2(delta(Slice(0, 2))) < max_error_position_)
                t = t + dt;

            for(auto& polygon : obstacles_) {
                polygon.filter(N_maxvertices_, robot_pose);
            }
    
            filterObstacles(robot_pose);

            int n = numX + numU - 1;
            for(auto& polygon : obstacles_) {
                for(auto& vertice : polygon.points) {
                    p(++n) = vertice.x;
                    p(++n) = vertice.y;
                }
                p(++n) = polygon.centroid.x;
                p(++n) = polygon.centroid.y;
            }
            
            /*----------------------------------IPOPT solver -------------------------------------*/
            args_["p"]   = p;
            args_["x0" ] = DM::vertcat({reshape(x0_, numX, 1),
                                        reshape(u0_, numU, 1)});
            
            casadi::DMDict res = solver(DMDict{{"x0",  args_["x0"]},
                                               {"lbx", args_["lbx"]},
                                               {"ubx", args_["ubx"]},
                                               {"lbg", obstacles_.empty() ? -casadi::inf : args_["lbg"]},
                                               {"ubg", obstacles_.empty() ?  casadi::inf : args_["ubg"]},
                                               {"p",   args_["p"]}});
            
            // std::map<std::string, casadi::GenericType> solver_stats = solver.stats();
            // for (const auto& kv : solver_stats) {
            //     if (kv.first.find("t_wall_") == 0 || kv.first.find("t_proc_") == 0) {
            //         ROS_INFO("%s: %f seconds", kv.first.c_str(), kv.second.as_double());
            //     }
            // }

            DM x_opt = (DM)res["x"];
            x0_      = x_opt(Slice(0, numX));
            u0_      = x_opt(Slice(numX, numX + numU));
            
            // ROS_INFO_STREAM("x0:  " << args_["x0"]);
            // ROS_INFO_STREAM("lbx: " << args_["lbx"]);
            // ROS_INFO_STREAM("ubx: " << args_["ubx"]);
            // ROS_INFO_STREAM("lbg: " << args_["lbg"]);
            // ROS_INFO_STREAM("ubg: " << args_["ubg"]);
            // ROS_INFO_STREAM("p:   " << args_["p"]);
            /*----------------------------------------------------------------------------------*/
            
            // Velocity publish
            geometry_msgs::Twist ctrl_vel;
            ctrl_vel.linear.x =  (double)u0_(0);
            ctrl_vel.angular.z = (double)u0_(1);
            ROS_INFO("Velocity command: v = %.3f m/s; w = %.3f rad/s",
                                        ctrl_vel.linear.x, ctrl_vel.angular.z = (double)u0_(1));
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
    private_nh_.param("max_vertices", N_maxvertices_, 3);
    private_nh_.param("T_sample", T_s_, 0.5);
    private_nh_.param("min_obstacle_distance", min_obstacle_distance_, 0.05);
    private_nh_.param("cir_radius", cir_radius_, 0.25);
    private_nh_.param("max_error_position", max_error_position_, 2.0);
    private_nh_.param("max_error_angle", max_error_angle_, M_PI / 6);
    private_nh_.param("avoidance_enable", avoidance_enable_, true);
    private_nh_.param("keep_lane", keep_lane_, true);
    private_nh_.param("width_lane", width_lane_, 1.0);

    if (loadFootprintFromParam("unpadded_footprint", unpadded_footprint_)) {
        ROS_INFO("Successfully loaded unpadded footprint.");
    } else {
        ROS_WARN("Failed to load unpadded footprint. Using default.");
    }

    ROS_INFO("Circle raidus of disc: %.3f [m], min_obstacle_distance: %.3f [m]", cir_radius_, min_obstacle_distance_);

    X = SX::sym("X", num_states_, N_horizon_+1);
    U = SX::sym("U", num_controls_, N_horizon_);
    P = SX::sym("P", num_states_ * (N_horizon_ + 1) + num_controls_ * N_horizon_ + 2 * N_maxobs_ * (N_maxvertices_ + 1));
    
    obj_ = setObjective();
    
    cst_ = setStateConstraints();

    if(avoidance_enable_)
        setCollisionConstraints(cst_);
    
    if(keep_lane_)
        setKeepLaneConstraints(cst_);

    // IPOPT solver
    SX x_ = SX::vertcat({SX::reshape(X, -1, 1), SX::reshape(U, -1, 1)});
    nlp_["x"] = x_;
    nlp_["f"] = obj_;
    nlp_["p"] = P;
    nlp_["g"] = cst_;

    opts_ = casadi::Dict();
    opts_["print_time"] = 0;
    opts_["ipopt.print_level"] = 0;
    opts_["ipopt.acceptable_tol"] = 1e-8;
    opts_["ipopt.max_iter"] = 500;
    opts_["ipopt.acceptable_obj_change_tol"] = 1e-6;

    // opts_["ipopt.mu_strategy"] = "adaptive";
    // opts_["ipopt.mu_init"] = 1e-2;
    // opts_["ipopt.warm_start_init_point"] = "yes";
    // opts_["ipopt.hessian_approximation"] = "limited-memory";
    // opts_["ipopt.linear_solver"] = "mumps";

    // opts_["ipopt.constr_viol_tol"] = 1e-6;
    // opts_["ipopt.bound_relax_factor"] = 0.0;
    // casadi::Dict jit_opts;
    // jit_opts["compiler"] = "clang++";
    // jit_opts["flags"] = "-O3";
    // opts_["jit_options"] = jit_opts;

    ref_traj_     = new Trajectory(nh_);
    global_frame_ = ref_traj_->getGlobalFrame();
    base_frame_   = ref_traj_->getBaseFrame();
    double v_maxtrans = ref_traj_->getMaxVelocityLinear();
    double v_maxrot   = ref_traj_->getMaxvelocityAngular();

    int numX = X.numel(), numU = U.numel();

    casadi::DM lbx = DM::zeros(numX + numU, 1);
    lbx(Slice(0, numX, num_states_)) = -casadi::inf;
    lbx(Slice(1, numX, num_states_)) = -casadi::inf;
    lbx(Slice(2, numX, num_states_)) = -casadi::inf;

    lbx(Slice(numX, numX + numU, num_controls_))     = -v_maxtrans;
    lbx(Slice(numX + 1, numX + numU, num_controls_)) = -v_maxrot;

    casadi::DM ubx = DM::zeros(numX + numU, 1);
    ubx(Slice(0, numX, num_states_)) = casadi::inf;
    ubx(Slice(1, numX, num_states_)) = casadi::inf;
    ubx(Slice(2, numX, num_states_)) = casadi::inf;

    ubx(Slice(numX, numX + numU, num_controls_))     = v_maxtrans;
    ubx(Slice(numX + 1, numX + numU, num_controls_)) = v_maxrot;

    args_["lbx"] = lbx;
    args_["ubx"] = ubx;

    DM lbg = DM::zeros(cst_.numel(), 1);
    DM ubg = DM::zeros(cst_.numel(), 1);
    ubg(Slice(numX, cst_.numel())) = casadi::inf;

    args_["lbg"] = lbg;
    args_["ubg"] = ubg;
}

SX MotionController::setObjective() {
    // State errors
    SX x = X(Slice(), N_horizon_);
    SX x_ref = P(Slice(num_states_ * N_horizon_, num_states_ * (N_horizon_ + 1)));
    SX obj = SX::mtimes(SX::mtimes(diffStateVector(x, x_ref).T(), Q_N_), diffStateVector(x, x_ref));
    for(int k = 0; k < N_horizon_; k++) {
        x = X(Slice(), k);
        x_ref = P(Slice(num_states_ * k, num_states_ * (k + 1)));
        obj = obj + SX::mtimes(SX::mtimes(diffStateVector(x, x_ref).T(), Q_), diffStateVector(x, x_ref));
    }

    // Control errors
    int start = num_states_ * (N_horizon_ + 1);
    int end = start + num_controls_;
    for(int k = 0; k < N_controls_; k++) {
        SX u = U(Slice(), k);
        SX u_next = U(Slice(), (k+1) % N_controls_);
        SX u_ref = P(Slice(start, end));

        obj = obj + SX::mtimes(SX::mtimes((u - u_ref).T(), R_), (u - u_ref))
            + SX::mtimes(SX::mtimes((u_next - u).T(), W_), (u_next - u));
        
        // obj = obj + SX::mtimes(SX::mtimes((u_next - u).T(), W_), (u_next - u));
        
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

SX closestDistance(const SX& P1, const SX& P2, const SX& Q) {
    SX u = P2 - P1;
    SX u_dot = SX::dot(u, u);

    SX t = SX::dot(u, (Q - P1)) / (u_dot + 1e-6);
    t = if_else(t < 0.0, SX(0.0), if_else(t > 1.0, SX(1.0), t));
    SX closestPoint1 = P1 + t * u;
    SX closestPoint2 = Q;

    SX delta = closestPoint1 - closestPoint2;
    return sqrt(SX::dot(delta, delta));
}

void MotionController::setCollisionConstraints(SX& cst) {
    SX oriented_footprint = SX::zeros(2, unpadded_footprint_.size());
    int numXU = num_states_ * (N_horizon_ + 1) + num_controls_ * N_horizon_;
    int s =  2 * (N_maxvertices_ + 1);
    for(int i = 0; i < N_horizon_ + 1; i++) {
        SX x = X(Slice(), i);
        transformFootprint(x, oriented_footprint, unpadded_footprint_);
        for(int n = 0; n < oriented_footprint.size2(); n++) {
            for(int j = 0; j < N_maxobs_; j++) {
                SX polygon = reshape(P(Slice(numXU + (j) * s, numXU + (j + 1) * s)), 2, (N_maxvertices_ + 1));
                double lamda = 50;
                SX sum_exp = 0;
                for (int k = 0; k < N_maxvertices_; k++) {
                    SX d_k = closestDistance(polygon(Slice(), k), polygon(Slice(), (k+1) % N_maxvertices_), oriented_footprint(Slice(), n));
                    sum_exp += exp(-lamda * d_k);
                }
                cst = vertcat(cst, (-log(sum_exp) / lamda) - min_obstacle_distance_ - cir_radius_);
            }  
        }
    }
}

/* LinearConstraints
SX computeLinearConstraints(const SX& poly) {
    int num_vertices = poly.size2() - 1;
    SX constraints = SX::zeros(3, num_vertices);   // a, b, c

    SX vertices    = poly(Slice(), Slice(0, num_vertices));

    SX centroid    = SX::zeros(2, 1);
    centroid(0, 0) = sum2(vertices(0, Slice())).scalar() / num_vertices;
    centroid(1, 0) = sum2(vertices(1, Slice())).scalar() / num_vertices;
    for(int i = 0; i < num_vertices; i++) {
        SX p1 = vertices(Slice(), i);
        SX p2 = vertices(Slice(), (i + 1) % num_vertices);

        // Normal vector (a, b)
        SX a = p2(1, 0) - p1(1, 0);
        SX b = p1(0, 0) - p2(0, 0);
        SX norm = sqrt(a * a + b * b);
        a = a / norm;
        b = b / norm;
        SX c = - (a * p1(0, 0) + b * p1(1, 0));
        SX chk = a * centroid(0, 0) + b * centroid(1, 0) + c;
        a = if_else(chk < SX(0.0), -a, a);
        b = if_else(chk < SX(0.0), -b, b);
        c = if_else(chk < SX(0.0), -c, c);
        constraints(Slice(), i) = vertcat(a, b, c);
    }
    return constraints;
}

void MotionController::setCollisionConstraints(casadi::SX& cst) {
    SX oriented_footprint = SX::zeros(2, unpadded_footprint_.size());
    for(int i = 0; i < N_horizon_; i++) {
        SX x = X(Slice(), i);
        transformFootprint(x, oriented_footprint, unpadded_footprint_);

        int start = num_states_ * (N_horizon_ + 1) + num_controls_ * N_horizon_;
        int end = start + 2 * (N_maxvertices_ + 1);
        for(int j = 0; j < N_maxobs_; j++) {
            SX polygon = reshape(P(Slice(start, end)), 2, (N_maxvertices_ + 1));
            SX abc = computeLinearConstraints(polygon);

            SX d_min = abc(0, 0) * x(0, 0) + abc(1, 0) * x(1, 0) + abc(2, 0);
            for (int k = 1; k < N_maxvertices_; k++) {
                SX d_k = abc(0, k) * x(0, 0) + abc(1, k) * x(1, 0) + abc(2, k);
                d_min = fmin(d_min, d_k);
            }
            cst = vertcat(cst, -d_min);
            start = end;
            end += 2 * (N_maxvertices_ + 1);
        }
    }
}
*/

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

    if (!tf_.canTransform(global_frame_, base_frame_, current_time, ros::Duration(0.1))) {
        ROS_WARN("TF transform not available from %s to %s", base_frame_.c_str(), global_frame_.c_str());
        return false;
    }

    try {
        geometry_msgs::TransformStamped transform = tf_.lookupTransform(global_frame_, base_frame_, current_time);
        tf2::doTransform(robot_pose, global_pose, transform);
    }catch (tf2::TransformException& ex) {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    robot_pose_[0] = global_pose.pose.position.x;
    robot_pose_[1] = global_pose.pose.position.y;
    robot_pose_[2] = tf2::getYaw(global_pose.pose.orientation);
    return true;
}

bool MotionController::loadFootprintFromParam(const std::string& param_name, std::vector<geometry_msgs::Point>& footprint) {
    XmlRpc::XmlRpcValue footprint_list;
    // Check if the parameter exists and is a list
    if(!nh_.getParam(param_name, footprint_list) || footprint_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
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
        deCompositionFootprint(footprint);
    }catch (const XmlRpc::XmlRpcException& ex) {
        ROS_ERROR("Error parsing footprint: %s", ex.getMessage().c_str());
        return false;
    }
    return true;
}

void MotionController::deCompositionFootprint(std::vector<geometry_msgs::Point>& footprint, int sx, int sy) {
    double xmin, ymin, xmax, ymax;
    xmin = std::numeric_limits<double>::max();
    ymin = std::numeric_limits<double>::max();
    xmax = -xmin; ymax = -ymin;
    for(int i = 0; i < (int)footprint.size(); i++) {
        if(footprint[i].x < xmin)
            xmin = footprint[i].x;
        else if(footprint[i].x > xmax)
            xmax = footprint[i].x;
        
        if(footprint[i].y < ymin)
            ymin = footprint[i].y;
        else if(footprint[i].y > ymax)
            ymax = footprint[i].y;
        else {
            ROS_WARN("Failed decomposition for footprint with sx x sy: %d x %d", sx, sy);
            return;
        }
    }

    footprint.clear();
    double scale_x = (xmax - xmin) / sx;
    double scale_y = (ymax - ymin) / sy;
    geometry_msgs::Point pt;
    for(int i = 0; i < sx; i++) {
        for(int j = 0; j < sy; j++) {
            pt.x = xmin + i * scale_x + scale_x / 2;
            pt.y = ymin + j * scale_y + scale_y / 2;
            pt.z = 0;
            footprint.push_back(pt);
        }
    }
    cir_radius_ = sqrt(scale_x * scale_x + scale_y * scale_y) / 2;
}

void MotionController::transformFootprint(const casadi::SX& x, casadi::SX& oriented_footprint,
                                       const std::vector<geometry_msgs::Point>& footprint_spec) {
    SX cos_th = cos(x(2));
    SX sin_th = sin(x(2));
    for(int i = 0; i < (int)footprint_spec.size(); i++) {
        oriented_footprint(0, i) = x(0) + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
        oriented_footprint(1, i) = x(1) + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
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