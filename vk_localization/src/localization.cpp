#include "vk_localization/localization.hpp"

void LocalizationNode::savePoseToServer() {
    // We need to apply the last transform to latest odom pose get the latest map pose to store.
    tf2::Transform odom_pose_tf2;
    tf2::convert(latest_odom_pose_.pose, odom_pose_tf2);
    tf2::Transform map_pose = latest_tf_.inverse() * odom_pose_tf2;

    double yaw = tf2::getYaw(map_pose.getRotation());
    private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
    private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
    private_nh_.setParam("initial_pose_a", yaw);
}

void LocalizationNode::updatePoseFromServer() {
    init_pose_[0] = 0.0;
    init_pose_[1] = 0.0;
    init_pose_[2] = 0.0;

    double tmp_pose;
    private_nh_.param("initial_pose_x", tmp_pose, init_pose_[0]);
    if(!std::isnan(tmp_pose)) { init_pose_[0] = tmp_pose; }
    else { ROS_WARN("Ignoring NAN in initial pose X position"); }

    private_nh_.param("initial_pose_y", tmp_pose, init_pose_[1]);
    if(!std::isnan(tmp_pose)) { init_pose_[1] = tmp_pose; }
    else { ROS_WARN("Ignoring NAN in initial pose Y position"); }

    private_nh_.param("initial_pose_a", tmp_pose, init_pose_[2]);
    if(!std::isnan(tmp_pose)) { init_pose_[2] = tmp_pose; }
    else { ROS_WARN("Ignoring NAN in initial pose yaw angle"); }
}

void LocalizationNode::checkLaserReceived(const ros::TimerEvent& event) {
    ros::Duration d = ros::Time::now() - last_laser_received_ts_;
    if(d > laser_check_interval_) {
        ROS_WARN("No laser scan received for %f seconds. Verify that data is being published on the %s topic.",
                  d.toSec(),
                  ros::names::resolve(scan_topic).c_str());
    }
}

void LocalizationNode::requestMap() {
    boost::recursive_mutex::scoped_lock ml(configuration_mutex_);
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;
    ROS_INFO("Requesting the map...");
    while(!ros::service::call("static_map", req, res)) {
        ROS_WARN("Request for map failed; trying again...");
        ros::Duration d(0.5);
        d.sleep();
    }
    handleMapMessage(res.map);
}

void LocalizationNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg) {
    if(first_map_only_ && first_map_received_) { return; }
    handleMapMessage(*msg);
    first_map_received_ = true;
}

void LocalizationNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg) {
    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
    ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
            msg.info.width,
            msg.info.height,
            msg.info.resolution);
    if(msg.header.frame_id != map_frame_id_) {
        ROS_WARN("Frame_id of map received: '%s' doesn't match map_frame_id: '%s' ",
                msg.header.frame_id.c_str(),
                map_frame_id_.c_str());
    }
    if(map_ != NULL) {
        map_delete(map_);
        map_ = NULL;
    }
    
    lasers_.clear();
    lasers_update_.clear();
    frame_to_laser_.clear();
    
    map_ = convertMap(msg);
    updatePoseFromServer();
    robot_pose = {init_pose_[0], init_pose_[1], init_pose_[2]};
    f_init_ = false;
    
    delete odom_;
    odom_ = new Odom();
    odom_->model_type = odom_model_type_;
    
    delete laser_;
    laser_ = new Laser(max_beams_);
    
    ROS_INFO("Initializing nearest distance map; this can take some time on large maps...");
    map_update_cspace(map_, initial_threshold_);
    ROS_INFO("Map is ready.");
    
    sigma = initial_threshold_;
    sm_ = new ScanMatcher();
    adp_thresh_ = new AdaptiveThreshold(initial_threshold_, min_motion_, laser_max_range_);
}

map_t* LocalizationNode::convertMap(const nav_msgs::OccupancyGrid& msg) {
    map_t* map = map_alloc();
    map->size_x = msg.info.width;
    map->size_y = msg.info.height;
    map->scale = msg.info.resolution;
    map->origin_x = msg.info.origin.position.x + (map->size_x / 2) * map->scale;
    map->origin_y = msg.info.origin.position.y + (map->size_y / 2) * map->scale;
    map->cells = new map_cell_t[map->size_x * map->size_y];
    for(int i = 0; i < map->size_x * map->size_y; i++) {
        if(msg.data[i] == 0) { map->cells[i].occ_state = -1; }
        else if(msg.data[i] == 100) { map->cells[i].occ_state = 1; }
        else { map->cells[i].occ_state = 0; }
    }
    return map;
}

bool LocalizationNode::getOdomPose(geometry_msgs::PoseStamped& odom_pose,
                                   double& x, double& y, double& yaw,
                                   const ros::Time& t, const std::string& f) {
    geometry_msgs::PoseStamped ident;
    ident.header.frame_id = stripSlash(f);
    ident.header.stamp = t;
    tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
    try {
        this->tf_->transform(ident, odom_pose, odom_frame_id_);
    } catch (const tf2::TransformException& e) {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }                                
    x = odom_pose.pose.position.x;
    y = odom_pose.pose.position.y;
    yaw = tf2::getYaw(odom_pose.pose.orientation);
    return true;
}

void LocalizationNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan) {
    std::string laser_scan_frame_id = stripSlash(laser_scan->header.frame_id);
    last_laser_received_ts_ = ros::Time::now();
    if(map_ == NULL) {
        return;
    }

    boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
    int laser_index = -1;
    
    // Do we have the base_link -> base_laser Tx yet?
    if(frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
        ROS_INFO("Setting up laser %d (frame_id = %s)\n", (int)frame_to_laser_.size(),
                                                            laser_scan_frame_id.c_str());
        lasers_.push_back(new Laser(*laser_));
        lasers_update_.push_back(true);
        laser_index = frame_to_laser_.size();

        geometry_msgs::PoseStamped ident;
        ident.header.frame_id = laser_scan_frame_id;
        ident.header.stamp = ros::Time();
        tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

        geometry_msgs::PoseStamped laser_pose;
        try {
            this->tf_->transform(ident, laser_pose, base_frame_id_);
        } catch (const tf2::TransformException& e) {
            ROS_ERROR("Couldn't transform from %s to %s even though the message notifier is in use", laser_scan_frame_id.c_str(),
                                                                                                      base_frame_id_.c_str());
            return;
        }

        Eigen::Vector3d laser_pose_ = {laser_pose.pose.position.x,
                                       laser_pose.pose.position.y,
                                       0.0}; // laser mounting angle gets computed later -> set to 0.0 here!
        lasers_[laser_index]->SetLaserPose(laser_pose_);
        ROS_INFO("Received laser's pose wrt robot: %.3f %3f %3f", laser_pose_[0], laser_pose_[1], laser_pose_[2]);
        frame_to_laser_[laser_scan_frame_id] = laser_index;
    } else {
        // we have the laser pose, retrieve laser index
        laser_index = frame_to_laser_[laser_scan_frame_id];
    }
    
    // where was the robot when this scan was taken?
    Eigen::Vector3d pose;
    if(!getOdomPose(latest_odom_pose_, pose[0], pose[1], pose[2],
                    laser_scan->header.stamp, base_frame_id_)) {
        ROS_ERROR("Couldn't determine robot's pose associated with laser scan");                
        return;
    }

    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    if(f_init_) {
        // Compute change in pose
        delta.head(2) = pose.head(2) - f_odom_pose_.head(2);
        delta[2] = angle_diff(pose[2], f_odom_pose_[2]);

        // See if we should update the filter
        bool update = fabs(delta[0]) > min_trans_ ||
                      fabs(delta[1]) > min_trans_ ||
                      fabs(delta[2]) > min_rot_;
        update = update || m_force_update;
        m_force_update = false;

        // Set the laser update flags
        if(update) {
            for(int i = 0; i < lasers_update_.size(); i++) {
                lasers_update_[i] = true;
            }
        }
    }

    bool force_publication = false;
    if(!f_init_) {
        // Pose at last filter update
        f_odom_pose_ = pose;
        
        // Filter is now initialized
        f_init_ = true;
        for(int i = 0; i < lasers_update_.size(); i++) {
            lasers_update_[i] = true;
        }
        force_publication = true;
    } else if(f_init_ && lasers_update_[laser_index]) { // if the robot has moved, update the filter
        odom_->pose = pose;
        odom_->delta = delta;
        odom_->UpdateMotion(f_odom_pose_, pose, robot_pose);
    }
    
    bool matched = false;
    // if the robot has moved, update the filter
    if(lasers_update_[laser_index]) {
        lasers_[laser_index]->range_count = laser_scan->ranges.size();
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, laser_scan->angle_min);
        geometry_msgs::QuaternionStamped min_q, inc_q;
        min_q.header.stamp = laser_scan->header.stamp;
        min_q.header.frame_id = stripSlash(laser_scan->header.frame_id);
        tf2::convert(q, min_q.quaternion);

        inc_q.header = min_q.header;
        q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
        tf2::convert(q, inc_q.quaternion);
        try {
            tf_->transform(min_q, min_q, base_frame_id_);
            tf_->transform(inc_q, inc_q, base_frame_id_);
        } catch(const tf2::TransformException& e) {
            ROS_WARN("Unable to transform min/max laser angles into base frames: %s", e.what());
            return;
        }

        double angle_min = tf2::getYaw(min_q.quaternion);
        double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;
        angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;
        // ROS_INFO("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);
        
        // Apply range min/max thresholds, if the you supplied them
        if(laser_min_range_ > 0.0) {
            lasers_[laser_index]->range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
        } else {
            lasers_[laser_index]->range_min = laser_scan->range_min;
        }

        if(laser_max_range_ > 0.0) {
            lasers_[laser_index]->range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
        } else {
            lasers_[laser_index]->range_max = laser_scan->range_max;
        }
        lasers_[laser_index]->ranges = new double[lasers_[laser_index]->range_count][2];
        for(int i = 0; i < lasers_[laser_index]->range_count; i++) {
            if(laser_scan->ranges[i] <= lasers_[laser_index]->range_min) {
                lasers_[laser_index]->ranges[i][0] = lasers_[laser_index]->range_max; 
            } else if(laser_scan->ranges[i] > lasers_[laser_index]->range_max) {
                lasers_[laser_index]->ranges[i][0] = std::numeric_limits<decltype(lasers_[laser_index]->range_max)>::max();
            } else {
                lasers_[laser_index]->ranges[i][0] = laser_scan->ranges[i];
            }
            lasers_[laser_index]->ranges[i][1] = angle_min + i * angle_increment;
        }

        lasers_update_[laser_index] = false;

        // latest odom pose
        f_odom_pose_ = pose;

        // pointcloud in the local frame
        std::vector<Eigen::Vector2d> points;
        lasers_[laser_index]->computePointCloud(points);

        delete [] lasers_[laser_index]->ranges;
        
        Eigen::Matrix3d initial_guess = ConvertToHomogeneous(robot_pose);
        Eigen::Matrix3d T_t = sm_->Registration(points, map_, initial_guess, 3.0 * sigma, sigma / 3.0);
        Eigen::Matrix3d current_deviation = initial_guess.inverse() * T_t;
        adp_thresh_->UpdateModelDeviation(current_deviation);
        sigma = adp_thresh_->ComputeThreshold();

        robot_pose.head(2) = T_t.block<2, 1>(0, 2);
        robot_pose[2] = atan2(T_t(1, 0), T_t(0, 0));
        matched = true;
    }

    if(matched || force_publication) {
        geometry_msgs::PoseWithCovarianceStamped p;
        p.header.frame_id = map_frame_id_;
        p.header.stamp = laser_scan->header.stamp;
        p.pose.pose.position.x = robot_pose[0];
        p.pose.pose.position.y = robot_pose[1];

        tf2::Quaternion q;
        q.setRPY(0, 0, robot_pose[2]);
        tf2::convert(q, p.pose.pose.orientation);
        pose_pub_.publish(p);
        last_published_pose = p;

        // Subtracting base to odom from map to base and send map to odom instead
        geometry_msgs::PoseStamped odom_to_map;
        try {
            tf2::Quaternion q;
            q.setRPY(0, 0, robot_pose[2]);
            tf2::Transform tmp_tf(q, tf2::Vector3(robot_pose[0], robot_pose[1], 0.0));

            geometry_msgs::PoseStamped tmp_tf_stamped;
            tmp_tf_stamped.header.frame_id = base_frame_id_;
            tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
            tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

            this->tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
        } catch (const tf2::TransformException&) {
            ROS_INFO("Failed to subtract base to odom transform");
            return;
        }

        tf2::convert(odom_to_map.pose, latest_tf_);
        latest_tf_valid_ = true;
        if(tf_broadcast_) {
            // We want to send a transform that is good up until a
            // tolerance time so that odom can be used
            ros::Time transform_expiration = laser_scan->header.stamp + transform_tolerance_;
            geometry_msgs::TransformStamped tmp_tf_stamped;
            tmp_tf_stamped.header.frame_id = map_frame_id_;
            tmp_tf_stamped.header.stamp = transform_expiration;
            tmp_tf_stamped.child_frame_id = odom_frame_id_;
            tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);

            this->tfb_->sendTransform(tmp_tf_stamped);
            sent_first_transform_ = true;
        }
    } else if(latest_tf_valid_) {
        if(tf_broadcast_) {
            ros::Time transform_expiration = laser_scan->header.stamp + transform_tolerance_;
            geometry_msgs::TransformStamped tmp_tf_stamped;
            tmp_tf_stamped.header.frame_id = map_frame_id_;
            tmp_tf_stamped.header.stamp = transform_expiration;
            tmp_tf_stamped.child_frame_id = odom_frame_id_;
            tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);

            this->tfb_->sendTransform(tmp_tf_stamped);
        }

        // Is it time to save our last pose to the param server
        ros::Time now = ros::Time::now();
        if(save_pose_period.toSec() > 0.0 && (now - save_pose_last_time) >= save_pose_period) {
            this->savePoseToServer();
            save_pose_last_time = now;
        }
    }
}

void LocalizationNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    handleInitialPoseMessage(*msg);
    if(force_update_after_initialpose_) {
        m_force_update = true;
    }
}

void LocalizationNode::handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
    if(msg.header.frame_id == "") {
        ROS_WARN("Received initial pose with empty frame_id!");
    }else if(stripSlash(msg.header.frame_id) != map_frame_id_) {
        ROS_WARN("Ignoring initial pose in frame \"%s\", initial pose must be in the map frame, \"%s\"",
                stripSlash(msg.header.frame_id).c_str(),
                map_frame_id_.c_str());
        return;
    }
    geometry_msgs::TransformStamped tx_odom;
    try {
        tx_odom = tf_->lookupTransform(base_frame_id_, msg.header.stamp,
                                       base_frame_id_, ros::Time::now(),
                                       odom_frame_id_, ros::Duration(0.5));
    } catch (const tf2::TransformException& e) {
        if(sent_first_transform_) {
            ROS_WARN("Failed to transform initial pose in time %s", e.what());
        }
        tf2::convert(tf2::Transform::getIdentity(), tx_odom.transform);
    }
    tf2::Transform tx_odom_tf2;
    tf2::convert(tx_odom.transform, tx_odom_tf2);
    tf2::Transform pose_old, pose_new;
    tf2::convert(msg.pose.pose, pose_old);
    pose_new = pose_old * tx_odom_tf2;

    // Transform into the global frame
    ROS_INFO("Setting pose %.6f: %.3f %.3f %.3f",
                ros::Time::now().toSec(),
                pose_new.getOrigin().x(),
                pose_new.getOrigin().y(),
                tf2::getYaw(pose_new.getRotation()));
    // Re-initialize the filter
    robot_pose = {pose_new.getOrigin().x(),
                  pose_new.getOrigin().y(),
                  tf2::getYaw(pose_new.getRotation())};
    f_init_ = false;
}
void sigintHandler(int sig) {
    // Save latest pose as we're shutting down.
    localization_node_ptr->savePoseToServer();
    ros::shutdown();
}

LocalizationNode::LocalizationNode() : sent_first_transform_(false),
                                       latest_tf_valid_(false),
                                       map_(NULL),
                                       adp_thresh_(NULL),
                                       sm_(NULL),
                                       odom_(NULL),
                                       laser_(NULL),
                                       private_nh_("~"),
                                       first_map_received_(false)
{
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    // Get params off the param server
    private_nh_.param("use_map_topic", use_map_topic_, false);
    private_nh_.param("first_map_only", first_map_only_, false);

    double tmp;
    private_nh_.param("gui_publish_rate", tmp, -1.0);
    gui_publish_period = ros::Duration(1.0 / tmp);
    private_nh_.param("save_pose_rate", tmp, 0.5);
    save_pose_period = ros::Duration(1.0 / tmp);

    private_nh_.param("laser_min_range", laser_min_range_, 0.05);
    private_nh_.param("laser_max_range", laser_max_range_, 25.0);
    private_nh_.param("laser_max_beams", max_beams_, 540);

    private_nh_.param("odom_model_type", odom_model_type_, std::string("diff"));
    private_nh_.param("min_trans", min_trans_, 0.1);
    private_nh_.param("min_rot", min_rot_, 0.1);
    private_nh_.param("initial_threshold", initial_threshold_, 0.5);
    private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
    private_nh_.param("map_frame_id", map_frame_id_, std::string("map"));
    private_nh_.param("min_motion", min_motion_, 0.1);

    double tmp_tol;
    private_nh_.param("transform_tolerance", tmp_tol, 1.0);
    private_nh_.param("tf_broadcast", tf_broadcast_, true);
    private_nh_.param("force_update_after_initialpose", force_update_after_initialpose_, false);
    transform_tolerance_.fromSec(tmp_tol);

    odom_frame_id_ = stripSlash(odom_frame_id_);
    base_frame_id_ = stripSlash(base_frame_id_);
    map_frame_id_ = stripSlash(map_frame_id_);

    updatePoseFromServer();

    tfb_.reset(new tf2_ros::TransformBroadcaster());
    tf_.reset(new tf2_ros::Buffer());
    tfl_.reset(new tf2_ros::TransformListener(*tf_));

    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("vklc_pose", 2, true);

    laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic, 50);
    laser_scan_filter_ = new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_, odom_frame_id_, 50, nh_);
    laser_scan_filter_->registerCallback(boost::bind(&LocalizationNode::laserReceived,
                                                     this, _1));

    initial_pose_sub_ = nh_.subscribe("initialpose", 2, &LocalizationNode::initialPoseReceived, this);

    if(use_map_topic_) {
        map_sub_ = nh_.subscribe("map", 1, &LocalizationNode::mapReceived, this);
        ROS_INFO("Subscribed to map topic.");
    }else {
        requestMap();
    }
    m_force_update = false;

    laser_check_interval_ = ros::Duration(15.0);
    check_laser_timer_ = nh_.createTimer(laser_check_interval_,
                                         boost::bind(&LocalizationNode::checkLaserReceived, this, _1));

}

LocalizationNode::~LocalizationNode() {
    delete laser_scan_filter_;
    delete laser_scan_sub_;
    delete sm_;
    delete adp_thresh_;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_localization");
    ros::NodeHandle nh_;
    signal(SIGINT, sigintHandler);
    localization_node_ptr.reset(new LocalizationNode());
    ros::spin();
    localization_node_ptr.reset();
    return 0;
}