#include <vk_costmap_2d/obstacle_layer.hpp>

using vk_costmap_2d::NO_INFORMATION;
using vk_costmap_2d::LETHAL_OBSTACLE;
using vk_costmap_2d::FREE_SPACE;

using vk_costmap_2d::ObservationBuffer;
using vk_costmap_2d::Observation;

namespace vk_costmap_2d {

void ObstacleLayer::onInitialize() {
    ros::NodeHandle private_nh("~/" + name_), nh;
    rolling_window_ = layered_costmap_->isRolling();
    default_value_ = NO_INFORMATION;

    ObstacleLayer::matchSize();
    updated_ = true;

    global_frame_ = layered_costmap_->getGlobalFrameID();
    double transform_tolerance;
    private_nh.param("transform_tolerance", transform_tolerance, 0.2);

    std::string topics_string;
    private_nh.param("observation_sources", topics_string, std::string(""));
    ROS_INFO("Subscribed to Topics: %s", topics_string.c_str());

    double obstacle_range, raytrace_range;
    private_nh.param("obstacle_range", obstacle_range, 2.0);
    private_nh.param("raytrace_range", raytrace_range, 3.0);
    private_nh.param("combination_method", combination_method_, 1);
    
    // split the topics based on whitespace
    std::stringstream ss(topics_string);
    std::string source;
    while (ss >> source) {
        ros::NodeHandle source_node(private_nh, source);
        
        // get the parameters for the specific topic
        double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
        std::string topic, sensor_frame, data_type;
        bool inf_is_valid, clearing, marking;

        source_node.param("topic", topic, source);
        source_node.param("sensor_frame", sensor_frame, std::string(""));
        source_node.param("observation_keep_time", observation_keep_time, 0.0);
        source_node.param("expected_update_rate", expected_update_rate, 0.0);
        source_node.param("data_type", data_type, std::string("LaserScan"));
        source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
        source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
        source_node.param("inf_is_valid", inf_is_valid, false);
        source_node.param("clearing", clearing, false);
        source_node.param("marking", marking, true);
        
        if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan")) {
            ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
            throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
        }

        ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

        // create an observation buffer
        observation_buffers_.push_back(
            boost::shared_ptr<ObservationBuffer> (new ObservationBuffer(source, observation_keep_time, expected_update_rate,
                                                                        min_obstacle_height, max_obstacle_height,
                                                                        obstacle_range, raytrace_range, *tf_, global_frame_, sensor_frame, transform_tolerance)));
                                                                
        // check if we'll add this buffer to our marking observation buffers
        if (marking)
            marking_buffers_.push_back(observation_buffers_.back());

        // check if we'll also add this buffer to our clearing observation buffers
        if (clearing)
            clearing_buffers_.push_back(observation_buffers_.back());
        
        ROS_DEBUG("Created an observation buffer for source %s, topic %s, global frame: %s, "
                  "expected update rate: %.2f, observation persistence: %.2f",
                  source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

        // create a callback for the topic
        if (data_type == "LaserScan") {
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub(
                new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, topic, 50)
            );

            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> filter (
                new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50, nh)
            );

            if(inf_is_valid) {
                filter->registerCallback(boost::bind(&ObstacleLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
            } else {
                filter->registerCallback(boost::bind(&ObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back()));
            }
            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);
            observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
        }else if (data_type == "PointCloud") {
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud>> sub(
                new message_filters::Subscriber<sensor_msgs::PointCloud>(nh, topic, 50)
            );
            if (inf_is_valid) {}
            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud>> filter (
                new tf2_ros::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50, nh)
            );
            filter->registerCallback(boost::bind(&ObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));
            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);
        }else {
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub(
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, topic, 50)
            );
            if (inf_is_valid) {}
            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> filter (
                new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50, nh)
            );
            filter->registerCallback(boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));
            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);
        }
        if (sensor_frame != "") {
            std::vector < std::string > target_frames;
            target_frames.push_back(global_frame_);
            target_frames.push_back(sensor_frame);
            observation_notifiers_.back()->setTargetFrames(target_frames);
        }
    }
}

void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                 double* min_y, double* max_x, double* max_y) {
    if(rolling_window_)
        updateOrigin(robot_x -getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    bool updated = true;
    std::vector<Observation> observations, clearing_observations;

    // get the marking observations
    updated = updated && getMarkingObservations(observations);

    // get the clearing observations
    updated = updated && getClearingObservations(clearing_observations);
    
    // update the global current status
    updated_ = updated;

    // raytrace freespace
    for(unsigned int i = 0; i < clearing_observations.size(); ++i) {
        raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
    }
    
    for(std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it) {
        const Observation& obs = *it;
        const sensor_msgs::PointCloud2& cloud = *(obs.cloud_);
        double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        for (; iter_x !=iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            double px = *iter_x, py = *iter_y, pz = *iter_z;
            if(pz > max_obstacle_height_)
                continue;
    
            // compute the squared distance from the hitpoint to the pointcloud's origin
            double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x)
                           + (py - obs.origin_.y) * (py - obs.origin_.y)
                           + (pz - obs.origin_.z) * (pz - obs.origin_.z);

            if(sq_dist >= sq_obstacle_range)
                continue;

            // compute the map coordinates for the observation
            unsigned int mx, my;
            if(!worldToMap(px, py, mx, my))
                continue;
            unsigned int index = getIndex(mx, my);
            costmap_[index] = LETHAL_OBSTACLE; 
            touch(px, py, min_x, min_y, max_x, max_y); 
        }
    }
    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (footprint_clearing_enabled_) {
        setConvexPolygonCost(transformed_footprint_, FREE_SPACE);
    }

    switch (combination_method_) {
        case 0:  // Overwrite
            updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
            break;
        case 1:  // Maximum
            updateWithMax(master_grid, min_i, min_j, max_i, max_j); 
            break;
        default:  // Nothing
            break;
    }
}

void ObstacleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg,
                                      const boost::shared_ptr<ObservationBuffer>& buffer) {
    sensor_msgs::PointCloud2 cloud;
    cloud.header = msg->header; 
    try{
        projector_.transformLaserScanToPointCloud(msg->header.frame_id, *msg, cloud, *tf_);
    }catch (tf2::TransformException &ex) {
        ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
             ex.what());
        projector_.projectLaser(*msg, cloud);
    }

    buffer->lock_mutex_.lock();
    buffer->bufferCloud(cloud); 
    buffer->lock_mutex_.unlock();
}

void ObstacleLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_msg,
                                              const boost::shared_ptr<ObservationBuffer>& buffer) {
    // Filter positive infinities ("Inf"s) to max_range.
    float epsilon = 1e-4;
    sensor_msgs::LaserScan msg = *raw_msg;
    for(int i = 0; i < msg.ranges.size(); i++) {
        float range = msg.ranges[i];
        if(!std::isfinite(range) && range > 0) {
            msg.ranges[i] = msg.range_max - epsilon;
        }
    }

    sensor_msgs::PointCloud2 cloud;
    cloud.header = msg.header;
    try{
        projector_.transformLaserScanToPointCloud(msg.header.frame_id, msg, cloud, *tf_);
    }catch (tf2::TransformException &ex) {
        ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
             ex.what());
        projector_.projectLaser(msg, cloud);
    }
    buffer->lock_mutex_.lock();
    buffer->bufferCloud(cloud);
    buffer->lock_mutex_.unlock();                                                    
}

void ObstacleLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& msg,
                                       const boost::shared_ptr<ObservationBuffer>& buffer) {
    sensor_msgs::PointCloud2 cloud2;
    if(!sensor_msgs::convertPointCloudToPointCloud2(*msg, cloud2)) {
        ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
        return;
    }
    buffer->lock_mutex_.lock();
    buffer->bufferCloud(cloud2);
    buffer->lock_mutex_.unlock();  
}

void ObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg,
                                        const boost::shared_ptr<ObservationBuffer>& buffer) {
    buffer->lock_mutex_.lock();
    buffer->bufferCloud(*msg);
    buffer->lock_mutex_.unlock();
}

bool ObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const {
    bool updated = true;
    // get the marking observations
    for(unsigned int i = 0; i < marking_buffers_.size(); i++) {
        marking_buffers_[i]->lock_mutex_.lock();
        marking_buffers_[i]->getObservations(marking_observations);
        updated = marking_buffers_[i]->isUpdated() && updated;
        marking_buffers_[i]->lock_mutex_.unlock();
    }
    return updated;
}

bool ObstacleLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const {
    bool updated = true;
    // get the clearing observations
    for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) {
        clearing_buffers_[i]->lock_mutex_.lock();
        clearing_buffers_[i]->getObservations(clearing_observations);
        updated = clearing_buffers_[i]->isUpdated() && updated;
        clearing_buffers_[i]->lock_mutex_.unlock();
    }
    return updated;
}

void ObstacleLayer::raytraceFreespace(const Observation& clearing_observation,
                                      double* min_x, double* min_y,
                                      double* max_x, double* max_y) {
    // get the world coordinates of the origin of the sensor
    double ox = clearing_observation.origin_.x;
    double oy = clearing_observation.origin_.y;
    const sensor_msgs::PointCloud2 &cloud = *(clearing_observation.cloud_);                                     

    // convert to the map coordinates of the origin of the sensor
    unsigned int x0, y0;
    if (!worldToMap(ox, oy, x0, y0)) {
        ROS_WARN_THROTTLE(1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
                          ox, oy);
        return;
    }
    
    // tinh toan cac diem cuoi cua ban do
    double origin_x = origin_x_, origin_y = origin_y_;
    double map_end_x = origin_x + size_x_ * resolution_;
    double map_end_y = origin_y + size_y_ * resolution_;
    
    touch(ox, oy, min_x, min_y, max_x, max_y);

    // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        double wx = *iter_x;
        double wy = *iter_y;

        double a = wx - ox;
        double b = wy - oy;

        if(wx < origin_x) {
            double t = (origin_x - ox) / a;
            wx = origin_x;
            wy = oy + b * t;
        }
        if (wy < origin_y) {
            double t = (origin_y - oy) / b;
            wx = ox + a * t;
            wy = origin_y;
        }

        if (wx > map_end_x) {
            double t = (map_end_x - ox) / a;
            wx = map_end_x - .001;
            wy = oy + b * t;
        }
        if (wy > map_end_y) {
            double t = (map_end_y - oy) / b;
            wx = ox + a * t;
            wy = map_end_y - .001;
        }

        // get the map coordinates of its endpoint
        unsigned int x1, y1;
        if (!worldToMap(wx, wy, x1, y1))
            continue;
    
        unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
        raytraceLine(FREE_SPACE, x0, y0, x1, y1, cell_raytrace_range);
        updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
    }
}

void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y) {
    double dx = wx-ox, dy = wy-oy;
    double full_distance = hypot(dx, dy);
    double scale = std::min(1.0, range / full_distance);
    double ex = ox + dx * scale, ey = oy + dy * scale;
    touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw,
                                    double* min_x, double* min_y,
                                    double* max_x, double* max_y) {
    if(!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
    for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
        touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

}   // vk_costmap_2d namespace