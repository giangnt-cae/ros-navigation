#include "vk_costmap_2d/costmap_2d_ros.hpp"
namespace vk_costmap_2d {

char* Costmap2DROS::cost_translation_table_ = NULL;

Costmap2DROS::Costmap2DROS(const std::string &name, tf2_ros::Buffer& tf) :
    layered_costmap_(NULL),
    name_(name),
    tf_(tf),
    transform_tolerance_(0.3),
    last_publish_(0),
    map_update_thread_shutdown_(false),
    private_nh("~")
{
    if(cost_translation_table_ == NULL) {
        cost_translation_table_ = new char[256];
        cost_translation_table_[0] = 0;         // No obstacle
        cost_translation_table_[253] = 99;      // INSCRIBED obstacle
        cost_translation_table_[254] = 100;     // LETHAL obstacle
        cost_translation_table_[255] = -1;      // UNKNOWN

        for(int i = 1; i < 253; i++) {
            cost_translation_table_[i] = char(1 + (97 * (i-1)) / 251);
        }
    }

    bool rolling_window;
    private_nh.param("global_frame", global_frame_, std::string("map"));
    private_nh.param("base_frame", base_frame_, std::string("base_link"));
    private_nh.param("rolling_window", rolling_window, true);
    private_nh.param("publish_rate", publish_rate, 10.0);
    private_nh.param("width", width_, 3.0);
    private_nh.param("height", height_, 3.0);
    private_nh.param("resolution", resolution_, 0.02);
    private_nh.param("origin_x", origin_x_, -width_/2);
    private_nh.param("origin_y", origin_y_, -height_/2);
    private_nh.param("robot_radius", robot_radius_, 0.5);
    private_nh.param("footprint_padding_X", footprint_padding_X_, (float)0.0);
    private_nh.param("footprint_padding_Y", footprint_padding_Y_, (float)0.0);

    if (loadFootprintFromParam(private_nh, "unpadded_footprint", unpadded_footprint_)) {
        ROS_INFO("Successfully loaded unpadded footprint.");
        padded_footprint_ = unpadded_footprint_;
        #ifdef makeFootprintFromRadius
        padded_footprint_ = makeFootprintFromRadius(robot_radius_, 32);
        #endif
        
        if(footprint_padding_X_ > 0)
            padFootprintX(padded_footprint_, footprint_padding_X_);
        if(footprint_padding_Y_ > 0)
            padFootprintY(padded_footprint_, footprint_padding_Y_);
    } else {
        ROS_WARN("Failed to load unpadded footprint. Using default.");
    }

    ros::Time last_error = ros::Time::now();
    std::string tf_error;
    // we need to make sure that the transform between the robot base frame and the global frame is available
    while (ros::ok() 
           && !tf_.canTransform(global_frame_, base_frame_, ros::Time(), ros::Duration(0.1), &tf_error)) {
        ros::spinOnce();
        if (last_error + ros::Duration(5.0) < ros::Time::now()) {
            ROS_WARN("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
                     base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
            last_error = ros::Time::now();
        }
        tf_error.clear();
    }

    footprint_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);
    costmap_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("costmap", 1);
    agv_pub_ = private_nh.advertise<vk_costmap_2d::AgvInfo>("agv_info", 1);

    layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window);
    layered_costmap_->setFootprint(padded_footprint_);

    // Add static layer
    // boost::shared_ptr<Layer> static_layer(new StaticLayer());
    // layered_costmap_->addLayer(static_layer);
    // static_layer->initialize(layered_costmap_, name + "/static_layer", &tf_);

    // Add obstacle layer from sensor data
    boost::shared_ptr<Layer> obstacle_layer(new ObstacleLayer());
    layered_costmap_->addLayer(obstacle_layer);
    obstacle_layer->initialize(layered_costmap_, name + "/obstacle_layer", &tf_);
    
    // Add AGV layer
    boost::shared_ptr<Layer> agv_layer(new AgvLayer());
    layered_costmap_->addLayer(agv_layer);
    agv_layer->initialize(layered_costmap_, name + "/agv_layer", &tf_);
    agv_.id = agv_layer->getID();
    
    // Add inflation layer
    boost::shared_ptr<Layer> inflation_layer(new InflationLayer());
    layered_costmap_->addLayer(inflation_layer);
    inflation_layer->initialize(layered_costmap_, name + "/inflation_layer", &tf_);

    if(!layered_costmap_->isSizeLocked()) {
        layered_costmap_->resizeMap((unsigned int)(width_/resolution_), (unsigned int)(height_/resolution_),
                                    resolution_, origin_x_, origin_y_);
    }
    
    if(publish_rate > 0) {
        publish_cycle = ros::Duration(1.0/publish_rate);
        map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, publish_rate));
    }
}

Costmap2DROS::~Costmap2DROS() { 
    map_update_thread_shutdown_ = true;
    if (map_update_thread_ != NULL) {
        map_update_thread_->join();
        delete map_update_thread_;
    }
    delete layered_costmap_; 
}

void Costmap2DROS::mapUpdateLoop(double frequency) {
    ros::Rate r(frequency);
    while (nh.ok() && !map_update_thread_shutdown_) {
        updateMap();
        if (publish_cycle.toSec() > 0 && layered_costmap_->isInitialized()) {
            /* If publish global costmap, only use updated map */
            // unsigned int x0, y0, xn, yn;
            // layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
            // ...

            ros::Time now = ros::Time::now();
            if (last_publish_ + publish_cycle < now) {
                publishCostmap();
                last_publish_ = now;
            }
        }
        r.sleep();
        if (r.cycleTime() > ros::Duration(1 / frequency))
            ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency, r.cycleTime().toSec());
    }
}

void Costmap2DROS::publishCostmap() {
    Costmap2D* master_costmap_ = layered_costmap_->getCostmap();
    double resolution = master_costmap_->getResolution();
    grid_.header.frame_id = global_frame_;
    grid_.header.stamp = ros::Time::now();
    grid_.info.resolution = resolution;
    grid_.info.width = master_costmap_->getSizeInCellsX();
    grid_.info.height = master_costmap_->getSizeInCellsY();

    double wx, wy;
    master_costmap_->mapToWorld(0, 0, wx, wy);
    grid_.info.origin.position.x = wx - resolution / 2;
    grid_.info.origin.position.y = wy - resolution / 2;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.resize(grid_.info.width * grid_.info.height);

    unsigned char* data = master_costmap_->getCharMap();
    for (unsigned int i = 0; i < grid_.data.size(); i++) {
        grid_.data[i] = cost_translation_table_[data[i]];
    }
    costmap_pub_.publish(grid_);

    auto layer = boost::dynamic_pointer_cast<vk_costmap_2d::CostmapLayer>(layered_costmap_->getLayers()->at(0));
    if(layer) {
        agv_.map.header = grid_.header;
        agv_.map.info = grid_.info;
        agv_.map.data.resize(grid_.data.size());
        unsigned char* data = layer->getCharMap();
        for (unsigned int i = 0; i < agv_.map.data.size(); i++) {
            agv_.map.data[i] = cost_translation_table_[data[i]];
        }
    }
    agv_pub_.publish(agv_);
}

void Costmap2DROS::updateMap() {
    geometry_msgs::PoseStamped pose;
    if (getRobotPose(pose)) {
        double x = pose.pose.position.x,
               y = pose.pose.position.y,
               yaw = tf2::getYaw(pose.pose.orientation);
        
        layered_costmap_->updateMap(x, y, yaw);
    
        geometry_msgs::PolygonStamped footprint;
        footprint.header.frame_id = global_frame_;
        footprint.header.stamp = ros::Time::now();
        transformFootprint(x, y, yaw, padded_footprint_, footprint);
        footprint_pub_.publish(footprint);

        agv_.robot_pose = pose;
        agv_.footprint = footprint;
    }
}

bool Costmap2DROS::getRobotPose(geometry_msgs::PoseStamped& global_pose) const {
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
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
        return false;
    }

    return true;
}

bool Costmap2DROS::loadFootprintFromParam(ros::NodeHandle& nh, const std::string& param_name, std::vector<geometry_msgs::Point>& footprint) {
    XmlRpc::XmlRpcValue footprint_list;
    // Check if the parameter exists and is a list
    if(!nh.getParam(param_name, footprint_list) || footprint_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Parameter %s is not a valid list!", param_name.c_str());
        return false;
    }
    try{
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
    } catch (const XmlRpc::XmlRpcException& ex) {
        ROS_ERROR("Error parsing footprint: %s", ex.getMessage().c_str());
        return false;
    }
    return true;
}

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_costmap_2d");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    vk_costmap_2d::Costmap2DROS vkm("costmap", buffer);
    ros::spin();
    return 0;
}