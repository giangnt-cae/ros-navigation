#include "vk_costmap_2d/observation.hpp"

namespace vk_costmap_2d {

ObservationBuffer::ObservationBuffer(std::string topic_name, double observation_keep_time, double expected_update_rate,
                    double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                    double raytrace_range, tf2_ros::Buffer& tf2_buffer, std::string global_frame,
                    std::string sensor_frame, double tf_tolerance) : 
    topic_name_(topic_name), observation_keep_time_(observation_keep_time),
    expected_update_rate_(expected_update_rate), last_updated_(ros::Time::now()),
    global_frame_(global_frame), sensor_frame_(sensor_frame), tf2_buffer_(tf2_buffer),
    min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range), raytrace_range_(raytrace_range), tf_tolerance_(tf_tolerance) {}

ObservationBuffer::~ObservationBuffer() {}

void ObservationBuffer::bufferCloud(const sensor_msgs::PointCloud2& cloud) {
    geometry_msgs::PointStamped global_origin; 
    observation_list_.push_front(Observation());
    std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;
    try {
        geometry_msgs::PointStamped local_origin;
        local_origin.header.stamp = cloud.header.stamp;
        local_origin.header.frame_id = origin_frame;
        local_origin.point.x = 0;
        local_origin.point.y = 0;
        local_origin.point.z = 0;
        tf2_buffer_.transform(local_origin, global_origin, global_frame_);
        tf2::convert(global_origin.point, observation_list_.front().origin_);
        observation_list_.front().raytrace_range_ = raytrace_range_;
        observation_list_.front().obstacle_range_ = obstacle_range_;

        sensor_msgs::PointCloud2 global_frame_cloud;
        tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_);
        global_frame_cloud.header.stamp = cloud.header.stamp;

        /* Filter with height threshold */
        sensor_msgs::PointCloud2& observation_cloud = *(observation_list_.front().cloud_);
        observation_cloud.height = global_frame_cloud.height;
        observation_cloud.width = global_frame_cloud.width;
        observation_cloud.fields = global_frame_cloud.fields;
        observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
        observation_cloud.point_step = global_frame_cloud.point_step;
        observation_cloud.row_step = global_frame_cloud.row_step;
        observation_cloud.is_dense = global_frame_cloud.is_dense;
        
        unsigned int cloud_size = global_frame_cloud.height*global_frame_cloud.width;
        sensor_msgs::PointCloud2Modifier modifier(observation_cloud);
        modifier.resize(cloud_size);
        unsigned int point_count = 0;
        
        sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
        std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin();
        std::vector<unsigned char>::const_iterator iter_global_end = global_frame_cloud.data.end();
        std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
        for(; iter_global != iter_global_end; ++iter_z, iter_global += global_frame_cloud.point_step) {
            if((*iter_z) <= max_obstacle_height_ && (*iter_z) >= min_obstacle_height_) {
                std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
                iter_obs += global_frame_cloud.point_step;
                ++point_count;
            }
        }

        modifier.resize(point_count);
        observation_cloud.header.stamp = cloud.header.stamp;
        observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;

    }catch(tf2::TransformException& ex) {
        observation_list_.pop_front();
        ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
              cloud.header.frame_id.c_str(), ex.what());
        return;
    }
    last_updated_ = ros::Time::now();
    removeStaleObservations();
}

void ObservationBuffer::removeStaleObservations() {
    if(!observation_list_.empty()) {
        std::list<Observation>::iterator obs_it = observation_list_.begin();
        if(observation_keep_time_ == ros::Duration(0.0)) {
            observation_list_.erase(++obs_it, observation_list_.end());
            return;
        }

        for(obs_it= observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
            Observation& obs = *obs_it;
            if((last_updated_ - obs.cloud_->header.stamp) > observation_keep_time_) {
                observation_list_.erase(obs_it, observation_list_.end());
                return;
            }
        }
    }
}

void ObservationBuffer::getObservations(std::vector<Observation>& observations) {
    removeStaleObservations();
    std::list<Observation>::iterator obs_it;
    for(obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
        observations.push_back(*obs_it);
    }
}

bool ObservationBuffer::isUpdated() const {
    if(expected_update_rate_ == ros::Duration(0.0)) return true;

    bool updated = (ros::Time::now() - last_updated_).toSec()
                    <= expected_update_rate_.toSec();
    return updated;
}

}   // namespace vk_costmap_2d