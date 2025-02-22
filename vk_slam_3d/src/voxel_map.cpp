#include <vk_slam_3d/voxel_map.hpp>

namespace vk_slam_3d {

void VoxelHashMap::addPoints(const std::vector<Eigen::Vector3d>& points) {
    double voxel_size = getVoxelSize();
    int max_points_per_voxel = getNumMaxPointsPerVoxel();
    for(auto& point : points) {
        Eigen::Vector3i voxel = (point / voxel_size).cast<int>();
        map_.insert(point, voxel, max_points_per_voxel);
    }
}

void VoxelHashMap::update(std::vector<Eigen::Vector3d>& points,
                          Eigen::Affine3d& T) {
    std::vector<Eigen::Vector3d> points_transformed;
    for(auto& point : points) {
        Eigen::Vector3d point_transformed = T.rotation() * point + T.translation();;
        points_transformed.push_back(point_transformed);
    }
    addPoints(points_transformed);
}

std::vector<Eigen::Vector3d> VoxelHashMap::extractPoints() {
    std::vector<Eigen::Vector3d> points;
    Voxel* temp;
    for(int i = 0; i < map_.gethashsize(); i++) {
        temp = map_.hash_table[i];
        while(temp != NULL) {
            for(auto& point : temp->points) { points.push_back(point); }
            temp = temp->next;
        }
    }
    return points;
}

sensor_msgs::PointCloud VoxelHashMap::eigenToPointCloudMsg(const std::vector<Eigen::Vector3d>& points,
                                                           std::string frame) {
    sensor_msgs::PointCloud pointclouds;
    pointclouds.header.frame_id = frame;
    pointclouds.header.stamp = ros::Time::now();
    for(auto& p : points) {
        geometry_msgs::Point32 point_msg;
        point_msg.x = p[0];
        point_msg.y = p[1];
        point_msg.z = p[2];
        pointclouds.points.push_back(point_msg);
    }
    return pointclouds;
}

}   // namespace vk_slam_3d