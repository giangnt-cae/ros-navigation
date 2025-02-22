#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <vk_slam_3d/voxel_grid.hpp>

namespace vk_slam_3d {

class VoxelHashMap {
    private:
        vk_slam_3d::VoxelGrid map_;
        
        double voxel_size_;
        int max_points_per_voxel_;

    public:
        VoxelHashMap(double voxel_size, int max_points_per_voxel)
            : voxel_size_(voxel_size),
            max_points_per_voxel_(max_points_per_voxel) {}

        ~VoxelHashMap() {}

        // Check if the voxel map is empty or not
        bool empty() const { return map_.empty(); }

        void addPoints(const std::vector<Eigen::Vector3d>& points);
        void update(std::vector<Eigen::Vector3d>& points, Eigen::Affine3d& T);
        std::vector<Eigen::Vector3d> extractPoints();
        
        sensor_msgs::PointCloud eigenToPointCloudMsg(const std::vector<Eigen::Vector3d>& points, std::string frame);

        // Get the size of voxel
        double getVoxelSize() { return voxel_size_; }
        int getNumMaxPointsPerVoxel() { return max_points_per_voxel_; }
        
        // Get points in the voxel
        std::vector<Eigen::Vector3d> getPoints(const Eigen::Vector3i& voxel) { return map_.getpoints(voxel); }

        typedef boost::shared_ptr<VoxelHashMap> Ptr;
        typedef boost::shared_ptr<const VoxelHashMap> ConstPtr;

};  // class VoxelHashMap


}   // namespace vk_slam_3d
