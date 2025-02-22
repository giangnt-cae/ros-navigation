#pragma once

#include <ros/ros.h>

#include <eigen3/Eigen/Core>

namespace vk_slam_3d {

struct Voxel {
    Eigen::Vector3i position;
    std::vector<Eigen::Vector3d> points;
    Voxel* next;
};
    
class VoxelGrid {
    private:
        static const int hash_size_ = (1 << 20 - 1);
        int hashfunction(const Eigen::Vector3i& voxel);
    
    public:
        // Insert one point into a voxel
        void insert(const Eigen::Vector3d& point,
                   const Eigen::Vector3i& voxel);
        
        // Insert one point into a voxel if number of points < max_points_per_voxel
        void insert(const Eigen::Vector3d& point,
                   const Eigen::Vector3i& voxel, 
                   int max_points_per_voxel);
            
        // Get points in the voxel
        std::vector<Eigen::Vector3d> getpoints(const Eigen::Vector3i& voxel);
        
        // Check if the voxel grid is empty or not
        bool empty() const { for(auto& pt : hash_table) { if(pt != NULL) { return false; }} return true; }

        Voxel* hash_table[hash_size_];
    
        inline int gethashsize() { return hash_size_; }

    };  // class VoxelGrid
    
}   // namespace vk_slam_3d