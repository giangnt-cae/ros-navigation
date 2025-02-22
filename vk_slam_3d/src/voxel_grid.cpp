#include <vk_slam_3d/voxel_grid.hpp>

namespace vk_slam_3d {

int VoxelGrid::hashfunction(const Eigen::Vector3i& voxel) {
    std::size_t p1 = 73856093, p2 = 19349669, p3 = 83492791;
    return ((voxel[0]*p1) ^ (voxel[1]*p2) ^ (voxel[2]*p3)) % hash_size_;
}

void VoxelGrid::insert(const Eigen::Vector3d& point,
                       const Eigen::Vector3i& voxel) {
    int hash_value = hashfunction(voxel);
    Voxel* temp = new Voxel;
    temp->position = voxel;
    temp->points.push_back(point);
    temp->next = NULL;

    if(hash_table[hash_value] == NULL) {
        hash_table[hash_value] = temp;
        return;
    }

    Voxel* curr = hash_table[hash_value];
    bool inserted = false;
    while(curr != NULL) {
        if(curr->position == voxel) {
            curr->points.push_back(point);
            return;
        }
        curr = curr->next;
    }
    if(!inserted) {
        temp->next = hash_table[hash_value];
        hash_table[hash_value] = temp;
    }
}

void VoxelGrid::insert(const Eigen::Vector3d& point,
                       const Eigen::Vector3i& voxel,
                       int max_points_per_voxel) {
    int hash_value = hashfunction(voxel);
    Voxel* temp = new Voxel;
    temp->position = voxel;
    temp->points.push_back(point);
    temp->next = NULL;

    if(hash_table[hash_value] == NULL) {
        hash_table[hash_value] = temp;
        return;
    }

    Voxel* curr = hash_table[hash_value];
    bool inserted = false;
    while(curr != NULL) {
        if(curr->position == voxel && curr->points.size() < max_points_per_voxel) {
            curr->points.push_back(point);
            return;
        }
        curr = curr->next;
    }

    if(!inserted) {
        temp->next = hash_table[hash_value];
        hash_table[hash_value] = temp;
    }
}

std::vector<Eigen::Vector3d> VoxelGrid::getpoints(const Eigen::Vector3i& voxel) {
    std::vector<Eigen::Vector3d> points;
    int hash_value = hashfunction(voxel);
    Voxel* temp = hash_table[hash_value];
    while(temp != NULL) {
        if(temp->position == voxel) {
            for(auto& point : temp->points) { points.push_back(point); }
        }
        temp = temp->next;
    }
    return points;
}

}   // namespace vk_slam_3d