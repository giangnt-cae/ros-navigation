#include "vk_costmap_2d/costmap_layer.hpp"

namespace vk_costmap_2d {

void CostmapLayer::matchSize() {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
}

void CostmapLayer::clearArea(int start_x, int start_y, int end_x,
                             int end_y, bool invert_area) {
    unsigned char* grid = getCharMap();
    for(int x = 0; x < (int)getSizeInCellsX(); x++) {
        for(int y = 0; y < (int)getSizeInCellsY(); y++) {
            bool inside_area = (x > start_x && x < end_x) && 
                               (y > start_y && y < end_y);
            // Nếu invert_area == false, chỉ xóa các ô bên ngoài vùng
            // Nếu invert_area == true, chỉ xóa các ô bên trong vùng
            if(inside_area == invert_area) {
                int index = getIndex(x, y);
                if(grid[index] != NO_INFORMATION) {
                    grid[index] = NO_INFORMATION;
                }
            }
        }
    }
}

void CostmapLayer::updateWithTrueOverwrite(vk_costmap_2d::Costmap2D& master_grid,
                                           int min_i, int min_j, 
                                           int max_i, int max_j) {
    if(!enabled_) return;

    unsigned char* master = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();
    for(int j = min_j; j < max_j; j++) {
        unsigned int it = j * span + min_i;
        for(int i = min_i; i < max_i; i++) {
            master[it] = costmap_[it];
            it++;
        }
    }
}

void CostmapLayer::updateWithOverwrite(vk_costmap_2d::Costmap2D& master_grid,
                                       int min_i, int min_j, 
                                       int max_i, int max_j) {
    if(!enabled_) return;

    unsigned char* master = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();
    for(int j = min_j; j < max_j; j++) {
        unsigned int it = j * span + min_i;
        for(int i = min_i; i < max_i; i++) {
            if(costmap_[it] != NO_INFORMATION) {
                master[it] = costmap_[it];
            }
            it++;
        }
    }
}

void CostmapLayer::updateWithMax(vk_costmap_2d::Costmap2D& master_grid,
                                 int min_i, int min_j, 
                                 int max_i, int max_j) {
    if(!enabled_) return;

    unsigned char* master = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();
    for(int j = min_j; j < max_j; j++) {
        unsigned int it = j * span + min_i;
        for(int i = min_i; i < max_i; i++) {
            if(costmap_[it] == NO_INFORMATION) {
                it++;
                continue;
            }

            unsigned char old_cost = master[it];
            if(old_cost == NO_INFORMATION || old_cost < costmap_[it]) {
                master[it] = costmap_[it];
            }
            it++;
        }
    }
}

void CostmapLayer::updateWithMaxupdateWithAddition(vk_costmap_2d::Costmap2D& master_grid,
                                                   int min_i, int min_j, 
                                                   int max_i, int max_j) {
    if(!enabled_) return;

    unsigned char* master = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();
    for(int j = min_j; j < max_j; j++) {
        unsigned int it = j * span + min_i;
        for(int i = min_i; i < max_i; i++) {
            if(costmap_[it] != NO_INFORMATION) {
                it++;
                continue;
            }
            unsigned char old_cost = master[it];
            if(old_cost == NO_INFORMATION) master[it] = costmap_[it];
            else {
                int sum = old_cost + costmap_[it];
                if(sum >= INSCRIBED_INFLATED_OBSTACLE) {
                    master[it] = INSCRIBED_INFLATED_OBSTACLE - 1;
                }else {
                    master[it] = sum;
                }
            }
        }
    }
}

void CostmapLayer::touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y) {
    *min_x = std::min(x, *min_x);
    *min_y = std::min(y, *min_y);
    *max_x = std::max(x, *max_x);
    *max_y = std::max(y, *max_y);
}

}   // namespace vk_costmap_2d