#include "vk_costmap_2d/inflation_layer.hpp"

using vk_costmap_2d::LETHAL_OBSTACLE;
using vk_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using vk_costmap_2d::NO_INFORMATION;

namespace vk_costmap_2d {

InflationLayer::InflationLayer() :
    resolution_(0),
    inflation_radius_(0),
    inscribed_radius_(0),
    weight_(0),
    inflate_unknown_(false),
    cell_inflation_radius_(0),
    cached_cell_inflation_radius_(0),
    seen_(NULL),
    cached_costs_(NULL),
    cached_distances_(NULL),
    last_min_x_(-std::numeric_limits<float>::max()),
    last_min_y_(-std::numeric_limits<float>::max()),
    last_max_x_(std::numeric_limits<float>::max()),
    last_max_y_(std::numeric_limits<float>::max()) {}

void InflationLayer::onInitialize() {
    boost::recursive_mutex::scoped_lock lock(inflation_access_);
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle nh;
    updated_ = true;
    private_nh.param("weight", weight_, 1.0);
    if (seen_)
      delete[] seen_;
    seen_ = NULL;
    seen_size_ = 0;
    matchSize();
}

void InflationLayer::matchSize() {
    boost::recursive_mutex::scoped_lock lock(inflation_access_);
    Costmap2D* master_costmap = layered_costmap_->getCostmap();
    resolution_ = master_costmap->getResolution();
    inflation_radius_ = layered_costmap_->getCircumscribedRadius();
    // inflation_radius_ = 0.1;
    inscribed_radius_ = layered_costmap_->getInscribedRadius();
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    computeCaches();

    unsigned int size_x = master_costmap->getSizeInCellsX();
    unsigned int size_y = master_costmap->getSizeInCellsY();
    if (seen_)
        delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
}

void InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double* min_x, double* min_y,
                                  double* max_x, double* max_y) {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;                                
}

void InflationLayer::onFootprintChanged() {
    inscribed_radius_ = layered_costmap_->getInscribedRadius();
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    computeCaches();
}

void InflationLayer::deleteKernels() {
    if(cached_distances_ != NULL) {
        for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
            if (cached_distances_[i])
                delete[] cached_distances_[i];
        }
        if (cached_distances_)
            delete[] cached_distances_;
        cached_distances_ = NULL;
    }
    if (cached_costs_ != NULL) {
        for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
            if (cached_costs_[i])
                delete[] cached_costs_[i];
        }
        delete[] cached_costs_;
        cached_costs_ = NULL;
    }
}

void InflationLayer::computeCaches() {
    if (cell_inflation_radius_ == 0)
        return;
    if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
        deleteKernels();
        cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
        cached_distances_ = new double*[cell_inflation_radius_ + 2];
        for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
            cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
            cached_distances_[i] = new double[cell_inflation_radius_ + 2];
            for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
                cached_distances_[i][j] = hypot(i, j);
            }
            cached_cell_inflation_radius_ = cell_inflation_radius_;
        }
    }

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
        for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
            cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
        }
    }
}

inline void InflationLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y) {
    if (!seen_[index]) {
        double distance = distanceLookup(mx, my, src_x, src_y);
        if (distance > cell_inflation_radius_)
            return;
        inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
    }                                   

}

void InflationLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j,
                                 int max_i, int max_j) {
    boost::recursive_mutex::scoped_lock lock(inflation_access_);
    if (cell_inflation_radius_ == 0)
        return;
    unsigned char* master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX();
    unsigned int size_y = master_grid.getSizeInCellsY();
    if (seen_ == NULL) {
        ROS_WARN("InflationLayer::updateCosts(): seen_ array is NULL");
        seen_size_ = size_x * size_y;
        seen_ = new bool[seen_size_];
    } else if (seen_size_ != size_x * size_y) {
        ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
        delete[] seen_;
        seen_size_ = size_x * size_y;
        seen_ = new bool[seen_size_];
    }
    memset(seen_, false, size_x * size_y * sizeof(bool));

    min_i -= cell_inflation_radius_;
    min_j -= cell_inflation_radius_;
    max_i += cell_inflation_radius_;
    max_j += cell_inflation_radius_;

    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(int(size_x), max_i);
    max_j = std::min(int(size_y), max_j);

    // Start with lethal obstacles: by definition distance is 0.0
    std::vector<CellData>& obs_bin = inflation_cells_[0.0];
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            int index = master_grid.getIndex(i, j);
            unsigned char cost = master_array[index];
            if (cost == LETHAL_OBSTACLE)
                obs_bin.push_back(CellData(index, i, j, i, j));
        }
    }

    std::map<double, std::vector<CellData> >::iterator bin;
    for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin) {
        for (int i = 0; i < bin->second.size(); ++i) {
            const CellData& cell = bin->second[i];
            unsigned int index = cell.index_;
            if (seen_[index]) { continue; }
            seen_[index] = true;

            unsigned int mx = cell.x_;
            unsigned int my = cell.y_;
            unsigned int sx = cell.src_x_;
            unsigned int sy = cell.src_y_;

            unsigned char cost = costLookup(mx, my, sx, sy);
            unsigned char old_cost = master_array[index];
            if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
                master_array[index] = cost;
            else
                master_array[index] = std::max(old_cost, cost);
            if (mx > 0)
                enqueue(index - 1, mx - 1, my, sx, sy);
            if (my > 0)
                enqueue(index - size_x, mx, my - 1, sx, sy);
            if (mx < size_x - 1)
                enqueue(index + 1, mx + 1, my, sx, sy);
            if (my < size_y - 1)
                enqueue(index + size_x, mx, my + 1, sx, sy);
        }
    }
    inflation_cells_.clear();
}

}   // vk_costmap_2d namespace