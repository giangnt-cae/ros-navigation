#include <graph_based_slam/map.hpp>

namespace slam2d {

Map2D::Map2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
             double origin_x, double origin_y, int default_value) :
    size_x_(cells_size_x),
    size_y_(cells_size_y),
    resolution_(resolution),
    origin_x_(origin_x),
    origin_y_(origin_y),
    occmap_(NULL),
    default_value_(default_value)
{
    initMaps(size_x_, size_y_);
    resetFullMap();
}

Map2D::Map2D() :
    size_x_(0),
    size_y_(0),
    resolution_(0.0),
    origin_x_(0.0),
    origin_y_(0.0),
    occmap_(NULL) {}

Map2D::~Map2D() {
    deleteMaps();
}

void Map2D::initMaps(unsigned int size_x, unsigned int size_y) {
    boost::recursive_mutex::scoped_lock lock(configuration_mutex_);
    delete [] occmap_;
    occmap_ = new int[size_x * size_y];
}

void Map2D::resetFullMap() {
    boost::recursive_mutex::scoped_lock lock(configuration_mutex_);
    memset(occmap_, default_value_, size_x_*size_y_*sizeof(int));
}

void Map2D::deleteMaps() {
    boost::recursive_mutex::scoped_lock lock(configuration_mutex_);
    delete [] occmap_;
    occmap_ = NULL;
}

void Map2D::resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                      double origin_x, double origin_y) {
    size_x_ = size_x;
    size_y_ = size_y;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    initMaps(size_x, size_y);                        
    resetFullMap();
}

int Map2D::getCost(unsigned int mx, unsigned int my) const {
    return  occmap_[getIndex(mx, my)];
}

void Map2D::setCost(unsigned int mx, unsigned int my, int cost) {
    occmap_[getIndex(mx, my)] = cost;
}

void Map2D::updateCost(unsigned int mx, unsigned int my, int value) {
    occmap_[getIndex(mx, my)] += value;
    if(occmap_[getIndex(mx, my)] > MAX_OCCUPIED)
        occmap_[getIndex(mx, my)] = MAX_OCCUPIED;
    else if (occmap_[getIndex(mx, my)] < MIN_FREE)
        occmap_[getIndex(mx, my)] = MIN_FREE;
    else
        return;
}

void Map2D::mapToWorld(unsigned int mx, unsigned int my,
                           double& wx, double& wy) const {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;                        
}

bool Map2D::worldToMap(double wx, double wy,
                       unsigned int& mx, unsigned int& my) const {
    if(wx < origin_x_ || wy < origin_y_) return false;

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if(mx < size_x_ && my < size_y_) return true;

    return false;
}

bool Map2D::mapValid(int mx, int my) const {
    return ( (mx >= 0) && (mx < size_x_) && (my >= 0) && (my < size_y_) );
}


}   // slam2d namespace