#include "vk_costmap_2d/costmap_2d.hpp"

namespace vk_costmap_2d {

void bresenham2D(std::vector<MapLocation>& cells, 
                 MapLocation pt_begin, MapLocation pt_end,
                 double max_length) {
    int x0, y0, x1, y1;
    x0 = pt_begin.x;
    y0 = pt_begin.y;
    x1 = pt_end.x;
    y1 = pt_end.y;
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    double dist = std::hypot(dx, dy);
    if (dist > max_length) {
        double scale = max_length / dist;
        x1 = x0 + (int)((x1 - x0) * scale);
        y1 = y0 + (int)((y1 - y0) * scale);
        dx = std::abs(x1 - x0);
        dy = std::abs(y1 - y0);
    }
    int err = dx - dy;
    while (true) {
        cells.push_back({(unsigned int)x0, (unsigned int)y0});
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void Costmap2D::raytraceLine(unsigned char value, unsigned int x0, unsigned int y0,
                             unsigned int x1, unsigned int y1, unsigned int max_length) {
    int dx = x1 - x0;
    int dy = y1 - y0;
    int abs_dx = std::abs(dx);
    int abs_dy = std::abs(dy);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    double dist = std::hypot(dx, dy);
    if (dist > max_length) {
        double scale = max_length / dist;
        x1 = x0 + (int)(dx * scale);
        y1 = y0 + (int)(dy * scale);
        dx = x1 - x0;
        dy = y1 - y0;
        abs_dx = std::abs(dx);
        abs_dy = std::abs(dy);
    }
    int err = abs_dx - abs_dy;
    while (true) {
        setCost((unsigned int)x0, (unsigned int)y0, value);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;

        if (e2 > -abs_dy) {
            err -= abs_dy;
            x0 += sx;
        }
        
        if (e2 < abs_dx) {
            err += abs_dx;
            y0 += sy;
        }
    }
}

Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
                     double origin_x, double origin_y, unsigned char default_value) :
    size_x_(cells_size_x),
    size_y_(cells_size_y),
    resolution_(resolution),
    origin_x_(origin_x),
    origin_y_(origin_y),
    costmap_(NULL),
    default_value_(default_value)
{
    initMaps(size_x_, size_y_);
    resetFullMap();
}

Costmap2D::Costmap2D() :
    size_x_(0),
    size_y_(0),
    resolution_(0.0),
    origin_x_(0.0),
    origin_y_(0.0),
    costmap_(NULL) {}

Costmap2D::~Costmap2D() {
  deleteMaps();
}

void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y) {
    boost::recursive_mutex::scoped_lock lock(configuration_mutex_);
    delete [] costmap_;
    costmap_ = new unsigned char[size_x * size_y];
}

void Costmap2D::resetFullMap() {
    boost::recursive_mutex::scoped_lock lock(configuration_mutex_);
    memset(costmap_, default_value_, size_x_*size_y_*sizeof(unsigned char));
}

void Costmap2D::deleteMaps() {
    boost::recursive_mutex::scoped_lock lock(configuration_mutex_);
    delete [] costmap_;
    costmap_ = NULL;
}

void Costmap2D::resetRegionOfMap(unsigned int x0, unsigned int y0,
                                 unsigned int xn, unsigned int yn) {
    boost::recursive_mutex::scoped_lock lock(configuration_mutex_);
    unsigned int len = xn - x0;
    for(unsigned int idx = y0*size_x_ + x0; idx < yn*size_x_ + x0; idx += size_x_) {
        memset(costmap_ + idx, default_value_, len * sizeof(unsigned char));
    }
}

void Costmap2D::resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                          double origin_x, double origin_y) {
    size_x_ = size_x;
    size_y_ = size_y;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    initMaps(size_x, size_y);                        
    resetFullMap();
}

void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y) {
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - origin_x_) / resolution_);
    cell_oy = int((new_origin_y - origin_y_) / resolution_);
    if (cell_ox == 0 && cell_oy == 0) return;

    double new_grid_ox, new_grid_oy;
    new_grid_ox = origin_x_ + cell_ox * resolution_;
    new_grid_oy = origin_y_ + cell_oy * resolution_;

    int size_x = size_x_;
    int size_y = size_y_;
    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = std::min(std::max(cell_ox, 0), size_x);
    lower_left_y = std::min(std::max(cell_oy, 0), size_y);
    upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
    upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;

    unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
    // copy the local window in the costmap to the local map
    copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_,
                  local_map, 0, 0, cell_size_x,
                  cell_size_x, cell_size_y);
    resetFullMap();

    origin_x_ = new_grid_ox;
    origin_y_ = new_grid_oy;

    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    copyMapRegion(local_map, 0, 0, cell_size_x,
                  costmap_, start_x, start_y, size_x_,
                  cell_size_x, cell_size_y);
    delete [] local_map;
}

unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const {
    return  costmap_[getIndex(mx, my)];
}

void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost) {
    costmap_[getIndex(mx, my)] = cost;
}

void Costmap2D::mapToWorld(unsigned int mx, unsigned int my,
                           double& wx, double& wy) const {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;                        
}

bool Costmap2D::worldToMap(double wx, double wy,
                           unsigned int& mx, unsigned int& my) const {
    if(wx < origin_x_ || wy < origin_y_) return false;

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if(mx < size_x_ && my < size_y_) return true;

    return false;
}

void Costmap2D::worldToMapNoBounds(double wx, double wy, int& mx, int& my) const {
    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);
}

void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const {
    if(wx < origin_x_) { mx = 0; }
    else if(wx >= resolution_*size_x_ + origin_x_) { mx = size_x_ - 1; }
    else { mx = (int)((wx - origin_x_) / resolution_); }

    if(wy < origin_y_) { my = 0; }
    else if(wy >= resolution_*size_y_ + origin_y_) { my = size_y_ - 1; }
    else { my = (int)((wy - origin_y_) / resolution_); }
}

unsigned int Costmap2D::cellDistance(double world_dist) {
  double cells_dist = std::max(0.0, ceil(world_dist / resolution_));
  return (unsigned int)cells_dist;
}

bool Costmap2D::setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon,
                                     unsigned char cost_value) {
    std::vector<MapLocation> map_polygon;
    for(unsigned int i = 0; i < polygon.size(); i++) {
        MapLocation loc;
        // if(!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y)) { return false; }
        int mx, my;
        worldToMapEnforceBounds(polygon[i].x, polygon[i].y, mx, my);
        loc.x = mx; loc.y = my;
        map_polygon.push_back(loc);
    }

    std::vector<MapLocation> polygon_cells;
    convexFillCells(map_polygon, polygon_cells);
    for(unsigned int i = 0; i < polygon_cells.size(); i++) {
        unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
        costmap_[index] = cost_value;
    }                                       
    return true;
}

void Costmap2D::polygonOutlineCells(const std::vector<MapLocation>& polygon,
                                    std::vector<MapLocation>& polygon_cells) {
    for(unsigned int i = 0; i < polygon.size() - 1; i++) {
        bresenham2D(polygon_cells, polygon[i], polygon[i+1]);
    }
    if(!polygon.empty()) {
        unsigned int last_index = polygon.size() - 1;
        bresenham2D(polygon_cells, polygon[last_index], polygon[0]);
    }                                    
}

void Costmap2D::convexFillCells(const std::vector<MapLocation>& polygon,
                                std::vector<MapLocation>& polygon_cells) {
    if (polygon.size() < 3) return;

    polygonOutlineCells(polygon, polygon_cells);
    
    // Sort by x
    std::sort(polygon_cells.begin(), polygon_cells.end(),
              [](const MapLocation& a, const MapLocation& b) {
                  return (a.x < b.x) || (a.x == b.x && a.y < b.y);
              });
    unsigned int min_x = polygon_cells.front().x;
    unsigned int max_x = polygon_cells.back().x;

    std::vector<MapLocation> filled_cells;

    for (unsigned int x = min_x; x <= max_x; ++x) {
        std::vector<MapLocation> column_cells;
        for (const auto& cell : polygon_cells) {
            if (cell.x == x) {
                column_cells.push_back(cell);
            }
        }
        if (column_cells.empty()) continue;

        // Find min_pt and max_pt (by y)
        MapLocation min_pt = *std::min_element(column_cells.begin(), column_cells.end(),
                                               [](const MapLocation& a, const MapLocation& b) {
                                                   return a.y < b.y;
                                               });

        MapLocation max_pt = *std::max_element(column_cells.begin(), column_cells.end(),
                                               [](const MapLocation& a, const MapLocation& b) {
                                                   return a.y < b.y;
                                               });
        for (unsigned int y = min_pt.y; y <= max_pt.y; ++y) {
            filled_cells.push_back({x, y});
        }
    }
    polygon_cells.insert(polygon_cells.end(), filled_cells.begin(), filled_cells.end());
}


}   // vk_costmap_2d namespace