#include <convert_polygon/convert_to_polygons.hpp>

namespace convert_polygon {

ConvertToPolygonDBSConcaveHull::ConvertToPolygonDBSConcaveHull() {
    costmap_ = NULL;
    neighbor_size_x_ = neighbor_size_y_ = -1;
    offset_x_ = offset_y_ = 0.0;
    init();
}

void ConvertToPolygonDBSConcaveHull::init() {
    costmap_ = NULL;
    nh_.param("cluster_max_distance", cluster_max_distance_, 0.5);
    nh_.param("cluster_min_pts", cluster_min_pts_, 3);
    nh_.param("cluster_max_pts_", cluster_max_pts_, 30);
    nh_.param("convex_hull_min_pt_separation", convex_hull_min_pt_separation_, 0.05);
    nh_.param("global_frame", global_frame_, std::string("map"));
    nh_.param("concave_hull_depth", concave_hull_depth_, 2.0);

    local_map_sub_ = nh_.subscribe("vk_costmap_2d/costmap", 5, &ConvertToPolygonDBSConcaveHull::mapCallback, this);

    obstacle_pub_  = nh_.advertise<convert_polygon::ObstacleArrayMsg>("obstacles", 100);

    markers_pub_   = nh_.advertise<visualization_msgs::MarkerArray>("polygons", 10);

    ros::spin();
}

void ConvertToPolygonDBSConcaveHull::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if(costmap_ == NULL)
        costmap_ = new Costmap2D;
    costmap_->size_x = msg->info.width;
    costmap_->size_y = msg->info.height;
    costmap_->resolution = msg->info.resolution;
    costmap_->origin_x = msg->info.origin.position.x;
    costmap_->origin_y = msg->info.origin.position.y;
    costmap_->data.resize(costmap_->size_x * costmap_->size_y);

    occupied_cells_.clear();
    // Allocate neighbor lookup
    int cells_x = (int)(costmap_->getSizeInMetersX() / cluster_max_distance_) + 1;
    int cells_y = (int)(costmap_->getSizeInMetersY() / cluster_max_distance_) + 1;

    if(cells_x != neighbor_size_x_ || cells_y != neighbor_size_y_) {
        neighbor_size_x_ = cells_x;
        neighbor_size_y_ = cells_y;
        neighbor_lookup_.resize(neighbor_size_x_ * neighbor_size_y_);
    }
    offset_x_ = costmap_->origin_x;
    offset_y_ = costmap_->origin_y;
    for (auto& n : neighbor_lookup_)
        n.clear();

    for(int i = 0; i < costmap_->size_x; i++) {
        for(int j = 0; j < costmap_->size_y; j++) {
            int map_index = i + j * costmap_->size_x;
            costmap_->data[map_index] = msg->data[map_index];
            if(costmap_->data[map_index] == LETHAL_OBSTACLE) {
                double x, y;
                costmap_->mapToWorld((unsigned int)i, (unsigned int)j, x, y);
                int idx = occupied_cells_.size();
                occupied_cells_.emplace_back(x, y);
                int cx, cy;
                pointToNeighborCells(occupied_cells_.back(), cx, cy);
                int nidx = neighborCellsToIndex(cx, cy);
                if(nidx >= 0)
                    neighbor_lookup_[nidx].push_back(idx);
            }
        }
    }

    updateObstacles();
}

void ConvertToPolygonDBSConcaveHull::findNeighbors(int current_index, std::vector<int>& neighbor_indices) {
    neighbor_indices.clear();
    double dist_square = cluster_max_distance_ * cluster_max_distance_;
    const KeyPoint& kp = occupied_cells_[current_index];
    int cx, cy;
    pointToNeighborCells(kp, cx, cy);
    const int kernel[9][2] = {{-1, -1}, {0, -1}, {1, -1},
                              {-1,  0}, {0,  0}, {1,  0},
                              {-1,  1}, {0,  1}, {1,  1}};
    for(int i = 0; i < 9; i++) {
        int idx = neighborCellsToIndex(cx + kernel[i][0], cy + kernel[i][1]);
        if(idx < 0 || idx >= int(neighbor_lookup_.size()))
            continue;
        const std::vector<int>& indexs = neighbor_lookup_[idx];
        for(int point_idx : indexs) {
            if(point_idx == current_index)
                continue;
            const KeyPoint& other = occupied_cells_[point_idx];
            double dx = other.x - kp.x;
            double dy = other.y - kp.y;
            if(dx*dx + dy*dy <= dist_square)
                neighbor_indices.push_back(point_idx);
        }
    }
}

void ConvertToPolygonDBSConcaveHull::clusteringDBSCAN(std::vector<std::vector<KeyPoint>>& clusters) {
    std::vector<bool> visited(occupied_cells_.size(), false);

    clusters.clear();
    int cluster_id = 0; // current cluster_id
    clusters.push_back(std::vector<KeyPoint>());
    for(int i = 0; i < (int)occupied_cells_.size(); i++) {
        if(!visited[i]) {
            visited[i] = true;
            std::vector<int> neighbor_indices;
            findNeighbors(i, neighbor_indices);
            if((int)neighbor_indices.size() < cluster_min_pts_) {
                clusters[0].push_back(occupied_cells_[i]);
            }else {
                ++cluster_id;
                clusters.push_back(std::vector<KeyPoint>());

                clusters[cluster_id].push_back(occupied_cells_[i]);
                for(int j = 0; j < (int)neighbor_indices.size(); j++) {
                    if((int)clusters[cluster_id].size() == cluster_max_pts_)
                        break;
                    if(!visited[neighbor_indices[j]]) {
                        visited[neighbor_indices[j]] = true;
                        std::vector<int> further_neighbor_indices;
                        findNeighbors(neighbor_indices[j], further_neighbor_indices);
                        if((int)further_neighbor_indices.size() >= cluster_min_pts_) {
                            neighbor_indices.insert(neighbor_indices.end(), further_neighbor_indices.begin(), further_neighbor_indices.end());
                            clusters[cluster_id].push_back(occupied_cells_[neighbor_indices[j]]);
                        }
                    }
                }
            }
        }
    }
}

void ConvertToPolygonDBSConcaveHull::updateObstacles() {
    std::vector<std::vector<KeyPoint>> clusters;
    clusters.clear();
    clusteringDBSCAN(clusters);
    if((int)clusters.size() == 0)
        return;
    visualization_msgs::MarkerArray vs_markers;
    vs_markers.markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "Polygons";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.01;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.5);
    marker.frame_locked = true;

    convert_polygon::ObstacleArrayMsg obstacles_;
    obstacles_.obstacles.clear();
    obstacles_.header.frame_id = global_frame_;
    obstacles_.header.stamp = ros::Time::now();
    convert_polygon::ObstacleMsg obstacle;
    geometry_msgs::Point point;
    geometry_msgs::Polygon polygon;
    marker.id = 0;
    clusters.erase(clusters.begin());       // First cluster is only noises
    for(auto& cluster : clusters) {
        if((int)cluster.size() < 2)
            continue;
        polygon.points.clear();
        marker.points.clear();
    
        convexHull(cluster, polygon);       // Closed polygon

        // concaveHull(cluster, concave_hull_depth_, polygon);
        
        obstacle.polygon = polygon;
        computeCentroidAndRadius(polygon, obstacle.centroid, obstacle.radius);
        obstacle.id      = marker.id;
        obstacles_.obstacles.push_back(obstacle);
        /*-----------------------------------*/
        for(auto& vertice : polygon.points) {
            point.x = vertice.x;
            point.y = vertice.y;
            marker.points.push_back(point);
        }
        vs_markers.markers.push_back(marker);

        /*-----------------------------------*/
        ++marker.id;
    }
    obstacle_pub_.publish(obstacles_);
    markers_pub_.publish(vs_markers);
}

int ConvertToPolygonDBSConcaveHull::neighborCellsToIndex(int cx, int cy) {
    if(cx , 0 || cx >= neighbor_size_x_ || cy < 0 || cy >= neighbor_size_y_)
        return -1;
    return cy * neighbor_size_x_ + cx;
}

void ConvertToPolygonDBSConcaveHull::pointToNeighborCells(const KeyPoint& kp, int& cx, int& cy) {
    cx = int((kp.x - offset_x_) / cluster_max_distance_);
    cy = int((kp.y - offset_y_) / cluster_max_distance_);
}

bool isXCoordinateSmaller(const ConvertToPolygonDBSConcaveHull::KeyPoint& p1, const ConvertToPolygonDBSConcaveHull::KeyPoint& p2) {
    return (p1.x < p2.x) || (p1.x == p2.x && p1.y < p2.y);                         
}

void ConvertToPolygonDBSConcaveHull::convexHull(std::vector<KeyPoint>& cluster, geometry_msgs::Polygon& polygon) {
    std::vector<geometry_msgs::Point32>& points = polygon.points;
    // Sort cluster by x and y
    std::sort(cluster.begin(), cluster.end(), isXCoordinateSmaller);
    
    // Get the indices of points with min x-coord and min|max y-coord
    int i;
    int minmin = 0, minmax;
    double xmin = cluster[0].x;
    for(i = 1; i < (int)cluster.size(); i++) {
        if(cluster[i].x != xmin) break;
    }
    minmax = i - 1;     // [xmin, ymax]

    if(minmax == (int)cluster.size() - 1) { // degenerate case: all x-coords == xmin
        points.push_back(geometry_msgs::Point32());
        cluster[minmin].toPointMsg(points.back());
        if(cluster[minmax].y != cluster[minmin].y) {
            points.push_back(geometry_msgs::Point32());
            cluster[minmax].toPointMsg(points.back());
        }
        // add polygon endpoint
        points.push_back(geometry_msgs::Point32());
        cluster[minmin].toPointMsg(points.back());
        return;
    }
    
    // Get the indices of points with max x-coord and min|max y-coord
    int maxmin, maxmax = (int)cluster.size() - 1;
    double xmax = cluster.back().x;
    for (i = cluster.size() - 2; i >= 0; i--)
        if (cluster[i].x != xmax) break;
    maxmin = i+1;
    
    // Compute the lower hull
    // push  minmin point onto stack
    points.push_back(geometry_msgs::Point32());
    cluster[minmin].toPointMsg(points.back());
    i = minmax;
    while(++i <= maxmin) {
        if(crossProduct(cluster[minmin], cluster[maxmin], cluster[i]) >= 0 && i < maxmin)
            continue;
        
        while(points.size() > 1) {
            if(crossProduct(points[points.size() - 2], points.back(), cluster[i]) > 0)
                break;
            points.pop_back();
        }
        points.push_back(geometry_msgs::Point32());
        cluster[i].toPointMsg(points.back());
    }

    // Next, compute the upper hull
    if(maxmax != maxmin) {
        points.push_back(geometry_msgs::Point32());
        cluster[maxmax].toPointMsg(points.back());
    }

    int bot = (int)points.size();   // the bottom point of the upper hull stack
    i = maxmin;
    while(--i >= minmax) {
        if(crossProduct(cluster[maxmax], cluster[minmax], cluster[i]) >= 0 && i > minmax)
            continue;
        while((int)points.size() > bot) {
            if(crossProduct(points[points.size() - 2], points.back(), cluster[i]) > 0)
                break;
            points.pop_back();
        }
        points.push_back(geometry_msgs::Point32());
        cluster[i].toPointMsg(points.back());
    }
    
    if (minmax != minmin) {
        points.push_back(geometry_msgs::Point32());
        cluster[minmin].toPointMsg(points.back());
    }

    simplifyPolygon(polygon);
}

void ConvertToPolygonDBSConcaveHull::concaveHull(std::vector<KeyPoint>& cluster, double depth, geometry_msgs::Polygon& polygon) {
    convexHull(cluster, polygon);

    std::vector<geometry_msgs::Point32>& concave_list = polygon.points;

    for (int i = 0; i < (int)concave_list.size() - 1; ++i) {
        // find nearest inner point pk from line (vertex1 -> vertex2)
        const geometry_msgs::Point32& vertex1 = concave_list[i];
        const geometry_msgs::Point32& vertex2 = concave_list[i+1];

        bool found;
        size_t nearest_idx = findNearestInnerPoint(vertex1, vertex2, cluster, concave_list, &found);
        if (!found) 
          continue;  
        
        double line_length = norm2d(vertex1, vertex2);
                
        double dst1 = norm2d(cluster[nearest_idx], vertex1);
        double dst2 = norm2d(cluster[nearest_idx], vertex2);
        double dd = std::min(dst1, dst2);
        if (dd<1e-8)
          continue;

        if (line_length / dd > depth) {
            // Check that new candidate edge will not intersect existing edges.
            bool intersects = checkLineIntersection(concave_list, vertex1, vertex2, vertex1, cluster[nearest_idx]);
            intersects |= checkLineIntersection(concave_list, vertex1, vertex2, cluster[nearest_idx], vertex2);
            if (!intersects) {
              geometry_msgs::Point32 new_point;
              cluster[nearest_idx].toPointMsg(new_point);
              concave_list.insert(concave_list.begin() + i + 1, new_point);
              i--;
            }
        }
    }
}

void ConvertToPolygonDBSConcaveHull::simplifyPolygon(geometry_msgs::Polygon& polygon) {
    int triangleThreshold = 3;
    if(polygon.points.size() > 1
       && std::abs(polygon.points.front().x - polygon.points.back().x) < 1e-5
       && std::abs(polygon.points.front().y - polygon.points.back().y) < 1e-5)
    {
        triangleThreshold = 4;
    }
    if (polygon.points.size() <= triangleThreshold)
        return;
    polygon.points = douglasPeucker(polygon.points.begin(), polygon.points.end(), convex_hull_min_pt_separation_);;
}

std::vector<geometry_msgs::Point32> douglasPeucker(std::vector<geometry_msgs::Point32>::iterator begin,
                                                   std::vector<geometry_msgs::Point32>::iterator end,
                                                   double eps)
{
    if (std::distance(begin, end) <= 2) {
        return std::vector<geometry_msgs::Point32>(begin, end);
    }
    // Find the point with the maximum distance from the line [begin, end)
    double dmax = std::numeric_limits<double>::lowest();
    std::vector<geometry_msgs::Point32>::iterator max_dist_it;
    std::vector<geometry_msgs::Point32>::iterator last = std::prev(end);
    for(auto it = std::next(begin); it != last; ++it) {
        double dist_square = convert_polygon::computeSquaredDistanceToLineSegment(*it, *begin, *last);
        if(dist_square > dmax) {
            max_dist_it = it;
            dmax = dist_square;
        }
    }

    if(dmax < eps * eps) {
        std::vector<geometry_msgs::Point32> result;
        result.push_back(*begin);
        result.push_back(*last);
        return result;
    }

    auto firstLineSimplified = douglasPeucker(begin, std::next(max_dist_it), eps);
    auto secondLineSimplified = douglasPeucker(max_dist_it, end, eps);
    // Combine the two lines into one line and return the merged line.
    // Note that we have to skip the first point of the second line, as it is duplicated above.
    firstLineSimplified.insert(firstLineSimplified.end(),
                               std::make_move_iterator(std::next(secondLineSimplified.begin())),
                               std::make_move_iterator(secondLineSimplified.end()));
    return firstLineSimplified;
}

void ConvertToPolygonDBSConcaveHull::computeCentroidAndRadius(geometry_msgs::Polygon& polygon, geometry_msgs::Point32& centroid, double& radius) {
    double area_threshold = 1e-4;
    if((int)polygon.points.size() == 2) {   // Is a point
        centroid = polygon.points[0];
        radius     = 0.0;
        return;
    }

    if((int)polygon.points.size() == 3) {   // Is a line
        centroid.x = (polygon.points[0].x + polygon.points[1].x) / 2.0;
        centroid.y = (polygon.points[0].y + polygon.points[1].y) / 2.0;
        radius     = std::hypot(polygon.points[0].x - centroid.x, polygon.points[0].y - centroid.y);
        return;
    }

    double cx = 0.0, cy = 0.0;
    double area = 0.0;
    int n = polygon.points.size();
    for(int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        area += (polygon.points[i].x * polygon.points[j].y) - (polygon.points[j].x * polygon.points[i].y);
    }
    area = 0.5 * std::abs(area);
    
    double max_edge = 0.0;
    int imax, jmax;
    if(area < area_threshold) {
        for(int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double d = std::pow(polygon.points[i].x - polygon.points[j].x, 2) +
                       std::pow(polygon.points[i].y - polygon.points[j].y, 2);
            if(d > max_edge) {
                max_edge = d;
                imax = i;
                jmax = j;
            }
        }
        centroid.x = (polygon.points[imax].x + polygon.points[jmax].x) / 2.0;
        centroid.y = (polygon.points[imax].y + polygon.points[jmax].y) / 2.0;
        radius     = std::sqrt(max_edge);
        return;
    }

    // Compute centroid
    for(int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        double common = (polygon.points[i].x * polygon.points[j].y) - (polygon.points[j].x * polygon.points[i].y);
        cx += (polygon.points[i].x + polygon.points[j].x) * common;
        cy += (polygon.points[i].y + polygon.points[j].y) * common;
    }
    cx /= (6 * area);
    cy /= (6 * area);

    double max_dist = 0.0;
    for (const auto& point : polygon.points) {
        max_dist = std::max(max_dist, std::hypot(point.x - cx, point.y - cy));
    }
    centroid.x = cx;
    centroid.y = cy;
    radius     = max_dist;
}

}   // convert_polygon namespace

