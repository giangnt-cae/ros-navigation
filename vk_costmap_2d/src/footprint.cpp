#include "vk_costmap_2d/footprint.hpp"

namespace vk_costmap_2d {

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1) {
    double a = pX - x0;
    double b = pY - y0;
    double c = x1 - x0;
    double d = y1 - y0;

    double param = (a*c + b*d) / (c*c + d*d);
    double xx, yy;
    if(param < 0) {
        xx = x0;
        yy = y0;
    }else if (param > 1){
        xx = x1;
        yy = y1;
    }else {
        xx = x0 + param * c;
        yy = y0 + param * d;
    }
    return distance(pX, pY, xx, yy);
}

bool intersectPointAndPolygon(std::vector<geometry_msgs::Point>& polygon, float x, float y) {
    bool chk = false;
    int i, j, nvert = polygon.size();
    for(i = 0, j = nvert - 1; i < nvert; j = i++) {
        float yi = polygon[i].y;
        float yj = polygon[j].y;
        float xi = polygon[i].x;
        float xj = polygon[j].x;
        if(((yi > y) != (yj > y)) && (x < (xj - xi)*(y - yi)/(yj - yi) + xi)) {
            chk = !chk;
        }
    }
    return chk;   
}

bool intersectPolygonAndPolygon(std::vector<geometry_msgs::Point>& polygon1,
                                std::vector<geometry_msgs::Point>& polygon2) {
    for(unsigned int i = 0; i < polygon1.size(); i++) {
        if(intersectPointAndPolygon(polygon2, polygon1[i].x, polygon1[i].y))
            return true;
    }                          
    for(unsigned int i = 0; i < polygon2.size(); i++) {
        if(intersectPointAndPolygon(polygon1, polygon2[i].x, polygon2[i].y))
            return true;
    }
    return false;  
}

void calculateMinAndMaxDistances(const std::vector<geometry_msgs::Point>& footprint,
                                 double& min_dist, double& max_dist) {
    min_dist = std::numeric_limits<double>::max();
    max_dist = 0.0;

    if(footprint.size() <= 2) return;
    for(unsigned int i = 0; i < footprint.size()-1; i++) {
        double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
        double edge_dist = distanceToLine(0.0, 0.0, footprint[i].x, footprint[i].y,
                                          footprint[i+1].x, footprint[i+1].y);
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
        max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
    }
    double vertex_dist = distance(0.0, 0.0, footprint.back().x, footprint.back().y);
    double edge_dist = distanceToLine(0.0, 0.0, footprint.back().x, footprint.back().y,
                                      footprint.front().x, footprint.front().y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
}

geometry_msgs::Point32 toPoint32(geometry_msgs::Point pt) {
  geometry_msgs::Point32 point32;
  point32.x = pt.x;
  point32.y = pt.y;
  point32.z = pt.z;
  return point32;
}

geometry_msgs::Point toPoint(geometry_msgs::Point32 pt) {
  geometry_msgs::Point point;
  point.x = pt.x;
  point.y = pt.y;
  point.z = pt.z;
  return point;
}

geometry_msgs::Polygon toPolygon(std::vector<geometry_msgs::Point> pts) {
  geometry_msgs::Polygon polygon;
  for (int i = 0; i < pts.size(); i++) {
    polygon.points.push_back(toPoint32(pts[i]));
  }
  return polygon;
}

std::vector<geometry_msgs::Point> toPointVector(geometry_msgs::Polygon polygon) {
  std::vector<geometry_msgs::Point> pts;
  for (int i = 0; i < polygon.points.size(); i++) {
    pts.push_back(toPoint(polygon.points[i]));
  }
  return pts;
}

void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        std::vector<geometry_msgs::Point>& oriented_footprint) {
  // Build the oriented footprint at a given location
  oriented_footprint.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::Point new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }
}

void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        geometry_msgs::PolygonStamped& oriented_footprint) {
  // Build the oriented footprint at a given location
  oriented_footprint.polygon.points.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::Point32 new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.polygon.points.push_back(new_pt);
  }
}

void padFootprint(std::vector<geometry_msgs::Point>& footprint, double padding) {
  for (unsigned int i = 0; i < footprint.size(); i++) {
    geometry_msgs::Point& pt = footprint[i];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}

void padFootprintX(std::vector<geometry_msgs::Point>& footprint, double padding) {
  for (unsigned int i = 0; i < footprint.size(); i++) {
    geometry_msgs::Point& pt = footprint[i];
    pt.x += sign0(pt.x) * padding;
  }
}

void padFootprintY(std::vector<geometry_msgs::Point>& footprint, double padding) {
  for (unsigned int i = 0; i < footprint.size(); i++) {
    geometry_msgs::Point& pt = footprint[i];
    pt.y += sign0(pt.y) * padding;
  }
}

std::vector<geometry_msgs::Point> makeFootprintFromRadius(double radius, unsigned int N) {
  std::vector<geometry_msgs::Point> points;
  // Loop over N angles around a circle making a point each time
  geometry_msgs::Point pt;
  for (int i = 0; i < N; ++i) {
    double angle = i * 2 * M_PI / N;
    pt.x = cos(angle) * radius;
    pt.y = sin(angle) * radius;
    points.push_back(pt);
  }
  return points;
}

std::vector<geometry_msgs::Point> makeFootprintFromWidthAndHeight(double width, double height) {
  std::vector<geometry_msgs::Point> points;
  geometry_msgs::Point pt;
  pt.x = width / 2.0; pt.y = height / 2.0;
  points.push_back(pt);

  pt.x = width / 2.0; pt.y = -height / 2.0;
  points.push_back(pt);

  pt.x = -width / 2.0; pt.y = -height / 2.0;
  points.push_back(pt);

  pt.x = -width / 2.0; pt.y = height / 2.0;
  points.push_back(pt);
  return points;
}

}   // namespace vk_costmap_2d 