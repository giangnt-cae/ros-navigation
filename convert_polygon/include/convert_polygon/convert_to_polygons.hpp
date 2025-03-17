#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <convert_polygon/ObstacleArrayMsg.h>
#include <convert_polygon/ObstacleMsg.h>


namespace convert_polygon {

static const unsigned char LETHAL_OBSTACLE = 100;

struct Costmap2D {
    unsigned int size_x;
    unsigned int size_y;
    double resolution;
    double origin_x;
    double origin_y;
    std::vector<unsigned char> data;

    double getSizeInMetersX() const { return (size_x - 1 + 0.5) * resolution; }
    double getSizeInMetersY() const { return (size_y - 1 + 0.5) * resolution; }
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
        wx = origin_x + (mx + 0.5) * resolution;
        wy = origin_y + (my + 0.5) * resolution;  
    }
};

template <typename Point1, typename Point2>  
inline double norm2d(const Point1& pt1, const Point2& pt2) {
    return std::sqrt(std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2));
}

template <typename Point1, typename Point2>  
inline bool isApprox2d(const Point1& pt1, const Point2& pt2, double threshold) {
  return ( std::abs(pt2.x-pt1.x)<threshold && std::abs(pt2.y-pt1.y)<threshold );
}

template <typename Point, typename LinePoint>
inline double computeSquaredDistanceToLineSegment(const Point& point, const LinePoint& line_start, const LinePoint& line_end, bool* is_inbetween=NULL)
{
    double dx = line_end.x - line_start.x;
    double dy = line_end.y - line_start.y;
    
    double length_sqr = dx*dx + dy*dy;
    double u = 0;

    if (length_sqr > 0)
        u = ((point.x - line_start.x) * dx + (point.y - line_start.y) * dy) / length_sqr;

    if (is_inbetween)
        *is_inbetween = (u >= 0 && u <= 1);
  
    if (u <= 0)
        return std::pow(point.x-line_start.x, 2) + std::pow(point.y-line_start.y, 2);
    
    if (u >= 1)
        return std::pow(point.x-line_end.x, 2) + std::pow(point.y-line_end.y, 2);
    
    return std::pow(point.x - (line_start.x + u*dx) ,2) + std::pow(point.y - (line_start.y + u*dy),2);
}

template <typename Point, typename LinePoint>
inline double computeDistanceToLineSegment(const Point& point, const LinePoint& line_start, const LinePoint& line_end, bool* is_inbetween=NULL) {
  return std::sqrt(computeSquaredDistanceToLineSegment(point, line_start, line_end, is_inbetween));
}

std::vector<geometry_msgs::Point32> douglasPeucker(std::vector<geometry_msgs::Point32>::iterator begin,
                                                   std::vector<geometry_msgs::Point32>::iterator end, double eps);

class ConvertToPolygonDBSConcaveHull {
    public:
        struct KeyPoint {
            KeyPoint() {}
            KeyPoint(double x_, double y_) : x(x_), y(y_) {}
            double x;
            double y;
            void toPointMsg(geometry_msgs::Point& point) const { point.x = x; point.y = y; point.z = 0; }
            void toPointMsg(geometry_msgs::Point32& point) const { point.x = x; point.y = y; point.z = 0; }
        };

        ConvertToPolygonDBSConcaveHull();

        ~ConvertToPolygonDBSConcaveHull() {}

        std::vector<KeyPoint> occupied_cells_;  // List of occupied cells in the current map
        
        std::vector<std::vector<int>> neighbor_lookup_;
        int neighbor_size_x_;
        int neighbor_size_y_;
        double offset_x_;
        double offset_y_;

        template< typename Point>
        static void convertPointToPolygon(const Point& point, geometry_msgs::Polygon& polygon) {
            polygon.points.resize(1);
            polygon.points.front().x = point.x;
            polygon.points.front().y = point.y;
            polygon.points.front().z = 0;
        }

        template <typename P1, typename P2, typename P3>
        long double crossProduct(const P1& O, const P2& A, const P3& B) { 
            return (long double)(A.x - O.x) * (long double)(B.y - O.y) - (long double)(A.y - O.y) * (long double)(B.x - O.x);
        }


        template <typename PointLine, typename PointCluster, typename PointHull>
        std::size_t findNearestInnerPoint(PointLine line_start, PointLine line_end, const std::vector<PointCluster>& cluster, const std::vector<PointHull>& hull, bool* found);
        
        template <typename Point1, typename Point2, typename Point3, typename Point4>
        bool checkLineIntersection(const Point1& line1_start, const Point2& line1_end, const Point3& line2_start, const Point4& line2_end);
    
        template <typename PointHull, typename Point1, typename Point2, typename Point3, typename Point4>
        bool checkLineIntersection(const std::vector<PointHull>& hull, const Point1& current_line_start, 
                               const Point2& current_line_end, const Point3& test_line_start, const Point4& test_line_end);
        
    private:
        double cluster_max_distance_;           // maximum distance to neighbors [m]
        int cluster_min_pts_;                   // minimum number of points that define a cluster
        int cluster_max_pts_;                   // maximum number of points that define a cluster
        double convex_hull_min_pt_separation_;  // Clear keypoints/vertices of the convex polygon
                                                //  that are close to each other [distance in meters] (0: keep all)
        double concave_hull_depth_;
        std::string global_frame_;

        ros::NodeHandle nh_;
        ros::Subscriber local_map_sub_;
        ros::Publisher markers_pub_, obstacle_pub_;

        void init();

        void convert();

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

        void clusteringDBSCAN(std::vector<std::vector<KeyPoint>>& clusters);

        void findNeighbors(int current_index, std::vector<int>& neighbor_indices);

        // Monotone Chain Algorithm (Andrew’s Algorithm)
        void convexHull(std::vector<KeyPoint>& cluster, geometry_msgs::Polygon& polygon);

        void concaveHull(std::vector<KeyPoint>& cluster, double depth, geometry_msgs::Polygon& polygon);

        void simplifyPolygon(geometry_msgs::Polygon& polygon);

        void updateObstacles();

        int neighborCellsToIndex(int cx, int cy);
        void pointToNeighborCells(const KeyPoint& kp, int& cx, int& cy);
        void computeCentroidAndRadius(geometry_msgs::Polygon& polygon, geometry_msgs::Point32& centroid, double& radius);

        Costmap2D* costmap_;
        std::mutex mutex_;


};  // ConvertToPolygonDBSConcaveHull


template <typename PointLine, typename PointCluster, typename PointHull>
std::size_t ConvertToPolygonDBSConcaveHull::findNearestInnerPoint(PointLine line_start, PointLine line_end, const std::vector<PointCluster>& cluster, const std::vector<PointHull>& hull, bool* found) {
    std::size_t nearsest_idx = 0;
    double distance = 0;
    *found = false;

    for (std::size_t i = 0; i < cluster.size(); ++i) {
        // Skip points that are already in the hull
        if (std::find_if( hull.begin(), hull.end(), boost::bind(isApprox2d<PointHull, PointCluster>, _1, boost::cref(cluster[i]), 1e-5) ) != hull.end() )
            continue;

        double dist = computeDistanceToLineSegment(cluster[i], line_start, line_end);
        bool skip = false;
        for (int j = 0; !skip && j < (int)hull.size() - 1; ++j) {
            double dist_temp = computeDistanceToLineSegment(cluster[i], hull[j], hull[j + 1]);
            skip |= dist_temp < dist;
        }
        if (skip) 
            continue;

        if (!(*found) || dist < distance) {
            nearsest_idx = i;
            distance = dist;
            *found = true;
        }
    }
    return nearsest_idx;
}

template <typename Point1, typename Point2, typename Point3, typename Point4>
bool ConvertToPolygonDBSConcaveHull::checkLineIntersection(const Point1& line1_start, const Point2& line1_end, const Point3& line2_start, const Point4& line2_end)
{
    double dx1 = line1_end.x - line1_start.x;
    double dy1 = line1_end.y - line1_start.y;
    double dx2 = line2_end.x - line2_start.x;
    double dy2 = line2_end.y - line2_start.y;
    double s = (-dy1 * (line1_start.x - line2_start.x) + dx1 * (line1_start.y - line2_start.y)) / (-dx2 * dy1 + dx1 * dy2);
    double t = ( dx2 * (line1_start.y - line2_start.y) - dy2 * (line1_start.x - line2_start.x)) / (-dx2 * dy1 + dx1 * dy2);
    return (s > 0 && s < 1 && t > 0 && t < 1);
}

template <typename PointHull, typename Point1, typename Point2, typename Point3, typename Point4>
bool ConvertToPolygonDBSConcaveHull::checkLineIntersection(const std::vector<PointHull>& hull, const Point1& current_line_start, 
                                                            const Point2& current_line_end, const Point3& test_line_start, const Point4& test_line_end)
{
    for (int i = 0; i < (int)hull.size() - 2; i++) 
    {
        if (isApprox2d(current_line_start, hull[i], 1e-5) && isApprox2d(current_line_end, hull[i+1], 1e-5))
        {
            continue;
        }

        if (checkLineIntersection(test_line_start, test_line_end, hull[i], hull[i+1])) 
        {
            return true;
        }
    }
    return false;
}

}   // convert_polygon namespace
