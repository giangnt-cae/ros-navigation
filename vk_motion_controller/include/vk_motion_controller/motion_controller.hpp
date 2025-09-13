#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <message_filters/subscriber.h>

#include <boost/thread.hpp>

#include <casadi/casadi.hpp>

#include <vk_motion_controller/trajectory_generation.hpp>
#include <convert_polygon/ObstacleMsg.h>
#include <convert_polygon/ObstacleArrayMsg.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#define MaxVerticesFilter
#define TimeOptimization
#define PublishVelocityCommand
#define PublishLocalPlanner
#define PublishReferenceTrajectory

using namespace casadi;

template <typename T>
inline T diffStateVector(const T& x1, const T& x2) {
    T e = x1 - x2;
    T a = atan2(sin(x1(2)), cos(x1(2)));
    T b = atan2(sin(x2(2)), cos(x2(2)));
    
    T d1 = a - b;
    T d2 = 2 * M_PI - fabs(d1);
    
    d2 = if_else(d1 > 0, -d2, d2);
    e(2) = if_else(fabs(d1) < fabs(d2), d1, d2);
    return e;
}

template <typename Point1, typename Point2>  
inline double norm2d(const Point1& pt1, const Point2& pt2) {
    return std::sqrt(std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2));
}

inline double sign0(double x) {
    return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

class MotionController {
    public:
        MotionController(tf2_ros::Buffer& tf);

        struct Polygon {
            std::vector<geometry_msgs::Point32> points;
            geometry_msgs::Point32 centroid;
            double radius;

            geometry_msgs::Point32 getMidPoint(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2) {
                geometry_msgs::Point32 midpoint;
                midpoint.x = (p1.x + p2.x) * 0.5;
                midpoint.y = (p1.y + p2.y) * 0.5;
                return midpoint;
            }

            // If the number of vertices of the polygon is less than max_vertices, add the last vertex 
            void addPoints(int max_points) {
                if(points.empty()) return;
                while((int)points.size() < max_points) {
                    geometry_msgs::Point32 midpoint = getMidPoint(points.front(),points.back());
                    points.push_back(midpoint);
                }
            }

            // If the number of polygon vertices is greater than max_vertices, 
            // select the max_vertices closest vertices to the robot
            void downPoints(int max_points, const double (&robot_pose)[3]) {
                if(points.empty() || (int)points.size() <= max_points) return;
                std::vector<int> indexs(points.size());
                std::iota(indexs.begin(), indexs.end(), 0);
                geometry_msgs::Point temp;
                temp.x = robot_pose[0]; temp.y = robot_pose[1];
                std::nth_element(indexs.begin(), indexs.begin() + max_points, indexs.end(), [&](int i, int j) {
                        return norm2d(points[i], temp) < norm2d(points[j], temp); });
                
                        indexs.resize(max_points);
                std::sort(indexs.begin(), indexs.end());
                
                std::vector<geometry_msgs::Point32> new_points;
                new_points.reserve(max_points);
    
                for (int i : indexs) {
                    new_points.push_back(points[i]);
                }
                points = std::move(new_points);
            }

            void filter(int max_points, const double (&robot_pose)[3]) {
                addPoints(max_points);
                downPoints(max_points, robot_pose);
            }
        };

        ~MotionController() {};

    private:
        casadi::SX X;   // State [num_states_ x (N_horizon_ + 1) ] -- x, y, theta
        casadi::SX U;   // Velocity [num_controls_ x N_horizon_ ] -- v, w
        casadi::SX P;   // Parameters [num_states_ * (N_horizon_ + 1) + num_controls_ * N_horizon_ + 2 * N_maxobs_ * (N_maxvertices_ + 1)]

        const int num_states_ = 3;
        const int num_controls_ = 2;
        int N_horizon_, N_controls_, N_maxobs_, N_maxvertices_;
        double T_s_;
        std::vector<geometry_msgs::Point> unpadded_footprint_, padded_footprint_;
        std::vector<Polygon> obstacles_;
        double min_obstacle_distance_, cir_radius_;
        double max_error_position_, max_error_angle_;
        double width_lane_;
        double width_map_, height_map_;
        double transform_tolerance_;
        int sx_, sy_;
        float footprint_padding_X_, footprint_padding_Y_;
        double eps_x_, eps_y_, eps_theta_;

        casadi::DM Q_;      // (x - x_ref)^T * Q_ * (x - x_ref)
        casadi::DM R_;      // (u - u_ref)^T * R_ * (u - u_ref)
        casadi::DM W_;      // (u_k+1 - u_k)^T * W_ * (u_k+1 - u_k)
        casadi::DM Q_N_;    // (x_N - x_Nref)^T * Q_N_ * (x_N - x_Nref)
        std::vector<double> Qdiag_, Rdiag_, Wdiag_, Q_Ndiag_;

        casadi::SX obj_, cst_;
        casadi::DMDict args_;
        casadi::Dict opts_;
        casadi::SXDict nlp_;

        void init();
        void run();
        casadi::SX setObjective();
        casadi::SX setStateConstraints();
        void setCollisionConstraints(casadi::SX& cst);
        void setKeepLaneConstraints(casadi::SX& cst);

        ros::NodeHandle nh_, private_nh_;
        ros::Subscriber obstacle_sub_;
        ros::Publisher vel_pub_, local_planner_pub_, reference_pub_;

        ros::Subscriber stop_signal_sub_;
        ros::Publisher communication_pub_;

        std::string global_frame_, base_frame_;
        bool getRobotPose(double (&robot_pose)[3]);

        void obstacleCallback(const convert_polygon::ObstacleArrayMsg& msg);

        void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);

        void stopSignalCallback(const std_msgs::String& msg);

        // Filtering to get N_maxobs_
        void filterObstacles(double (&robot_pose)[3]);

        bool loadFootprintFromParam(const std::string& param_name, std::vector<geometry_msgs::Point>& footprint);

        std::vector<geometry_msgs::Point> deCompositionFootprint(std::vector<geometry_msgs::Point>& footprint, int sx, int sy);

        // Given a pose and base footprint, build the oriented footprint of the robot
        void transformFootprint(const casadi::SX& x, casadi::SX& oriented_footprint,
                                const std::vector<geometry_msgs::Point>& footprint);
        
        void padFootprintX(std::vector<geometry_msgs::Point>& footprint, double padding);
        void padFootprintY(std::vector<geometry_msgs::Point>& footprint, double padding);

        bool checkIsGoal(double (&robot_pose)[3]);
        
        bool ctrl_, has_collision_;
        bool updated_obstacles_;
        bool avoidance_enable_, keep_lane_;
        std::mutex obstacles_mutex_;
        tf2_ros::Buffer& tf_;

        Trajectory* ref_traj_;
        laser_geometry::LaserProjection projector_;
        
        boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_scan_sub_;
        boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> filter_;
};