#include <vk_motion_controller/quintic_bezier_splines.hpp>

void QuinticBezierSpline::reset() {
    std::vector<Eigen::Vector2d>().swap(waypoints_);
    std::vector<double>().swap(waypoint_orientations_);
    std::vector<std::array<Eigen::Vector2d, 6>>().swap(segments_);
    std::vector<double>().swap(arc_lengths_);
}

bool QuinticBezierSpline::setWaypoints(const nav_msgs::Path& path) {
    if(!path.poses.empty()) {
        reset();
        num_waypoints_ = path.poses.size();
        waypoints_.emplace_back(path.poses[0].pose.position.x,
                                path.poses[0].pose.position.y);
        waypoint_orientations_.emplace_back(tf2::getYaw(path.poses[0].pose.orientation));

        double epsilon = 0.002;
        for (unsigned int i = 1; i < num_waypoints_ - 1; i++) {
            double x_prev = path.poses[i - 1].pose.position.x;
            double y_prev = path.poses[i - 1].pose.position.y;
            double x_curr = path.poses[i].pose.position.x;
            double y_curr = path.poses[i].pose.position.y;
            double x_next = path.poses[i + 1].pose.position.x;
            double y_next = path.poses[i + 1].pose.position.y;

            double vx_in = x_curr - x_prev;
            double vy_in = y_curr - y_prev;
            double len_in = std::hypot(vx_in, vy_in);
            vx_in /= len_in; vy_in /= len_in;

            double vx_out = x_next - x_curr;
            double vy_out = y_next - y_curr;
            double len_out = std::hypot(vx_out, vy_out);
            vx_out /= len_out; vy_out /= len_out;

            double x_rs = x_curr - epsilon * vx_in;
            double y_rs = y_curr - epsilon * vy_in;

            double x_re = x_curr + epsilon * vx_out;
            double y_re = y_curr + epsilon * vy_out;

            waypoints_.emplace_back(x_rs, y_rs);
            waypoints_.emplace_back(x_curr, y_curr);
            waypoints_.emplace_back(x_re, y_re);

            double theta = tf2::getYaw(path.poses[i].pose.orientation);
            waypoint_orientations_.emplace_back(theta);
        }

        waypoints_.emplace_back(path.poses[num_waypoints_ - 1].pose.position.x,
                                path.poses[num_waypoints_ - 1].pose.position.y);
        waypoint_orientations_.emplace_back(tf2::getYaw(path.poses[num_waypoints_ - 1].pose.orientation));
        num_waypoints_ = waypoints_.size();
        theta_start_ = tf2::getYaw(path.poses.front().pose.orientation);
        theta_goal_  = tf2::getYaw(path.poses.back().pose.orientation);
        return true;
    }
    return false;
}

std::vector<double> QuinticBezierSpline::computeDistanceOfSegments() {
    std::vector<double> distanceOfSegments;
    for(unsigned int i = 0; i < num_waypoints_ - 1; i++) {
        double dist = (waypoints_[i] - waypoints_[i+1]).norm();
        distanceOfSegments.push_back(dist);
    }
    return distanceOfSegments;
}

std::vector<Eigen::Vector2d> QuinticBezierSpline::computeFirstDerivativeHeuristics(std::vector<double>& distanceOfSegments) {
    std::vector<Eigen::Vector2d> first_derivatives(num_waypoints_);
    if(!use_orientation_robot_) {
        // First waypoint
        first_derivatives[0] = scalingCoefficient_ * (waypoints_[1] - waypoints_[0]);

        // Last waypoint
        first_derivatives[num_waypoints_-1] = scalingCoefficient_ * (waypoints_[num_waypoints_-1] - waypoints_[num_waypoints_-2]);
    }else {
        first_derivatives[0] = { cos(theta_start_), sin(theta_start_) };
        
        first_derivatives[num_waypoints_-1] = { cos(theta_goal_), sin(theta_goal_) };
    }

    // Inner waypoints
    for(unsigned int i = 1; i < num_waypoints_ - 1; i++) {
        if(distanceOfSegments[i-1] < 1e-3 && distanceOfSegments[i] < 1e-3) {
            first_derivatives[i] = Eigen::Vector2d::Zero();
            continue;
        }

        Eigen::Vector2d n1 = (waypoints_[i] - waypoints_[i-1]) / std::max(1e-3, distanceOfSegments[i-1]);
        Eigen::Vector2d n2 = (waypoints_[i+1] - waypoints_[i]) / std::max(1e-3, distanceOfSegments[i]);
        Eigen::Vector2d n = (n1 + n2).normalized();
        first_derivatives[i] = scalingCoefficient_ * std::min(distanceOfSegments[i-1], distanceOfSegments[i]) * n;
    }
    return first_derivatives;
}

Eigen::Vector2d QuinticBezierSpline::computeSecondDerivativeAtEndPoint(Eigen::Vector2d& A, Eigen::Vector2d& tA,
                                                                       Eigen::Vector2d& B, Eigen::Vector2d& tB) {
    return (6*A + 2*tA + 4*tB - 6*B);                                                                   
}

Eigen::Vector2d QuinticBezierSpline::computeSecondDerivativeAtBeginPoint(Eigen::Vector2d& B, Eigen::Vector2d& tB,
                                                                         Eigen::Vector2d& C, Eigen::Vector2d& tC) {
    return (-6*B - 4*tB - 2*tC + 6*C);                                                                   
}

std::vector<Eigen::Vector2d> QuinticBezierSpline::computeSecondDerivativeHeuristics(std::vector<double>& distanceOfSegments,
                                                                                    std::vector<Eigen::Vector2d>& first_derivatives) {
    std::vector<Eigen::Vector2d> second_derivatives(num_waypoints_);
    // First waypoint
    second_derivatives[0] = computeSecondDerivativeAtBeginPoint(waypoints_[0], first_derivatives[0],
                                                                waypoints_[1], first_derivatives[1]);

    // Last waypoint
    second_derivatives[num_waypoints_-1] = computeSecondDerivativeAtEndPoint(waypoints_[num_waypoints_-2], first_derivatives[num_waypoints_-2],
                                                                             waypoints_[num_waypoints_-1], first_derivatives[num_waypoints_-1]);

    // Inner waypoints
    for(unsigned int i = 1; i < num_waypoints_ - 1; i++) {
        double a = distanceOfSegments[i] / (distanceOfSegments[i] + distanceOfSegments[i-1]);
        double b = distanceOfSegments[i-1] / (distanceOfSegments[i] + distanceOfSegments[i-1]);

        second_derivatives[i] = a * computeSecondDerivativeAtEndPoint(waypoints_[i-1], first_derivatives[i-1],
                                                                      waypoints_[i], first_derivatives[i])
                              + b * computeSecondDerivativeAtBeginPoint(waypoints_[i], first_derivatives[i],
                                                                        waypoints_[i+1], first_derivatives[i+1]);
    }
    return second_derivatives;
}

std::array<Eigen::Vector2d, 6> QuinticBezierSpline::computeControlPointsOfSegment(Eigen::Vector2d& A, Eigen::Vector2d& tA, Eigen::Vector2d& aA,
                                                                                  Eigen::Vector2d& B, Eigen::Vector2d& tB, Eigen::Vector2d& aB) {
    std::array<Eigen::Vector2d, 6> control_points;                                                                               
    if ((A - B).norm() < 1e-3) {
        control_points[0] = A;
        control_points[5] = B;
    
        control_points[1] = control_points[0] + tA;
        control_points[2] = control_points[1] + aA;
        control_points[4] = control_points[5] - tB;
        control_points[3] = control_points[4] - aB;
    } else {
        control_points[0] = A;
        control_points[1] = control_points[0] + tA / 5;
        control_points[2] = -control_points[0] + 2.0 * control_points[1] + aA / 20;

        control_points[5] = B;
        control_points[4] = control_points[5] - tB / 5;
        control_points[3] = -control_points[5] + 2 * control_points[4] + aB / 20;
    }
    return control_points;
}

std::array<Eigen::Vector2d, 6> QuinticBezierSpline::computeCoefficientOfSegment(std::array<Eigen::Vector2d, 6>& control_points) {
    std::array<Eigen::Vector2d, 6> coefficients;
    coefficients[0] = control_points[0];
    coefficients[1] = 5 * (-control_points[0] + control_points[1]);
    coefficients[2] = 10 * (control_points[0] - 2 * control_points[1] + control_points[2]);
    coefficients[3] = 10 * (-control_points[0] + 3 * control_points[1] - 3 * control_points[2] + control_points[3]);
    coefficients[4] = 5 * (control_points[0] - 4 * control_points[1] + 6 * control_points[2] - 4 * control_points[3] 
                    + control_points[4]);
    coefficients[5] = -control_points[0] + 5 * control_points[1] - 10 * control_points[2] + 10 * control_points[3] 
                    - 5 * control_points[4] + control_points[5];
    return coefficients;
}

void QuinticBezierSpline::generationSpline() {
    std::vector<double> distanceOfSegments = computeDistanceOfSegments();
    std::vector<Eigen::Vector2d> first_derivatives = computeFirstDerivativeHeuristics(distanceOfSegments);
    std::vector<Eigen::Vector2d> second_derivatives = computeSecondDerivativeHeuristics(distanceOfSegments, first_derivatives);
    for(unsigned int i = 0; i < num_waypoints_ - 1; i++) {
        std::array<Eigen::Vector2d, 6> control_points = computeControlPointsOfSegment(waypoints_[i], first_derivatives[i], second_derivatives[i],
                                                                                     waypoints_[i+1], first_derivatives[i+1], second_derivatives[i+1]);
        std::array<Eigen::Vector2d, 6> coefficients = computeCoefficientOfSegment(control_points);
        segments_.push_back(coefficients);
    }
    computeArcLengths();
}

double QuinticBezierSpline::computeArcLength(double lower_limit, double upper_limit, int index_segment) {
    return rombergIntegrator(lower_limit, upper_limit, index_segment);
}

double QuinticBezierSpline::computeSpeedAtU(double u, int index_segment) {
    double u2 = u * u;
    double u3 = u2 * u;
    double u4 = u3 * u;
    std::array<Eigen::Vector2d, 6> coefficients = segments_[index_segment];
    Eigen::Vector2d q_dot = coefficients[1] + 2 * u * coefficients[2] + 3 * u2 * coefficients[3]
                            + 4 * u3 * coefficients[4] + 5 * u4 * coefficients[5];
    return q_dot.norm();
}

double QuinticBezierSpline::getCurvatureAtU(double u, int index_segment) {
    double u2 = u * u;
    double u3 = u2 * u;
    double u4 = u3 * u;
    std::array<Eigen::Vector2d, 6> coefficients = segments_[index_segment];
    Eigen::Vector2d q_dot = coefficients[1] + 2 * u * coefficients[2] + 3 * u2 * coefficients[3]
                            + 4 * u3 * coefficients[4] + 5 * u4 * coefficients[5];

    Eigen::Vector2d q_2dot = 2 * coefficients[2] + 6 * u * coefficients[3]
                            + 12 * u2 * coefficients[4] + 20 * u3 * coefficients[5];
    
    double numerator = fabs(q_dot[0] * q_2dot[1] - q_dot[1] * q_2dot[0]);
    double denominator = pow(q_dot.norm(), 3);
    return (denominator > 1e-8) ? (numerator / denominator) : 0.0;
}

double QuinticBezierSpline::rombergIntegrator(double a, double b, int index_segment) {
    int max_step = 50;
    double c, eps;
    double h = b - a;
    std::vector<double> R1, R2;
    R1.push_back((computeSpeedAtU(a, index_segment) + computeSpeedAtU(b, index_segment)) * h * 0.5);
    for(int i = 1; i < max_step; i++) {
        h *= 0.5;
        c = 0;
        eps = 1 << (i-1);
        for(int j = 1; j <= eps; j++) {
            c += computeSpeedAtU(a + (2*j - 1) * h, index_segment);
        }
        R2.push_back(h * c + 0.5 * R1[0]);

        double factor = 4.0;
        for(int j = 1; j <= i; j++) {
            R2.push_back((factor * R2[j-1] - R1[j-1]) / (factor - 1));
            factor *= 4;
        }

        if(i > 1 && std::fabs(R1[i-1] - R2[i]) < 1e-3) {
            return R2[i];
        }

        std::swap(R1, R2);
        R2.clear();
    }
    return R1.back();
}

void QuinticBezierSpline::computeArcLengths() {
    sum_length_ = 0;
    double l;
    for(unsigned int i = 0; i < num_waypoints_ - 1; i++) {
        l = computeArcLength(0, 1, i);
        arc_lengths_.push_back(l);
        sum_length_ += l;
    }
}

void QuinticBezierSpline::getState(double& x, double& y, double& theta, double u, int index_segment) {
    double u2 = u * u;
    double u3 = u2 * u;
    double u4 = u3 * u;
    double u5 = u4 * u;
    std::array<Eigen::Vector2d, 6> coefficients = segments_[index_segment];
    Eigen::Vector2d q = coefficients[0] + coefficients[1] * u + coefficients[2] * u2 + coefficients[3] * u3
                            + coefficients[4] * u4 + coefficients[5] * u5;

    Eigen::Vector2d q_dot = coefficients[1] + 2 * u * coefficients[2] + 3 * u2 * coefficients[3]
                            + 4 * u3 * coefficients[4] + 5 * u4 * coefficients[5];
    
    x = q[0];
    y = q[1];
    theta = atan2(q_dot[1], q_dot[0]);
}

void QuinticBezierSpline::getState(double (&x)[5], double u, double u_dot, int index_segment) {
    double u2 = u * u;
    double u3 = u2 * u;
    double u4 = u3 * u;
    double u5 = u4 * u;
    std::array<Eigen::Vector2d, 6> coefficients = segments_[index_segment];
    Eigen::Vector2d q = coefficients[0] + coefficients[1] * u + coefficients[2] * u2 + coefficients[3] * u3
                            + coefficients[4] * u4 + coefficients[5] * u5;
    
    Eigen::Vector2d q_dot = coefficients[1] + 2 * u * coefficients[2] + 3 * u2 * coefficients[3]
                            + 4 * u3 * coefficients[4] + 5 * u4 * coefficients[5];

    Eigen::Vector2d q_2dot = 2 * coefficients[2] + 6 * u * coefficients[3]
                            + 12 * u2 * coefficients[4] + 20 * u3 * coefficients[5];
    double numerator = fabs(q_dot[0] * q_2dot[1] - q_dot[1] * q_2dot[0]);
    double denominator = pow(q_dot.norm(), 3);
    double c = (denominator > 1e-8) ? (numerator / denominator) : 0.0;
    
    x[0] = q[0];
    x[1] = q[1];
    x[2] = atan2(q_dot[1], q_dot[0]);
    x[3] = q_dot.norm() * u_dot;
    x[4] = x[3] * c;
} 