#include <vk_motion_controller/velocity_profile.hpp>

NumericalVelocityProfile::NumericalVelocityProfile(double v_start, double v_end, double anpha,
                                                   double v_maxtrans, double v_maxrot,
                                                   double a_maxtrans, double a_maxrot,
                                                   double centrforce_max, double mass,
                                                   double scalingCoefficient, bool use_orientation_robot)
    : v_start_(v_start), v_end_(v_end), anpha_(anpha),
      v_maxtrans_(v_maxtrans), v_maxrot_(v_maxrot),
      a_maxtrans_(a_maxtrans), a_maxrot_(a_maxrot),
      centrforce_max_(centrforce_max), mass_(mass),
      spline_(scalingCoefficient, use_orientation_robot) {}

void NumericalVelocityProfile::computeDeltaS() {
    double sum_length = spline_.getSumLength();
    int M = 1;
    deltaS_ = sum_length;
    while(deltaS_ > 0.02) {
        M += 1;
        deltaS_ = sum_length/M;
    }
    num_points_ = M + 1;
}

RefPoint NumericalVelocityProfile::computeU(int index) {
    unsigned int num_segments = spline_.getNumOfWaypoints() - 1;
    double length_hat = 0;
    unsigned int k;
    for(unsigned int i = 0; i < num_segments; i++) {
        length_hat += spline_.getArcLengths()[i];
        if(index * deltaS_ <= length_hat) {
            k = i;
            break;
        }
    }

    double delta_length;
    if(k == 0)
        delta_length = index * deltaS_;
    else {
        delta_length = index * deltaS_;
        for(unsigned int j = 0; j < k; j++) {
            delta_length -= spline_.getArcLengths()[j];
        }
    }

    /* Bisector Method */
    double u_left = 0;
    double u_right = 1;
    double u = (u_left + u_right) / 2;
    double eps = spline_.computeArcLength(0, u, k) - delta_length;
    while(fabs(eps) > 1e-3) {
        if(eps < 0)
            u_left = u;
        else if (eps > 0)
            u_right = u;
        u = (u_left + u_right) / 2;
        eps = spline_.computeArcLength(0, u, k) - delta_length;
    }

    RefPoint p;
    p.u = u;
    p.t = 0.0;
    p.index = k;
    return p;
}

double NumericalVelocityProfile::getCurve(double u, unsigned int k) {
    return spline_.getCurvatureAtU(u, k);
}

double NumericalVelocityProfile::maxVelocityCurve(double& c) {
    return (anpha_ * v_maxrot_) / std::max(c, 1e-6);
}

double NumericalVelocityProfile::maxVelocityForce(double& c) {
    return sqrt(centrforce_max_ / (mass_ * std::max(c, 1e-6)));
}

double NumericalVelocityProfile::isolatedConstraints(double& c) {
    double v_maxf, v_maxc;
    v_maxf = maxVelocityForce(c);
    v_maxc = maxVelocityCurve(c);
    return std::min({anpha_ * v_maxtrans_, v_maxc, v_maxf});
}

void NumericalVelocityProfile::translationAccelerationConstraints(double& vi_1, double& vi) {
    vi = std::min(vi, sqrt(vi_1 * vi_1 + 2 * deltaS_ * a_maxtrans_));
}

void NumericalVelocityProfile::computeTime(double& vi_1, double& vi, double& Ti_1, double& Ti) {
    Ti = Ti_1 + 2 * deltaS_ / (vi + vi_1);
}

void NumericalVelocityProfile::setVelocityProfile() {
    computeDeltaS();
    double c;
    uts_.resize(num_points_);
    std::vector<double> v;
    v.resize(num_points_);
    v[0] = v_start_;
    for(int i = 1; i < num_points_; i++) {
        uts_[i] = computeU(i);
        c = getCurve(uts_[i].u, uts_[i].index);
        // Isolated constraints
        v[i] = isolatedConstraints(c);
        // Forward translational acceleration constraints
        translationAccelerationConstraints(v[i-1], v[i]);
    }
    
    // Backward translational acceleration constraints
    v.back() = v_end_;
    for(auto it = v.rbegin() + 1; it != v.rend() - 1; it++) {
        translationAccelerationConstraints(*it, *(it + 1));
    }

    // Assign time for points
    for(int i = 1; i < num_points_; i++) {
        computeTime(v[i-1], v[i], uts_[i-1].t, uts_[i].t);
    }
}