#include <graph_based_slam/scan_matcher.hpp>

namespace slam2d {

void ScanMatcher::transformPoints(const Eigen::Affine2d& T,
                                  std::vector<Eigen::Vector2d>& points) {
    for(auto& point : points) {
        point = T.rotation() * point + T.translation();
    }                                
}

void ScanMatcher::getCorrespondences(const std::vector<Eigen::Vector2d>& source,
                                     Map2D *map,
                                     double max_correspondence_distance,
                                     std::vector<Correspondence>& corres) {
    corres.clear();
    int s = ceil(max_correspondence_distance / map->getResolution());
    unsigned int mx, my;
    double wx, wy;
    for(const auto& point : source) {
        if(!map->worldToMap(point[0], point[1], mx, my)) continue;

        double d2min = std::numeric_limits<double>::max();
        Correspondence corre;
        corre.p = point;
        for(int i = -s; i <= s; i++) {
            for(int j = -s; j <= s; j++) {
                if(!map->mapValid((int)(mx + i), (int)(my + j))) continue;

                if(map->getCost((unsigned int)(mx + i), (unsigned int)(my + j)) < OCCUPIED) continue;

                map->mapToWorld((unsigned int)(mx + i), (unsigned int)(my + j), wx, wy);
                double d2 = square(point[0] - wx) + square(point[1] - wy);
                if(d2 < d2min) {
                    corre.q = {wx, wy};
                    d2min = d2;
                }
            }
        }
        if(d2min < std::numeric_limits<double>::max())
            corres.push_back(corre);
    }
}

void ScanMatcher::getCorrespondences(const std::vector<Eigen::Vector2d>& source,
                                     const std::vector<Eigen::Vector2d>& target,
                                     double max_correspondence_distance,
                                     std::vector<Correspondence>& corres) {
    corres.clear();
    double max_d2 = max_correspondence_distance * max_correspondence_distance;
    for(const auto& p : source) {
        double d2min = max_d2;
        Correspondence corre;
        corre.p = p;
        for(const auto& q : target) {
            double d2 = (p - q).squaredNorm();
            if(d2 < d2min) {
                d2min = d2;
                corre.q = q;
            }
        }
        if(d2min < max_d2)
            corres.push_back(corre);
    }
}

void ScanMatcher::computeCovarianceAndMean(std::vector<Correspondence>& corres,
                                           Eigen::Vector2d& mean_ref,
                                           Eigen::Vector2d& mean_cur,
                                           Eigen::Matrix2d& H) {
    if(corres.empty()) return;
    mean_ref = Eigen::Vector2d::Zero();
    mean_cur = Eigen::Vector2d::Zero();                                        
    for(const auto& corre : corres) {
        mean_cur += corre.p;
        mean_ref += corre.q;
    }
    mean_ref /= corres.size();
    mean_cur /= corres.size();
    H.setZero(2, 2);
    for(const auto& corre : corres) {
        H += (corre.q - mean_ref) * (corre.p - mean_cur).transpose();
    }                                       
}

Eigen::Affine2d ScanMatcher::registration(const std::vector<Eigen::Vector2d>& scan,
                                          Map2D* map,
                                          const Eigen::Affine2d& initial_guess,
                                          double max_correspondence_distance) {
    std::vector<Eigen::Vector2d> source = scan;
    transformPoints(initial_guess, source);
    Eigen::Affine2d T_icp = Eigen::Affine2d::Identity();
    std::vector<Correspondence> corres;
    Eigen::Vector2d mean_ref, mean_cur;
    Eigen::Matrix2d H;
    Eigen::MatrixXd U, V, Hx;
    Hx.setZero(2, 2);
    for(int i = 0; i < max_iterations_; i++) {
        getCorrespondences(source, map, max_correspondence_distance, corres);
        computeCovarianceAndMean(corres, mean_ref, mean_cur, H);
        Hx = H;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hx, Eigen::ComputeThinU | Eigen::ComputeThinV);
        U = svd.matrixU();
        V = svd.matrixV();
        Eigen::Matrix2d R = U*V.transpose();
        Eigen::Vector3d dx;
        dx.head<2>() = mean_ref - R * mean_cur;
        dx[2] = atan2(R(1, 0), R(0, 0));
        Eigen::Affine2d deltaT = XYZEulertoAffineMatrix(dx);
        transformPoints(deltaT, source);
        T_icp = deltaT * T_icp;
        if(dx.norm() < epsilon_) break;
    }
    return T_icp * initial_guess;
}

Eigen::Matrix3d hessianMatrix(std::vector<Correspondence>& corres,
                              Eigen::Affine2d& Tij) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Matrix<double, 2, 3> J;
    for(const auto& corre : corres) {
        Eigen::Vector2d v = Tij.rotation() * corre.p;
        J << 1, 0, -v[1],
             0, 1, v[0];

        H += J.transpose() * J;
    }
    return H;
}

Eigen::Affine2d ScanMatcher::registration(const std::vector<Eigen::Vector2d>& scan,
                                          const std::vector<Eigen::Vector2d>& target,
                                          const Eigen::Affine2d& initial_guess,
                                          double max_correspondence_distance,
                                          Eigen::Matrix3d& Hess, bool& successful) {
    successful = false;
    std::vector<Eigen::Vector2d> source = scan;
    transformPoints(initial_guess, source);
    Eigen::Affine2d T_icp = Eigen::Affine2d::Identity();
    std::vector<Correspondence> corres;
    Eigen::Vector2d mean_ref, mean_cur;
    Eigen::Matrix2d H;
    Eigen::MatrixXd U, V, Hx;
    Hx.setZero(2, 2);
    for(int i = 0; i < max_iterations_; i++) {
        getCorrespondences(source, target, max_correspondence_distance, corres);
        computeCovarianceAndMean(corres, mean_ref, mean_cur, H);
        Hx = H;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hx, Eigen::ComputeThinU | Eigen::ComputeThinV);
        U = svd.matrixU();
        V = svd.matrixV();
        Eigen::Matrix2d R = U*V.transpose();
        Eigen::Vector3d dx;
        dx.head<2>() = mean_ref - R * mean_cur;
        dx[2] = atan2(R(1, 0), R(0, 0));
        Eigen::Affine2d deltaT = XYZEulertoAffineMatrix(dx);
        transformPoints(deltaT, source);
        T_icp = deltaT * T_icp;
        if(dx.norm() < epsilon_) {
            successful = true;
            break;
        }
    }
    Eigen::Affine2d Tij = T_icp * initial_guess;
    Hess = hessianMatrix(corres, Tij);
    return Tij;
}

}   // slam2d namespace