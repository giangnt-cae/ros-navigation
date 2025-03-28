#include <graph_based_slam/optimization.hpp>

namespace slam2d {

template <typename T>
void updateSparseBlock(Eigen::SparseMatrix<T>& H, int i, int j, const Eigen::Matrix<T, 3, 3>& block) {
    for (int n = 3 * i; n < 3 * i + 3; n++) {
        for (int m = 3 * j; m < 3 * j + 3; m++) {
            H.coeffRef(n, m) += block(n % 3, m % 3);
        }
    }
}

Eigen::Vector3d computeError(Eigen::Affine2d& Ti, Eigen::Affine2d& Tj, Eigen::Affine2d& Tij) {
    Eigen::Matrix2d Ri = Ti.linear();
    Eigen::Matrix2d Rj = Tj.linear();
    Eigen::Matrix2d Rij = Tij.linear();
    double theta_i = std::atan2(Ri(1, 0), Ri(0, 0));
    double theta_j = std::atan2(Rj(1, 0), Rj(0, 0));
    double theta_ij = std::atan2(Rij(1, 0), Rij(0, 0));

    Eigen::Vector2d ti = Ti.translation();
    Eigen::Vector2d tj = Tj.translation();
    Eigen::Vector2d tij = Tij.translation();

    Eigen::Vector3d eij;
    eij << Rij.transpose() * (Ri.transpose() * (tj - ti) - tij),
           angle_diff(theta_j -theta_i, theta_ij);
    return eij;
}

Eigen::Matrix3d computeMatrixA(Eigen::Affine2d& Ti, Eigen::Affine2d& Tj, Eigen::Affine2d& Tij) {
    Eigen::Matrix3d Aij = Eigen::Matrix3d::Zero();
    Aij.block<2, 2>(0, 0) = -Tij.rotation().transpose() * Ti.rotation().transpose();
    Aij.block<2, 1>(0, 2) =  Tij.rotation().transpose() * (Ti.rotation() * SO2).transpose() *
                             (Tj.translation() - Ti.translation());
    Aij(2, 2) = -1;
    return Aij;
}

Eigen::Matrix3d computeMatrixB(Eigen::Affine2d& Ti, Eigen::Affine2d& Tj, Eigen::Affine2d& Tij) {
    Eigen::Matrix3d Bij = Eigen::Matrix3d::Zero();
    Bij.block<2, 2>(0, 0) = Tij.rotation().transpose() * Ti.rotation().transpose();
    Bij(2, 2) = 1;
    return Bij;
}

void graphOptimization(Graph* graph) {
    const int N = graph->nodes.size();
    Eigen::VectorXd b = Eigen::VectorXd::Zero(3*N);
    Eigen::VectorXd x(3*N);
    Eigen::SparseMatrix<double> H(3*N, 3*N);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // Initial guess x[]
    for(const auto& node : graph->nodes) {
        int i = 0;
        x.segment(3*i, 3) = node.pose;
        i++;
    }
    bool successful = false;
    int count = 0;
    while(true) {
        b.setZero();
        H.setZero();

        for(auto& cst : graph->edges) {
            unsigned int i = cst.ni, j = cst.nj;
            Eigen::Vector3d xi = x.segment(3*i, 3), xj = x.segment(3*j, 3);
            xi(2) = normalize(xi(2));
            xj(2) = normalize(xj(2));

            Eigen::Affine2d Ti = XYZEulertoAffineMatrix(xi);
            Eigen::Affine2d Tj = XYZEulertoAffineMatrix(xj);
            
            Eigen::Matrix3d Aij = computeMatrixA(Ti, Tj, cst.z);
            Eigen::Matrix3d Bij = computeMatrixB(Ti, Tj, cst.z);
            
            /* H[ii] += A_ij^T*omega_ij*A_ij */
            Eigen::Matrix3d tmp = Aij.transpose() * cst.omega * Aij;
            updateSparseBlock(H, i, i, tmp);

            /* H[ij] += A_ij^T*omega_ij*B_ij */
            tmp = Aij.transpose() * cst.omega * Bij;
            updateSparseBlock(H, i, j, tmp);

            /* H[ji] += B_ij^T*omega_ij*A_ij */
            tmp = Bij.transpose() * cst.omega * Aij;
            updateSparseBlock(H, j, i, tmp);

            /* H[jj] += B_ij^T*omega_ij*B_ij */
            tmp = Bij.transpose() * cst.omega * Bij;
            updateSparseBlock(H, j, j, tmp);

            Eigen::Vector3d eij = computeError(Ti, Tj, cst.z);
            /* b[i] += A_ij^T*omega_ij*e_ij */
            b.segment(3*i, Aij.rows()) += Aij.transpose() * cst.omega * eij;
            /* b[j] += B_ij^T*omega_ij*e_ij */
            b.segment(3*j, Bij.rows()) += Bij.transpose() * cst.omega * eij;
        }
        /* Keep first node fixed: H[11] += I */
        updateSparseBlock(H, 0, 0, I);

        H.makeCompressed();
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver;
        solver.compute(H);
        if(solver.info() != Eigen::Success) {
            ROS_WARN("Decomposition failed!");
            break;
        }

        Eigen::VectorXd dx = solver.solve(-b);
        x += dx;
        count++;
        if(dx.norm() < 1e-6 || count > MAX_INTERATIONS)
            successful = true;
            break;
    }

    /* H* = H; H*[11] -= I */
    // Eigen::SparseMatrix<double> H_star = H;
    // I = (-1.0) * I;
    // updateSparseBlock(H_star, 0, 0, I);

    // Eigen::SparseMatrix<double> H_red(3*(N -1), 3*(N - 1));
    // for(int i = 0; i < 3*(N - 1); i++) {
    //     for(int j = 0; j < 3*(N - 1); j++) {
    //         H_red.coeffRef(i, j) = H.coeffRef(i, j);
    //     }
    // }
    // H_red.makeCompressed();

    // Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    // solver.compute(H_red);

    // if (solver.info() != Eigen::Success) {
    //     ROS_WARN("Decomposition failed!");
    // } else {
    //     Eigen::MatrixXd sigma = solver.solve(Eigen::MatrixXd::Identity(3*(N -1), 3*(N - 1)));
    // }
    
    // Update node after optimization
    if(successful) {
        ROS_INFO("Optimization is successful!");
        for(int k = 0; k < N; k++) {
            graph->nodes[k].pose = x.segment(3*k, 3);
        }
    }
}

bool computeInformationMatrix(Graph* graph) {
    const int N = graph->nodes.size();
    Eigen::SparseMatrix<double> H(3*N, 3*N);
    H.setZero();

    for(auto& cst : graph->edges) {
        unsigned int i = cst.ni, j = cst.nj;
        Eigen::Vector3d xi = graph->nodes[i].pose, xj = graph->nodes[j].pose;
        xi(2) = normalize(xi(2));
        xj(2) = normalize(xj(2));

        Eigen::Affine2d Ti = XYZEulertoAffineMatrix(xi);
        Eigen::Affine2d Tj = XYZEulertoAffineMatrix(xj);
            
        Eigen::Matrix3d Aij = computeMatrixA(Ti, Tj, cst.z);
        Eigen::Matrix3d Bij = computeMatrixB(Ti, Tj, cst.z);
            
        /* H[ii] += A_ij^T*omega_ij*A_ij */
        Eigen::Matrix3d tmp = Aij.transpose() * cst.omega * Aij;
        updateSparseBlock(H, i, i, tmp);

        /* H[ij] += A_ij^T*omega_ij*B_ij */
        tmp = Aij.transpose() * cst.omega * Bij;
        updateSparseBlock(H, i, j, tmp);

        /* H[ji] += B_ij^T*omega_ij*A_ij */
        tmp = Bij.transpose() * cst.omega * Aij;
        updateSparseBlock(H, j, i, tmp);

        /* H[jj] += B_ij^T*omega_ij*B_ij */
        tmp = Bij.transpose() * cst.omega * Bij;
        updateSparseBlock(H, j, j, tmp);    
    }

    Eigen::SparseMatrix<double> H_red(3*(N -1), 3*(N - 1));
    for(int i = 0; i < 3*(N - 1); i++) {
        for(int j = 0; j < 3*(N - 1); j++) {
            H_red.coeffRef(i, j) = H.coeffRef(i, j);
        }
    }
    H_red.makeCompressed();
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(H_red);

    if (solver.info() != Eigen::Success) {
        ROS_WARN("Decomposition failed!");
        return false;
    } else {
        Eigen::MatrixXd sigma = solver.solve(Eigen::MatrixXd::Identity(3*(N -1), 3*(N - 1)));
        for(int k = 0; k < N - 1; k++) {
            graph->nodes[k].omega = sigma.block<3, 3>(3*k, 3*k).inverse();
        }
        return true;
    }
}


}   // slam2d namespace