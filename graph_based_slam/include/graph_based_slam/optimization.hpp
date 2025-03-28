#pragma once
#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/SparseCholesky>
#include <eigen3/Eigen/SparseQR>

#include <graph_based_slam/graph_based_slam.hpp>

namespace slam2d {

#define MAX_INTERATIONS 500
#define MAX_MALAHANOBIS_DISTANCE 3
    
static const Eigen::Matrix2d SO2 = (Eigen::Matrix2d() << 0, -1, 1, 0).finished();

template <typename T>
void updateSparseBlock(Eigen::SparseMatrix<T>& H, int i, int j, const Eigen::Matrix<T, 3, 3>& block);

Eigen::Vector3d computeError(Eigen::Affine2d& Ti, Eigen::Affine2d& Tj, Eigen::Affine2d& Tij);

Eigen::Matrix3d computeMatrixA(Eigen::Affine2d& Ti, Eigen::Affine2d& Tj, Eigen::Affine2d& Tij);

Eigen::Matrix3d computeMatrixB(Eigen::Affine2d& Ti, Eigen::Affine2d& Tj, Eigen::Affine2d& Tij);

void graphOptimization(Graph* graph);

bool computeInformationMatrix(Graph* graph);

}   // slam2d namespace