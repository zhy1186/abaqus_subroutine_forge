//
// Created by Hengyi Zhao on 2024/11/29.
//

#pragma once

#include <Eigen/Dense>

#include "abaqus_subroutine_forge.h"

inline Eigen::Matrix2d to_eigen(const Matrix2D& mat) {
  Eigen::Matrix2d eigen_mat;
  for (int i = 0; i < DIMENSION2; ++i) {
    for (int j = 0; j < DIMENSION2; ++j) {
      eigen_mat(i, j) = mat.data[i][j];
    }
  }
  return eigen_mat;
}

inline Eigen::Matrix3d to_eigen(const Matrix3D& mat) {
  Eigen::Matrix3d eigen_mat;
  for (int i = 0; i < DIMENSION3; ++i) {
    for (int j = 0; j < DIMENSION3; ++j) {
      eigen_mat(i, j) = mat.data[i][j];
    }
  }
  return eigen_mat;
}

inline Eigen::Vector2d to_eigen(const Vector2D& vec) {
  Eigen::Vector2d eigen_vec;
  eigen_vec(0) = vec.data[0];
  eigen_vec(1) = vec.data[1];
  return eigen_vec;
}

inline Eigen::Vector3d to_eigen(const Vector3D& vec) {
  Eigen::Vector3d eigen_vec;
  eigen_vec(0) = vec.data[0];
  eigen_vec(1) = vec.data[1];
  eigen_vec(2) = vec.data[2];
  return eigen_vec;
}

inline Eigen::Matrix<double, 3, 6> to_eigen(const MatrixB36& mat) {
  Eigen::Matrix<double, 3, 6> eigen_mat;
  eigen_mat(0, 0) = mat.data[0][0];
  eigen_mat(0, 1) = mat.data[0][1];
  eigen_mat(0, 2) = mat.data[0][2];
  eigen_mat(0, 3) = mat.data[0][3];
  eigen_mat(0, 4) = mat.data[0][4];
  eigen_mat(0, 5) = mat.data[0][5];
  eigen_mat(1, 0) = mat.data[1][0];
  eigen_mat(1, 1) = mat.data[1][1];
  eigen_mat(1, 2) = mat.data[1][2];
  eigen_mat(1, 3) = mat.data[1][3];
  eigen_mat(1, 4) = mat.data[1][4];
  eigen_mat(1, 5) = mat.data[1][5];
  eigen_mat(2, 0) = mat.data[2][0];
  eigen_mat(2, 1) = mat.data[2][1];
  eigen_mat(2, 2) = mat.data[2][2];
  eigen_mat(2, 3) = mat.data[2][3];
  eigen_mat(2, 4) = mat.data[2][4];
  eigen_mat(2, 5) = mat.data[2][5];
  return eigen_mat;
}

inline Eigen::Matrix<double, 6, 3> to_eigen(const MatrixB63& mat) {
  Eigen::Matrix<double, 6, 3> eigen_mat;
  eigen_mat(0, 0) = mat.data[0][0];
  eigen_mat(0, 1) = mat.data[0][1];
  eigen_mat(0, 2) = mat.data[0][2];
  eigen_mat(1, 0) = mat.data[1][0];
  eigen_mat(1, 1) = mat.data[1][1];
  eigen_mat(1, 2) = mat.data[1][2];
  eigen_mat(2, 0) = mat.data[2][0];
  eigen_mat(2, 1) = mat.data[2][1];
  eigen_mat(2, 2) = mat.data[2][2];
  eigen_mat(3, 0) = mat.data[3][0];
  eigen_mat(3, 1) = mat.data[3][1];
  eigen_mat(3, 2) = mat.data[3][2];
  eigen_mat(4, 0) = mat.data[4][0];
  eigen_mat(4, 1) = mat.data[4][1];
  eigen_mat(4, 2) = mat.data[4][2];
  eigen_mat(5, 0) = mat.data[5][0];
  eigen_mat(5, 1) = mat.data[5][1];
  eigen_mat(5, 2) = mat.data[5][2];
  return eigen_mat;
}

inline Eigen::Matrix<double, 6, 6> to_eigen(const Matrix6D& mat) {
  Eigen::Matrix<double, 6, 6> eigen_mat;

  eigen_mat(0, 0) = mat.data[0][0];
  eigen_mat(0, 1) = mat.data[0][1];
  eigen_mat(0, 2) = mat.data[0][2];
  eigen_mat(0, 3) = mat.data[0][3];
  eigen_mat(0, 4) = mat.data[0][4];
  eigen_mat(0, 5) = mat.data[0][5];
  eigen_mat(1, 0) = mat.data[1][0];
  eigen_mat(1, 1) = mat.data[1][1];
  eigen_mat(1, 2) = mat.data[1][2];
  eigen_mat(1, 3) = mat.data[1][3];
  eigen_mat(1, 4) = mat.data[1][4];
  eigen_mat(1, 5) = mat.data[1][5];
  eigen_mat(2, 0) = mat.data[2][0];
  eigen_mat(2, 1) = mat.data[2][1];
  eigen_mat(2, 2) = mat.data[2][2];
  eigen_mat(2, 3) = mat.data[2][3];
  eigen_mat(2, 4) = mat.data[2][4];
  eigen_mat(2, 5) = mat.data[2][5];

  eigen_mat(3, 0) = mat.data[3][0];
  eigen_mat(3, 1) = mat.data[3][1];
  eigen_mat(3, 2) = mat.data[3][2];
  eigen_mat(3, 3) = mat.data[3][3];
  eigen_mat(3, 4) = mat.data[3][4];
  eigen_mat(3, 5) = mat.data[3][5];
  eigen_mat(4, 0) = mat.data[4][0];
  eigen_mat(4, 1) = mat.data[4][1];
  eigen_mat(4, 2) = mat.data[4][2];
  eigen_mat(4, 3) = mat.data[4][3];
  eigen_mat(4, 4) = mat.data[4][4];
  eigen_mat(4, 5) = mat.data[4][5];
  eigen_mat(5, 0) = mat.data[5][0];
  eigen_mat(5, 1) = mat.data[5][1];
  eigen_mat(5, 2) = mat.data[5][2];
  eigen_mat(5, 3) = mat.data[5][3];
  eigen_mat(5, 4) = mat.data[5][4];
  eigen_mat(5, 5) = mat.data[5][5];

  return eigen_mat;
}

inline Eigen::Matrix<double, 6, 1> to_eigen(const Vector6D& v) {
  Eigen::Matrix<double, 6, 1> res;
  for (int i = 0; i < 6; i++) {
    res(i, 0) = v.data[i];
  }
  return res;
}
