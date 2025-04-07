//
// Created by Hengyi Zhao on 2024/11/28.
//
#include <gtest/gtest.h>

#include "abaqus_subroutine_forge.h"
#include "eigen_adaptor.h"

TEST(BasicMathTest, TransposeMatrix2D) {
  Matrix2D mat = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
  Matrix2D trans = matrix_2D_transpose(&mat);

  Eigen::Matrix2d eigen_trans = to_eigen(mat).transpose();
  EXPECT_EQ(eigen_trans, to_eigen(trans));
}

TEST(BasicMathTest, TransposeMatrix3D) {
  Matrix3D mat = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  Matrix3D trans = matrix_3D_transpose(&mat);

  Eigen::Matrix3d eigen_trans = to_eigen(mat).transpose();
  EXPECT_EQ(eigen_trans, to_eigen(trans));
}

TEST(BasicMathTest, TransposeMatrix6D) {
  Matrix6D mat = create_matrix_6D(
      1.0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
      21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36);
  Matrix6D trans = matrix_6D_transpose(&mat);

  Eigen::Matrix<double, 6, 6> eigen_trans = to_eigen(mat).transpose();
  EXPECT_EQ(eigen_trans, to_eigen(trans));
}

TEST(BasicMathTest, AddMatrix2D) {
  Matrix2D mat1 = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
  Matrix2D mat2 = create_matrix_2D(5.0, 6.0, 7.0, 8.0);
  Matrix2D result = matrix_2D_add(&mat1, &mat2);

  Eigen::Matrix2d mat1_eigen = to_eigen(mat1);
  Eigen::Matrix2d mat2_eigen = to_eigen(mat2);
  Eigen::Matrix2d result_eigen = mat1_eigen + mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, AddMatrix3D) {
  Matrix3D mat1 = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  Matrix3D mat2 =
      create_matrix_3D(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);
  Matrix3D result = matrix_3D_add(&mat1, &mat2);

  Eigen::Matrix3d mat1_eigen = to_eigen(mat1);
  Eigen::Matrix3d mat2_eigen = to_eigen(mat2);
  Eigen::Matrix3d result_eigen = mat1_eigen + mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, AddMatrix6D) {
  Matrix6D mat1 = create_matrix_6D(
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
      22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36);
  Matrix6D mat2 = create_matrix_6D(
      36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19,
      18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);
  Matrix6D sum = matrix_6D_add(&mat1, &mat2);
  Eigen::Matrix<double, 6, 6> sum_expected;
  sum_expected = to_eigen(mat1) + to_eigen(mat2);
  EXPECT_EQ(to_eigen(sum), sum_expected);
}

TEST(BasicMathTest, AddVector2D) {
  Vector2D mat1 = create_vector_2D(1.0, 2.0);
  Vector2D mat2 = create_vector_2D(3.0, 4.0);
  Vector2D result = vector_2D_add(&mat1, &mat2);

  Eigen::Vector2d mat1_eigen = to_eigen(mat1);
  Eigen::Vector2d mat2_eigen = to_eigen(mat2);
  Eigen::Vector2d result_eigen = mat1_eigen + mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, AddVector3D) {
  Vector3D mat1 = create_vector_3D(1.0, 2.0, 3.0);
  Vector3D mat2 = create_vector_3D(4.0, 5.0, 6.0);
  Vector3D result = vector_3D_add(&mat1, &mat2);

  Eigen::Vector3d mat1_eigen = to_eigen(mat1);
  Eigen::Vector3d mat2_eigen = to_eigen(mat2);
  Eigen::Vector3d result_eigen = mat1_eigen + mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, AddVector6D) {
  Vector6D mat1 = create_vector_6D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  Vector6D mat2 = create_vector_6D(7.0, 8.0, 9.0, 10.0, 11.0, 12.0);
  Vector6D result = vector_6D_add(&mat1, &mat2);

  Eigen::Vector<double, 6> mat1_eigen = to_eigen(mat1);
  Eigen::Vector<double, 6> mat2_eigen = to_eigen(mat2);
  Eigen::Vector<double, 6> result_eigen = mat1_eigen + mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NumMulMatrix2D) {
  const double factor = 5.0;
  Matrix2D mat = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
  Matrix2D result = matrix_2D_number_multiplication(factor, &mat);

  Eigen::Matrix2d mat_eigen = to_eigen(mat);
  Eigen::Matrix2d result_eigen = factor * mat_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NumMulMatrix3D) {
  const double factor = 11.0;
  Matrix3D mat = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  Matrix3D result = matrix_3D_number_multiplication(factor, &mat);

  Eigen::Matrix3d mat_eigen = to_eigen(mat);
  Eigen::Matrix3d result_eigen = factor * mat_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NumMulMatrix6D) {
  const double factor = 1.23;
  Matrix6D mat = create_matrix_6D(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                  15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
                                  26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36);
  Matrix6D res = matrix_6D_number_multiplication(factor, &mat);
  Eigen::Matrix<double, 6, 6> expected = factor * to_eigen(mat);

  EXPECT_EQ(to_eigen(res), expected);
}

TEST(BasicMathTest, Matrix12DNumberMultiplicationGeneral) {
  Matrix12D m;
  m.type = Matrix12X12;
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION_C3D4; ++j) {
      m.data[i][j] = i * DIMENSION_C3D4 + j;
    }
  }
  double factor = 2.5;
  Matrix12D result = matrix_12D_number_multiplication(factor, &m);

  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION_C3D4; ++j) {
      EXPECT_DOUBLE_EQ(result.data[i][j], factor * m.data[i][j]);
    }
  }

  m.type = Matrix12X12;
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION_C3D4; ++j) {
      m.data[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
  factor = 0.0;
  result = matrix_12D_number_multiplication(factor, &m);

  for (auto& i : result.data) {
    for (double j : i) {
      EXPECT_DOUBLE_EQ(j, 0.0);
    }
  }

  m.type = Matrix12X12;
  for (auto& i : m.data) {
    for (double& j : i) {
      j = 3.14;
    }
  }
  factor = 1.0;
  result = matrix_12D_number_multiplication(factor, &m);

  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION_C3D4; ++j) {
      EXPECT_DOUBLE_EQ(result.data[i][j], m.data[i][j]);
    }
  }
}

TEST(BasicMathTest, NumMulVector2D) {
  const double factor = 3.0;
  Vector2D mat = create_vector_2D(1.0, 2.0);
  Vector2D result = vector_2D_number_multiplication(factor, &mat);

  Eigen::Vector2d mat_eigen = to_eigen(mat);
  Eigen::Vector2d result_eigen = factor * mat_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NumMulVector3D) {
  const double factor = 4.0;
  Vector3D mat = create_vector_3D(1.0, 2.0, 3.0);
  Vector3D result = vector_3D_number_multiplication(factor, &mat);

  Eigen::Vector3d mat_eigen = to_eigen(mat);
  Eigen::Vector3d result_eigen = factor * mat_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NumMulVector6D) {
  const double factor = 4.0;
  Vector6D mat = create_vector_6D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  Vector6D result = vector_6D_number_multiplication(factor, &mat);

  Eigen::Vector<double, 6> mat_eigen = to_eigen(mat);
  Eigen::Vector<double, 6> result_eigen = factor * mat_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NegativeMatrix2D) {
  Matrix2D mat = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
  Matrix2D result = matrix_2D_negative(&mat);

  Eigen::Matrix2d mat_eigen = to_eigen(mat);
  Eigen::Matrix2d result_eigen = -mat_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NegativeMatrix3D) {
  Matrix3D mat = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  Matrix3D result = matrix_3D_negative(&mat);

  Eigen::Matrix3d mat_eigen = to_eigen(mat);
  Eigen::Matrix3d result_eigen = -mat_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NegativeMatrix6D) {
  Matrix6D mat = create_matrix_6D(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                  15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
                                  26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36);
  Matrix6D result = matrix_6D_negative(&mat);

  Eigen::Matrix<double, 6, 6> mat_eigen = to_eigen(mat);
  Eigen::Matrix<double, 6, 6> result_eigen = -mat_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NegativeVector2D) {
  Vector2D vec = create_vector_2D(1.0, 2.0);
  Vector2D result = vector_2D_negative(&vec);

  Eigen::Vector2d vec_eigen = to_eigen(vec);
  Eigen::Vector2d result_eigen = -vec_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NegativeVector3D) {
  Vector3D vec = create_vector_3D(1.0, 2.0, 3.0);
  Vector3D result = vector_3D_negative(&vec);

  Eigen::Vector3d vec_eigen = to_eigen(vec);
  Eigen::Vector3d result_eigen = -vec_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, NegativeVector6D) {
  Vector6D mat = create_vector_6D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  Vector6D result = vector_6D_negative(&mat);

  Eigen::Vector<double, 6> mat_eigen = to_eigen(mat);
  Eigen::Vector<double, 6> result_eigen = -mat_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, MinusMatrix2D) {
  Matrix2D mat1 = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
  Matrix2D mat2 = create_matrix_2D(5.0, 6.0, 7.0, 8.0);
  Matrix2D result = matrix_2D_minus(&mat1, &mat2);

  Eigen::Matrix2d mat1_eigen = to_eigen(mat1);
  Eigen::Matrix2d mat2_eigen = to_eigen(mat2);
  Eigen::Matrix2d result_eigen = mat1_eigen - mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, MinusMatrix3D) {
  Matrix3D mat1 = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  Matrix3D mat2 =
      create_matrix_3D(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);
  Matrix3D result = matrix_3D_minus(&mat1, &mat2);

  Eigen::Matrix3d mat1_eigen = to_eigen(mat1);
  Eigen::Matrix3d mat2_eigen = to_eigen(mat2);
  Eigen::Matrix3d result_eigen = mat1_eigen - mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, MinusMatrix6D) {
  Matrix6D mat1 = create_matrix_6D(
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
      22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36);
  Matrix6D mat2 = create_matrix_6D(
      1, 3, 4, 5, 6, 8, 7, 9, 10, 13, 11, 13, 25, 36, 37, 87, 98, 76, 54, 26,
      15, 11, 22, 57, 45, 34, 23, 43, 21, 56, 76, 5567, 54, 33, 25, 65);
  Matrix6D res = matrix_6D_minus(&mat1, &mat2);

  Eigen::Matrix<double, 6, 6> expected = to_eigen(mat1) - to_eigen(mat2);
  EXPECT_EQ(to_eigen(res), expected);
}

TEST(BasicMathTest, MinusVector2D) {
  Vector2D mat1 = create_vector_2D(1.0, 2.0);
  Vector2D mat2 = create_vector_2D(3.0, 4.0);
  Vector2D result = vector_2D_minus(&mat1, &mat2);

  Eigen::Vector2d mat1_eigen = to_eigen(mat1);
  Eigen::Vector2d mat2_eigen = to_eigen(mat2);
  Eigen::Vector2d result_eigen = mat1_eigen - mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, MinusVector3D) {
  Vector3D mat1 = create_vector_3D(1.0, 2.0, 3.0);
  Vector3D mat2 = create_vector_3D(4.0, 5.0, 6.0);
  Vector3D result = vector_3D_minus(&mat1, &mat2);

  Eigen::Vector3d mat1_eigen = to_eigen(mat1);
  Eigen::Vector3d mat2_eigen = to_eigen(mat2);
  Eigen::Vector3d result_eigen = mat1_eigen - mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, MinusVector6D) {
  Vector6D mat1 = create_vector_6D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  Vector6D mat2 = create_vector_6D(7.0, 8.0, 9.0, 10.0, 11.0, 12.0);
  Vector6D result = vector_6D_minus(&mat1, &mat2);

  Eigen::Vector<double, 6> mat1_eigen = to_eigen(mat1);
  Eigen::Vector<double, 6> mat2_eigen = to_eigen(mat2);
  Eigen::Vector<double, 6> result_eigen = mat1_eigen - mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, MulMatrix2DMatrix2D) {
  Matrix2D mat1 = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
  Matrix2D mat2 = create_matrix_2D(5.0, 6.0, 7.0, 8.0);
  Matrix2D result = matrix_2D_mul_matrix_2D(&mat1, &mat2);

  Eigen::Matrix2d mat1_eigen = to_eigen(mat1);
  Eigen::Matrix2d mat2_eigen = to_eigen(mat2);
  Eigen::Matrix2d result_eigen = mat1_eigen * mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, MulMatrix3DMatrix3D) {
  Matrix3D mat1 = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  Matrix3D mat2 =
      create_matrix_3D(10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);

  Matrix3D result = matrix_3D_mul_matrix_3D(&mat1, &mat2);

  Eigen::Matrix3d mat1_eigen = to_eigen(mat1);
  Eigen::Matrix3d mat2_eigen = to_eigen(mat2);
  Eigen::Matrix3d result_eigen = mat1_eigen * mat2_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, MulMatrix6DMatrix6D) {
  Matrix6D mat1 = create_matrix_6D(
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
      22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36);
  Matrix6D mat2 = create_matrix_6D(
      52, 21, 342, 4, 5, 8, 7, 8, 9, 10, 11, 102, 13, 14, 15, 16, 17, 18, 19,
      20, 21, 2, 23, 24, 25, 26, 27, 21, 29, 0, 31, 32, 33, 34, 35, 36);
  Matrix6D res = matrix_6D_mul_matrix_6D(&mat1, &mat2);

  Eigen::Matrix<double, 6, 6> mat1_eigen = to_eigen(mat1);
  Eigen::Matrix<double, 6, 6> mat2_eigen = to_eigen(mat2);
  Eigen::Matrix<double, 6, 6> expected = mat1_eigen * mat2_eigen;

  EXPECT_EQ(to_eigen(res), expected);
}

TEST(BasicMathTest, MulMatrix2DVector2D) {
  Matrix2D mat = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
  Vector2D vec = create_vector_2D(5.0, 6.0);
  Vector2D result = matrix_2D_mul_vector_2D(&mat, &vec);

  Eigen::Matrix2d mat_eigen = to_eigen(mat);
  Eigen::Vector2d vec_eigen = to_eigen(vec);
  Eigen::Vector2d result_eigen = mat_eigen * vec_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, MulMatrix3DVector3D) {
  Matrix3D mat = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  Vector3D vec = create_vector_3D(10.0, 11.0, 12.0);
  Vector3D result = matrix_3D_mul_vector_3D(&mat, &vec);

  Eigen::Matrix3d mat_eigen = to_eigen(mat);
  Eigen::Vector3d vec_eigen = to_eigen(vec);
  Eigen::Vector3d result_eigen = mat_eigen * vec_eigen;

  EXPECT_EQ(to_eigen(result), result_eigen);
}

TEST(BasicMathTest, MulMatrix6DVector6D) {
  Matrix6D mat = create_matrix_6D(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                  15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
                                  26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36);
  Vector6D vec = create_vector_6D(1, 2, 3, 4, 5, 6);
  Vector6D res = matrix_6D_mul_vector_6D(&mat, &vec);

  Eigen::Matrix<double, 6, 6> mat_eigen = to_eigen(mat);
  Eigen::Vector<double, 6> vec_eigen = to_eigen(vec);
  Eigen::Vector<double, 6> expected = mat_eigen * vec_eigen;

  EXPECT_EQ(to_eigen(res), expected);
}

TEST(BasicMathTest, MatrixB126MulMatrix6DTest) {
  MatrixB126 b126;
  b126.type = MatrixB12X6;
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION6; ++j) {
      b126.data[i][j] = i * 10 + j;
    }
  }
  Matrix6D identity;
  identity.type = Matrix6X6;
  for (int i = 0; i < DIMENSION_CPS3; ++i) {
    for (int j = 0; j < DIMENSION_CPS3; ++j) {
      identity.data[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
  MatrixB126 res_identity = matrix_B126_mul_matrix_6D(&b126, &identity);
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION6; ++j) {
      EXPECT_DOUBLE_EQ(res_identity.data[i][j], b126.data[i][j]);
    }
  }

  Matrix6D zero;
  zero.type = Matrix6X6;
  for (auto& i : zero.data) {
    for (double& j : i) {
      j = 0.0;
    }
  }
  MatrixB126 res_zero = matrix_B126_mul_matrix_6D(&b126, &zero);
  for (auto& i : res_zero.data) {
    for (double j : i) {
      EXPECT_DOUBLE_EQ(j, 0.0);
    }
  }

  MatrixB126 b126_general;
  b126_general.type = MatrixB12X6;
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION6; ++j) {
      b126_general.data[i][j] = i + j;
    }
  }

  Matrix6D m6d;
  m6d.type = Matrix6X6;
  for (int i = 0; i < DIMENSION_CPS3; ++i) {
    for (int j = 0; j < DIMENSION_CPS3; ++j) {
      m6d.data[i][j] = (i == j) ? 2.0 : 3.0;
    }
  }
  MatrixB126 res_general = matrix_B126_mul_matrix_6D(&b126_general, &m6d);

  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION6; ++j) {
      double expected = 0.0;
      for (int k = 0; k < DIMENSION6; ++k) {
        double multiplier = (k == j) ? 2.0 : 3.0;
        expected += (i + k) * multiplier;
      }
      EXPECT_DOUBLE_EQ(res_general.data[i][j], expected);
    }
  }
}

TEST(BasicMathTest, MatrixB126MulMatrixB612Test) {
  // CASE 1: Zero matrices test
  MatrixB126 b126_zero;
  b126_zero.type = MatrixB12X6;
  for (auto& i : b126_zero.data) {
    for (double& j : i) {
      j = 0.0;
    }
  }
  MatrixB612 b612_zero;
  b612_zero.type = MatrixB6X12;
  for (auto& i : b612_zero.data) {
    for (double& j : i) {
      j = 0.0;
    }
  }
  Matrix12D result_zero = matrix_B126_mul_matrix_B612(&b126_zero, &b612_zero);
  for (auto& i : result_zero.data) {
    for (double j : i) {
      EXPECT_DOUBLE_EQ(j, 0.0);
    }
  }

  // CASE 2: All-ones matrices test
  MatrixB126 b126_ones;
  b126_ones.type = MatrixB12X6;
  for (auto& i : b126_ones.data) {
    for (double& j : i) {
      j = 1.0;
    }
  }
  MatrixB612 b612_ones;
  b612_ones.type = MatrixB6X12;
  for (auto& i : b612_ones.data) {
    for (double& j : i) {
      j = 1.0;
    }
  }
  Matrix12D result_ones = matrix_B126_mul_matrix_B612(&b126_ones, &b612_ones);
  // Each element should be the sum of 6 ones, which is 6.
  for (auto& i : result_ones.data) {
    for (double j : i) {
      EXPECT_DOUBLE_EQ(j, 6.0);
    }
  }

  // CASE 3: General test with custom values
  // Fill MatrixB126 with values: b126.data[i][j] = i + j
  MatrixB126 b126_general;
  b126_general.type = MatrixB12X6;
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION6; ++j) {
      b126_general.data[i][j] = i + j;
    }
  }
  // Fill MatrixB612 with values: b612.data[i][j] = i - j
  MatrixB612 b612_general;
  b612_general.type = MatrixB6X12;
  for (int i = 0; i < DIMENSION6; ++i) {
    for (int j = 0; j < DIMENSION_C3D4; ++j) {
      b612_general.data[i][j] = i - j;
    }
  }
  Matrix12D result_general =
      matrix_B126_mul_matrix_B612(&b126_general, &b612_general);
  // Verify each element manually
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION_C3D4; ++j) {
      double expected = 0.0;
      for (int k = 0; k < DIMENSION6; ++k) {
        expected += (i + k) * (k - j);
      }
      EXPECT_DOUBLE_EQ(result_general.data[i][j], expected);
    }
  }
}

TEST(BasicMathTest, MatrixB126MulVector6DTest) {
  // CASE 1: Zero vector test
  MatrixB126 mat_b126;
  mat_b126.type = MatrixB12X6;
  // Fill mat_b126 with values: mat_b126.data[i][j] = i + j
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION6; ++j) {
      mat_b126.data[i][j] = i + j;
    }
  }
  Vector6D zero_vec = create_empty_vector_6D();  // all elements are 0
  Vector12D result_zero = matrix_B126_mul_vector_6D(&mat_b126, &zero_vec);
  for (double i : result_zero.data) {
    EXPECT_DOUBLE_EQ(i, 0.0);
  }

  // CASE 2: Unit vector test (all ones)
  Vector6D ones_vec;
  ones_vec.type = Vector6X1;
  for (double & i : ones_vec.data) {
    i = 1.0;
  }
  Vector12D result_ones = matrix_B126_mul_vector_6D(&mat_b126, &ones_vec);
  // For each row, the expected value is sum_{k=0}^{5}(i + k)
  // The sum over k from 0 to 5 of (i + k) is 6*i + (0+1+2+3+4+5) = 6*i + 15.
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    double expected = 6 * i + 15;
    EXPECT_DOUBLE_EQ(result_ones.data[i], expected);
  }

  // CASE 3: General test with custom vector values
  // Create a vector v with values: [0, 1, 2, 3, 4, 5]
  Vector6D v;
  v.type = Vector6X1;
  v.data[0] = 0.0;
  v.data[1] = 1.0;
  v.data[2] = 2.0;
  v.data[3] = 3.0;
  v.data[4] = 4.0;
  v.data[5] = 5.0;
  // Create a different mat_b126: fill with values: mat_b126.data[i][j] = i - j
  MatrixB126 mat_gen;
  mat_gen.type = MatrixB12X6;
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    for (int j = 0; j < DIMENSION6; ++j) {
      mat_gen.data[i][j] = i - j;
    }
  }
  Vector12D result_gen = matrix_B126_mul_vector_6D(&mat_gen, &v);
  // Compute expected result: for each row i, expected = sum_{k=0}^{5} ( (i - k)
  // * v[k] )
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    double expected = 0.0;
    for (int k = 0; k < DIMENSION6; ++k) {
      expected += (i - k) * v.data[k];
    }
    EXPECT_DOUBLE_EQ(result_gen.data[i], expected);
  }
}

TEST(BasicMathTest, DetMatrix2D) {
  Matrix2D mat = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
  double det = matrix_2D_determinant(&mat);
  double det_eigen = to_eigen(mat).determinant();
  EXPECT_EQ(det, det_eigen);
}

TEST(BasicMathTest, DetMatrix3D) {
  Matrix3D mat = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
  double det = matrix_3D_determinant(&mat);
  double det_eigen = to_eigen(mat).determinant();
  EXPECT_EQ(det, det_eigen);
}

TEST(BasicMathTest, DetMatrix6D) {
  Matrix6D mat = create_matrix_6D(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                  15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
                                  26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36);
  double det = matrix_6D_determinant(&mat);

  Eigen::Matrix<double, 6, 6> mat_eigen = to_eigen(mat);
  double expected = mat_eigen.determinant();

  EXPECT_EQ(det, expected);
}

TEST(BasicMathTest, InvMatrix2D) {
  Matrix2D mat = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
  Matrix2D reverse = matrix_2D_inverse(&mat);
  Eigen::Matrix2d reverse_eigen = to_eigen(mat).inverse();
  EXPECT_EQ(to_eigen(reverse), reverse_eigen);
}

TEST(BasicMathTest, InvSingularMatrix2D) {
  Matrix2D mat = create_matrix_2D(0.0, 0.0, 3.0, 4.0);

  EXPECT_DEATH(matrix_2D_inverse(&mat),
               "FATAL: Matrix is singular and cannot be inverted.");
}

TEST(BasicMathTest, InvMatrix3D) {
  Matrix3D mat = create_matrix_3D(1.0, 2.0, 4.0, 3.0, 6.0, 8.0, 17.0, 2.0, 4.0);
  Matrix3D reverse = matrix_3D_inverse(&mat);
  Eigen::Matrix3d reverse_eigen = to_eigen(mat).inverse();
  EXPECT_EQ(to_eigen(reverse), reverse_eigen);
}

TEST(BasicMathTest, InvSingularMatrix3D) {
  Matrix3D mat = create_matrix_3D(0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  EXPECT_DEATH(matrix_3D_inverse(&mat),
               "FATAL: Matrix is singular and cannot be inverted.");
}
