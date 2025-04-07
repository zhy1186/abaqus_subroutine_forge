//
// Created by Hengyi Zhao on 2024/11/28.
//

#include <gtest/gtest.h>

#include "abaqus_subroutine_forge.h"

TEST(MatrixTest, CreateMatrix2D) {
  Matrix2D mat = create_matrix_2D(1.0, 2.0, 3.0, 4.0);

  EXPECT_EQ(mat.type, Matrix2X2);
  EXPECT_DOUBLE_EQ(mat.data[0][0], 1.0);
  EXPECT_DOUBLE_EQ(mat.data[0][1], 2.0);
  EXPECT_DOUBLE_EQ(mat.data[1][0], 3.0);
  EXPECT_DOUBLE_EQ(mat.data[1][1], 4.0);
}

TEST(MatrixTest, CreateEmptyMatrix2D) {
  Matrix2D mat = create_empty_matrix_2D();

  EXPECT_DOUBLE_EQ(mat.data[0][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[0][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][1], 0.0);
}

TEST(MatrixTest, CreateEyeMatrix2D) {
  Matrix2D mat = create_eye_matrix_2D();

  EXPECT_DOUBLE_EQ(mat.data[0][0], 1.0);
  EXPECT_DOUBLE_EQ(mat.data[0][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][1], 1.0);
}

TEST(MatrixTest, Matrix2DGetElement) {
  Matrix2D mat = create_matrix_2D(1.0, 2.0, 3.0, 4.0);

  EXPECT_DOUBLE_EQ(matrix_2D_get_element(&mat, 0, 0), 1.0);
  EXPECT_DOUBLE_EQ(matrix_2D_get_element(&mat, 0, 1), 2.0);
  EXPECT_DOUBLE_EQ(matrix_2D_get_element(&mat, 1, 0), 3.0);
  EXPECT_DOUBLE_EQ(matrix_2D_get_element(&mat, 1, 1), 4.0);
}

TEST(MatrixTest, Matrix2DGetElementOutOfBounds) {
  Matrix2D mat = create_matrix_2D(1.0, 2.0, 3.0, 4.0);

  EXPECT_DEATH(matrix_2D_get_element(&mat, 2, 2),
               "FATAL: Index out of bounds \\(row: 2, col: 2\\)!");
}

TEST(MatrixTest, CreateMatrix3D) {
  Matrix3D mat = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

  EXPECT_EQ(mat.type, Matrix3X3);
  EXPECT_DOUBLE_EQ(mat.data[0][0], 1.0);
  EXPECT_DOUBLE_EQ(mat.data[0][1], 2.0);
  EXPECT_DOUBLE_EQ(mat.data[0][2], 3.0);
  EXPECT_DOUBLE_EQ(mat.data[1][0], 4.0);
  EXPECT_DOUBLE_EQ(mat.data[1][1], 5.0);
  EXPECT_DOUBLE_EQ(mat.data[1][2], 6.0);
  EXPECT_DOUBLE_EQ(mat.data[2][0], 7.0);
  EXPECT_DOUBLE_EQ(mat.data[2][1], 8.0);
  EXPECT_DOUBLE_EQ(mat.data[2][2], 9.0);
}

TEST(MatrixTest, CreateEmptyMatrix3D) {
  Matrix3D mat = create_empty_matrix_3D();

  EXPECT_DOUBLE_EQ(mat.data[0][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[0][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[0][2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][2], 0.0);
}

TEST(MatrixTest, CreateEyeMatrix3D) {
  Matrix3D mat = create_eye_matrix_3D();

  EXPECT_DOUBLE_EQ(mat.data[0][0], 1.0);
  EXPECT_DOUBLE_EQ(mat.data[0][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[0][2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][1], 1.0);
  EXPECT_DOUBLE_EQ(mat.data[1][2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][2], 1.0);
}

TEST(MatrixTest, Matrix3DGetElement) {
  Matrix3D mat = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

  EXPECT_DOUBLE_EQ(matrix_3D_get_element(&mat, 0, 0), 1.0);
  EXPECT_DOUBLE_EQ(matrix_3D_get_element(&mat, 0, 1), 2.0);
  EXPECT_DOUBLE_EQ(matrix_3D_get_element(&mat, 0, 2), 3.0);
  EXPECT_DOUBLE_EQ(matrix_3D_get_element(&mat, 1, 0), 4.0);
  EXPECT_DOUBLE_EQ(matrix_3D_get_element(&mat, 1, 1), 5.0);
  EXPECT_DOUBLE_EQ(matrix_3D_get_element(&mat, 1, 2), 6.0);
  EXPECT_DOUBLE_EQ(matrix_3D_get_element(&mat, 2, 0), 7.0);
  EXPECT_DOUBLE_EQ(matrix_3D_get_element(&mat, 2, 1), 8.0);
  EXPECT_DOUBLE_EQ(matrix_3D_get_element(&mat, 2, 2), 9.0);
}

TEST(MatrixTest, Matrix3DGetElementOutOfBounds) {
  Matrix3D mat = create_matrix_3D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

  EXPECT_DEATH(matrix_3D_get_element(&mat, 3, 3),
               "FATAL: Index out of bounds \\(row: 3, col: 3\\)!");
}

TEST(MechanicalMatrixTest, CreateMatrix6D) {
  Matrix6D mat = create_matrix_6D(
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0,
      15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0,
      27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0);

  EXPECT_EQ(mat.type, Matrix6X6);
  EXPECT_DOUBLE_EQ(mat.data[0][0], 1.0);
  EXPECT_DOUBLE_EQ(mat.data[0][1], 2.0);
  EXPECT_DOUBLE_EQ(mat.data[0][2], 3.0);
  EXPECT_DOUBLE_EQ(mat.data[0][3], 4.0);
  EXPECT_DOUBLE_EQ(mat.data[0][4], 5.0);
  EXPECT_DOUBLE_EQ(mat.data[0][5], 6.0);
  EXPECT_DOUBLE_EQ(mat.data[1][0], 7.0);
  EXPECT_DOUBLE_EQ(mat.data[1][1], 8.0);
  EXPECT_DOUBLE_EQ(mat.data[1][2], 9.0);
  EXPECT_DOUBLE_EQ(mat.data[1][3], 10.0);
  EXPECT_DOUBLE_EQ(mat.data[1][4], 11.0);
  EXPECT_DOUBLE_EQ(mat.data[1][5], 12.0);
  EXPECT_DOUBLE_EQ(mat.data[2][0], 13.0);
  EXPECT_DOUBLE_EQ(mat.data[2][1], 14.0);
  EXPECT_DOUBLE_EQ(mat.data[2][2], 15.0);
  EXPECT_DOUBLE_EQ(mat.data[2][3], 16.0);
  EXPECT_DOUBLE_EQ(mat.data[2][4], 17.0);
  EXPECT_DOUBLE_EQ(mat.data[2][5], 18.0);
  EXPECT_DOUBLE_EQ(mat.data[3][0], 19.0);
  EXPECT_DOUBLE_EQ(mat.data[3][1], 20.0);
  EXPECT_DOUBLE_EQ(mat.data[3][2], 21.0);
  EXPECT_DOUBLE_EQ(mat.data[3][3], 22.0);
  EXPECT_DOUBLE_EQ(mat.data[3][4], 23.0);
  EXPECT_DOUBLE_EQ(mat.data[3][5], 24.0);
  EXPECT_DOUBLE_EQ(mat.data[4][0], 25.0);
  EXPECT_DOUBLE_EQ(mat.data[4][1], 26.0);
  EXPECT_DOUBLE_EQ(mat.data[4][2], 27.0);
  EXPECT_DOUBLE_EQ(mat.data[4][3], 28.0);
  EXPECT_DOUBLE_EQ(mat.data[4][4], 29.0);
  EXPECT_DOUBLE_EQ(mat.data[4][5], 30.0);
  EXPECT_DOUBLE_EQ(mat.data[5][0], 31.0);
  EXPECT_DOUBLE_EQ(mat.data[5][1], 32.0);
  EXPECT_DOUBLE_EQ(mat.data[5][2], 33.0);
  EXPECT_DOUBLE_EQ(mat.data[5][3], 34.0);
  EXPECT_DOUBLE_EQ(mat.data[5][4], 35.0);
  EXPECT_DOUBLE_EQ(mat.data[5][5], 36.0);
}

TEST(MechanicalMatrixTest, CreateEmptyMatrix6D) {
  Matrix6D mat =
      create_matrix_6D(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  EXPECT_DOUBLE_EQ(mat.data[0][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[0][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[0][2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[0][3], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[0][4], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[0][5], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][3], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][4], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1][5], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][3], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][4], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2][5], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[3][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[3][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[3][2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[3][3], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[3][4], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[3][5], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[4][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[4][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[4][2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[4][3], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[4][4], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[4][5], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[5][0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[5][1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[5][2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[5][3], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[5][4], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[5][5], 0.0);
}

TEST(MechanicalMatrixTest, Matrix6DGetElement) {
  Matrix6D mat = create_matrix_6D(
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0,
      15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0,
      27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0);

  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 0, 0), 1.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 0, 1), 2.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 0, 2), 3.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 0, 3), 4.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 0, 4), 5.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 0, 5), 6.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 1, 0), 7.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 1, 1), 8.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 1, 2), 9.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 1, 3), 10.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 1, 4), 11.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 1, 5), 12.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 2, 0), 13.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 2, 1), 14.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 2, 2), 15.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 2, 3), 16.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 2, 4), 17.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 2, 5), 18.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 3, 0), 19.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 3, 1), 20.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 3, 2), 21.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 3, 3), 22.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 3, 4), 23.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 3, 5), 24.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 4, 0), 25.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 4, 1), 26.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 4, 2), 27.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 4, 3), 28.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 4, 4), 29.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 4, 5), 30.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 5, 0), 31.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 5, 1), 32.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 5, 2), 33.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 5, 3), 34.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 5, 4), 35.0);
  EXPECT_DOUBLE_EQ(matrix_6D_get_element(&mat, 5, 5), 36.0);
}

TEST(MechanicalMatrixTest, Matrix6DFillAbaqusDoubleArray) {
  Matrix6D mat = create_matrix_6D(
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0,
      15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0,
      27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0);
  auto *dst = (double *)malloc(DIMENSION_CPS3 * DIMENSION3 * sizeof(double));
  if (dst != nullptr) {
    matrix_6D_fill_abaqus_double_array(&mat, dst);
    for (int i = 0; i < (DIMENSION_CPS3 * DIMENSION_CPS3); ++i) {
      EXPECT_DOUBLE_EQ(dst[i], i + 1);
    }
    free(dst);
  } else {
    FAIL() << "malloc failed.";
  }
}

TEST(MechanicalMatrixTest, Matrix6DGetElementOutOfBound) {
  Matrix6D mat = create_empty_matrix_6D();

  EXPECT_DEATH(matrix_6D_get_element(&mat, 6, 6),
               "FATAL: Index out of bounds \\(row: 6, col: 6\\)!");
}

TEST(VectorTest, CreateVector2D) {
  Vector2D mat = create_vector_2D(1.0, 2.0);

  EXPECT_EQ(mat.type, Vector2X1);
  EXPECT_DOUBLE_EQ(mat.data[0], 1.0);
  EXPECT_DOUBLE_EQ(mat.data[1], 2.0);
}

TEST(VectorTest, CreateEmptyVector2D) {
  Vector2D mat = create_empty_vector_2D();

  EXPECT_DOUBLE_EQ(mat.data[0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1], 0.0);
}

TEST(VectorTest, Vector2DGetElement) {
  Vector2D mat = create_vector_2D(1.0, 2.0);

  EXPECT_DOUBLE_EQ(vector_2D_get_element(&mat, 0), 1.0);
  EXPECT_DOUBLE_EQ(vector_2D_get_element(&mat, 1), 2.0);
}

TEST(VectorTest, Vector2DGetElementOutOfBounds) {
  Vector2D mat = create_vector_2D(1.0, 2.0);

  EXPECT_DEATH(vector_2D_get_element(&mat, 3),
               "FATAL: Index out of bounds \\(row: 3\\)!");
}

TEST(VectorTest, CreateVector3D) {
  Vector3D mat = create_vector_3D(1.0, 2.0, 3.0);

  EXPECT_EQ(mat.type, Vector3X1);
  EXPECT_DOUBLE_EQ(mat.data[0], 1.0);
  EXPECT_DOUBLE_EQ(mat.data[1], 2.0);
  EXPECT_DOUBLE_EQ(mat.data[2], 3.0);
}

TEST(VectorTest, CreateEmptyVector3D) {
  Vector3D mat = create_empty_vector_3D();

  EXPECT_DOUBLE_EQ(mat.data[0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2], 0.0);
}

TEST(VectorTest, Vector3DGetElement) {
  Vector3D mat = create_vector_3D(1.0, 2.0, 3.0);

  EXPECT_DOUBLE_EQ(vector_3D_get_element(&mat, 0), 1.0);
  EXPECT_DOUBLE_EQ(vector_3D_get_element(&mat, 1), 2.0);
  EXPECT_DOUBLE_EQ(vector_3D_get_element(&mat, 2), 3.0);
}

TEST(VectorTest, Vector3DGetElementOutOfBounds) {
  Vector3D mat = create_vector_3D(1.0, 2.0, 3.0);

  EXPECT_DEATH(vector_3D_get_element(&mat, 3),
               "FATAL: Index out of bounds \\(row: 3\\)!");
}

TEST(VectorTest, CreateVector6D) {
  Vector6D mat = create_vector_6D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  EXPECT_EQ(mat.type, Vector6X1);
  EXPECT_DOUBLE_EQ(mat.data[0], 1.0);
  EXPECT_DOUBLE_EQ(mat.data[1], 2.0);
  EXPECT_DOUBLE_EQ(mat.data[2], 3.0);
  EXPECT_DOUBLE_EQ(mat.data[3], 4.0);
  EXPECT_DOUBLE_EQ(mat.data[4], 5.0);
  EXPECT_DOUBLE_EQ(mat.data[5], 6.0);
}

TEST(VectorTest, CreateEmptyVector6D) {
  Vector6D mat = create_empty_vector_6D();

  EXPECT_DOUBLE_EQ(mat.data[0], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[1], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[2], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[3], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[4], 0.0);
  EXPECT_DOUBLE_EQ(mat.data[5], 0.0);
}

TEST(VectorTest, Vector6DGetElement) {
  Vector6D mat = create_vector_6D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  EXPECT_DOUBLE_EQ(vector_6D_get_element(&mat, 0), 1.0);
  EXPECT_DOUBLE_EQ(vector_6D_get_element(&mat, 1), 2.0);
  EXPECT_DOUBLE_EQ(vector_6D_get_element(&mat, 2), 3.0);
  EXPECT_DOUBLE_EQ(vector_6D_get_element(&mat, 3), 4.0);
  EXPECT_DOUBLE_EQ(vector_6D_get_element(&mat, 4), 5.0);
  EXPECT_DOUBLE_EQ(vector_6D_get_element(&mat, 5), 6.0);
}

TEST(VectorTest, Vector6DGetElementOutOfBounds) {
  Vector6D mat = create_vector_6D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  EXPECT_DEATH(vector_6D_get_element(&mat, 6),
               "FATAL: Index out of bounds \\(row: 6\\)!");
}

TEST(VectorTest, Vector6DFillAbaqusDoubleArray) {
  Vector6D vec = create_vector_6D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  auto *dst = (double *)malloc(DIMENSION_CPS3 * sizeof(double));
  if (dst != nullptr) {
    vector_6D_fill_abaqus_double_array(&vec, dst);

    EXPECT_DOUBLE_EQ(dst[0], 1.0);
    EXPECT_DOUBLE_EQ(dst[1], 2.0);
    EXPECT_DOUBLE_EQ(dst[2], 3.0);
    EXPECT_DOUBLE_EQ(dst[3], 4.0);
    EXPECT_DOUBLE_EQ(dst[4], 5.0);
    EXPECT_DOUBLE_EQ(dst[5], 6.0);
    free(dst);
  } else {
    fatal_error("malloc failed !");
  }
}

TEST(BasicMatrix3D, Vector12D) {
  Vector12D vec =
      create_vector_12D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7, 8, 9, 10, 11, 12);

  ASSERT_NO_THROW(vector_12D_print(&vec));
  // check vector_12D_get_element
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    EXPECT_EQ(vector_12D_get_element(&vec, i), i + 1);
  }
  // check vector_12D_get_element error
  EXPECT_DEATH(vector_12D_get_element(&vec, DIMENSION_C3D4 + 1),
               "FATAL: Index out of bounds \\(index: 13\\)!");
  EXPECT_DEATH(vector_12D_get_element(&vec, -1),
               "FATAL: Index out of bounds \\(index: -1\\)!");
  // check empty vector 12D
  vec = create_empty_vector_12D();
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    EXPECT_EQ(vector_12D_get_element(&vec, i), 0);
  }
}

// print function test to de-warning by IDE
TEST(NaiveTest, CallPrintFunctions) {
  Matrix2D mat2d = create_empty_matrix_2D();
  Matrix3D mat3d = create_empty_matrix_3D();
  Vector2D vec2d = create_empty_vector_2D();
  Vector3D vec3d = create_empty_vector_3D();
  Vector6D vec6d = create_empty_vector_6D();
  Vector12D vec12d = create_empty_vector_12D();
  MatrixB36 matb36 = create_empty_matrix_B36();
  MatrixB63 matb63 = create_matrix_B63_from_B36(&matb36);
  Matrix6D mat6d = create_empty_matrix_6D();
  MatrixB612 matb612 = create_empty_matrix_B612();
  MatrixB126 matrixB126 = create_matrix_B126_from_B612(&matb612);

  matrix_2D_print(&mat2d);
  matrix_3D_print(&mat3d);
  vector_2D_print(&vec2d);
  vector_3D_print(&vec3d);
  vector_6D_print(&vec6d);
  vector_12D_print(&vec12d);
  matrix_B36_print(&matb36);
  matrix_B63_print(&matb63);
  matrix_6D_print(&mat6d);
  matrix_B612_print(&matb612);
  matrix_B126_print(&matrixB126);

  SUCCEED();  // no need to check output content, just call functions
}
