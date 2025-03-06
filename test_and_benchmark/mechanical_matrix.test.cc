//
// Created by Hengyi Zhao on 2025/3/5.
//

#include <gtest/gtest.h>

#include "abaqus_subroutine_forge.h"

TEST(MechanicalMatrixTest, CreateMatrixB36) {
  MatrixB36 mat =
      create_matrix_B36(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
                        12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);

  EXPECT_EQ(mat.type, MatrixB3X6);
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
}

TEST(MechanicalMatrixTest, CreateEmptyMatrixB36) {
  MatrixB36 mat = create_empty_matrix_B36();

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
}

TEST(MechanicalMatrixTest, MatrixB36GetElement) {
  MatrixB36 mat =
      create_matrix_B36(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
                        12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);

  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 0, 0), 1.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 0, 1), 2.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 0, 2), 3.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 0, 3), 4.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 0, 4), 5.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 0, 5), 6.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 1, 0), 7.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 1, 1), 8.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 1, 2), 9.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 1, 3), 10.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 1, 4), 11.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 1, 5), 12.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 2, 0), 13.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 2, 1), 14.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 2, 2), 15.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 2, 3), 16.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 2, 4), 17.0);
  EXPECT_DOUBLE_EQ(matrix_B36_get_element(&mat, 2, 5), 18.0);
}

TEST(MechanicalMatrixTest, MatrixB36GetElementOutOfBounds) {
  MatrixB36 mat = create_empty_matrix_B36();

  EXPECT_DEATH(matrix_B36_get_element(&mat, 3, 3),
               "FATAL: Index out of bounds \\(row: 3, col: 3\\)!");
}

TEST(MechanicalMatrixTest, CreateMatrixB63) {
  MatrixB36 mat_b36 =
      create_matrix_B36(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
                        12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);
  MatrixB63 mat_b63 = create_matrix_B63_from_B36(&mat_b36);

  EXPECT_EQ(mat_b63.type, MatrixB6X3);
  EXPECT_DOUBLE_EQ(mat_b63.data[0][0], 1.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[0][1], 7.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[0][2], 13.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[1][0], 2.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[1][1], 8.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[1][2], 14.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[2][0], 3.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[2][1], 9.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[2][2], 15.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[3][0], 4.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[3][1], 10.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[3][2], 16.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[4][0], 5.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[4][1], 11.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[4][2], 17.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[5][0], 6.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[5][1], 12.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[5][2], 18.0);
}

TEST(MechanicalMatrixTest, MatrixB63GetElement) {
  MatrixB36 mat_b36 =
      create_matrix_B36(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
                        12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);
  MatrixB63 mat_b63 = create_matrix_B63_from_B36(&mat_b36);

  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 0, 0), 1.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 0, 1), 7.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 0, 2), 13.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 1, 0), 2.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 1, 1), 8.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 1, 2), 14.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 2, 0), 3.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 2, 1), 9.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 2, 2), 15.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 3, 0), 4.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 3, 1), 10.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 3, 2), 16.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 4, 0), 5.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 4, 1), 11.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 4, 2), 17.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 5, 0), 6.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 5, 1), 12.0);
  EXPECT_DOUBLE_EQ(matrix_B63_get_element(&mat_b63, 5, 2), 18.0);
}

TEST(MechanicalMatrixTest, MatrixB63GetElementOutOfBounds) {
  MatrixB63 mat = create_empty_matrix_B63();

  EXPECT_DEATH(matrix_B63_get_element(&mat, 3, 3),
               "FATAL: Index out of bounds \\(row: 3, col: 3\\)!");
}

TEST(MechanicalMatrixTest, MatrixB36Transpose) {
  MatrixB36 mat_b36 =
      create_matrix_B36(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
                        12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);
  MatrixB63 mat_b63 = matrix_B36_transpose(&mat_b36);

  EXPECT_DOUBLE_EQ(mat_b63.data[0][0], 1.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[0][1], 7.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[0][2], 13.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[1][0], 2.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[1][1], 8.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[1][2], 14.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[2][0], 3.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[2][1], 9.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[2][2], 15.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[3][0], 4.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[3][1], 10.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[3][2], 16.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[4][0], 5.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[4][1], 11.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[4][2], 17.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[5][0], 6.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[5][1], 12.0);
  EXPECT_DOUBLE_EQ(mat_b63.data[5][2], 18.0);
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

TEST(MechanicalMatrixTest, Matrix6DGetElementOutOfBound) {
  Matrix6D mat = create_empty_matrix_6D();

  EXPECT_DEATH(matrix_6D_get_element(&mat, 6, 6),
               "FATAL: Index out of bounds \\(row: 6, col: 6\\)!");
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
