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

TEST(MechanicalMatrixTest, MatrixB612GetElement) {
  MatrixB612 mat = create_matrix_B612(
      11, 12, 13, 14, 15, 16, 17, 18, 19, 110, 111, 112, 21, 22, 23, 24, 25, 26,
      27, 28, 29, 210, 211, 212, 31, 32, 33, 34, 35, 36, 37, 38, 39, 310, 311,
      312, 41, 42, 43, 44, 45, 46, 47, 48, 49, 410, 411, 412, 51, 52, 53, 54,
      55, 56, 57, 58, 59, 510, 511, 512, 61, 62, 63, 64, 65, 66, 67, 68, 69,
      610, 611, 612);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 0), 11);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 1), 12);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 2), 13);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 3), 14);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 4), 15);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 5), 16);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 6), 17);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 7), 18);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 8), 19);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 9), 110);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 10), 111);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 0, 11), 112);

  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 0), 21);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 1), 22);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 2), 23);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 3), 24);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 4), 25);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 5), 26);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 6), 27);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 7), 28);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 8), 29);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 9), 210);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 10), 211);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 1, 11), 212);

  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 0), 31);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 1), 32);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 2), 33);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 3), 34);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 4), 35);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 5), 36);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 6), 37);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 7), 38);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 8), 39);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 9), 310);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 10), 311);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 2, 11), 312);

  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 0), 41);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 1), 42);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 2), 43);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 3), 44);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 4), 45);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 5), 46);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 6), 47);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 7), 48);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 8), 49);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 9), 410);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 10), 411);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 3, 11), 412);

  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 0), 51);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 1), 52);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 2), 53);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 3), 54);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 4), 55);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 5), 56);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 6), 57);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 7), 58);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 8), 59);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 9), 510);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 10), 511);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 4, 11), 512);

  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 0), 61);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 1), 62);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 2), 63);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 3), 64);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 4), 65);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 5), 66);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 6), 67);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 7), 68);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 8), 69);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 9), 610);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 10), 611);
  EXPECT_DOUBLE_EQ(matrix_B612_get_element(&mat, 5, 11), 612);

  EXPECT_DEATH(matrix_B612_get_element(&mat, -1, 3),
               "FATAL: Index out of bounds \\(row: -1, col: 3\\)!");
  EXPECT_DEATH(matrix_B612_get_element(&mat, 6, 3),
               "FATAL: Index out of bounds \\(row: 6, col: 3\\)!");
  EXPECT_DEATH(matrix_B612_get_element(&mat, 1, -1),
               "FATAL: Index out of bounds \\(row: 1, col: -1\\)!");
  EXPECT_DEATH(matrix_B612_get_element(&mat, 1, 12),
               "FATAL: Index out of bounds \\(row: 1, col: 12\\)!");
}

TEST(MechanicalMatrixTest, CreateEmptyMatrixB612) {
  MatrixB612 mat = create_empty_matrix_B612();
  for (auto &i : mat.data) {
    for (double j : i) {
      EXPECT_EQ(j, 0);
    }
  }
}

TEST(MechanicalMatrixTest, MatrixB126GetElement) {
  MatrixB612 mat612 = create_matrix_B612(
      11, 12, 13, 14, 15, 16, 17, 18, 19, 110, 111, 112, 21, 22, 23, 24, 25, 26,
      27, 28, 29, 210, 211, 212, 31, 32, 33, 34, 35, 36, 37, 38, 39, 310, 311,
      312, 41, 42, 43, 44, 45, 46, 47, 48, 49, 410, 411, 412, 51, 52, 53, 54,
      55, 56, 57, 58, 59, 510, 511, 512, 61, 62, 63, 64, 65, 66, 67, 68, 69,
      610, 611, 612);
  MatrixB126 mat126 = create_matrix_B126_from_B612(&mat612);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 0), 11);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 1), 21);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 2), 31);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 3), 41);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 4), 51);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 5), 61);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 0), 12);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 1), 22);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 2), 32);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 3), 42);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 4), 52);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 5), 62);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 0), 13);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 1), 23);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 2), 33);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 3), 43);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 4), 53);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 5), 63);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 0), 14);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 1), 24);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 2), 34);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 3), 44);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 4), 54);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 5), 64);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 0), 15);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 1), 25);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 2), 35);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 3), 45);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 4), 55);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 5), 65);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 0), 16);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 1), 26);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 2), 36);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 3), 46);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 4), 56);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 5), 66);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 0), 17);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 1), 27);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 2), 37);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 3), 47);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 4), 57);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 5), 67);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 0), 18);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 1), 28);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 2), 38);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 3), 48);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 4), 58);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 5), 68);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 0), 19);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 1), 29);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 2), 39);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 3), 49);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 4), 59);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 5), 69);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 0), 110);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 1), 210);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 2), 310);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 3), 410);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 4), 510);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 5), 610);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 0), 111);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 1), 211);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 2), 311);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 3), 411);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 4), 511);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 5), 611);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 0), 112);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 1), 212);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 2), 312);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 3), 412);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 4), 512);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 5), 612);

  EXPECT_DEATH(matrix_B126_get_element(&mat126, 1, 12),
               "FATAL: Index out of bounds \\(row: 1, col: 12\\)!");
  EXPECT_DEATH(matrix_B126_get_element(&mat126, -1, 2),
               "FATAL: Index out of bounds \\(row: -1, col: 2\\)!");
  EXPECT_DEATH(matrix_B126_get_element(&mat126, 12, 3),
               "FATAL: Index out of bounds \\(row: 12, col: 3\\)!");
  EXPECT_DEATH(matrix_B126_get_element(&mat126, 3, -1),
               "FATAL: Index out of bounds \\(row: 3, col: -1\\)!");
}

TEST(MechanicalMatrixTest, MatirxB612Transpose) {
    MatrixB612 mat612 = create_matrix_B612(
      11, 12, 13, 14, 15, 16, 17, 18, 19, 110, 111, 112, 21, 22, 23, 24, 25, 26,
      27, 28, 29, 210, 211, 212, 31, 32, 33, 34, 35, 36, 37, 38, 39, 310, 311,
      312, 41, 42, 43, 44, 45, 46, 47, 48, 49, 410, 411, 412, 51, 52, 53, 54,
      55, 56, 57, 58, 59, 510, 511, 512, 61, 62, 63, 64, 65, 66, 67, 68, 69,
      610, 611, 612);
  MatrixB126 mat126 = matrix_B612_transpose(&mat612);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 0), 11);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 1), 21);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 2), 31);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 3), 41);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 4), 51);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 0, 5), 61);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 0), 12);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 1), 22);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 2), 32);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 3), 42);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 4), 52);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 1, 5), 62);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 0), 13);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 1), 23);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 2), 33);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 3), 43);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 4), 53);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 2, 5), 63);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 0), 14);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 1), 24);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 2), 34);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 3), 44);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 4), 54);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 3, 5), 64);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 0), 15);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 1), 25);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 2), 35);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 3), 45);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 4), 55);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 4, 5), 65);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 0), 16);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 1), 26);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 2), 36);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 3), 46);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 4), 56);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 5, 5), 66);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 0), 17);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 1), 27);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 2), 37);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 3), 47);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 4), 57);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 6, 5), 67);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 0), 18);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 1), 28);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 2), 38);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 3), 48);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 4), 58);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 7, 5), 68);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 0), 19);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 1), 29);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 2), 39);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 3), 49);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 4), 59);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 8, 5), 69);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 0), 110);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 1), 210);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 2), 310);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 3), 410);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 4), 510);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 9, 5), 610);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 0), 111);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 1), 211);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 2), 311);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 3), 411);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 4), 511);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 10, 5), 611);

  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 0), 112);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 1), 212);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 2), 312);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 3), 412);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 4), 512);
  EXPECT_DOUBLE_EQ(matrix_B126_get_element(&mat126, 11, 5), 612);
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
