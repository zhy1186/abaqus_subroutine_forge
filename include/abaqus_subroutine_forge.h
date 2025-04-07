// abaqus_subroutine_forge.h -- header for ABAQUS user-subroutine development
// Created by Hengyi Zhao (zhaohy19@mails.thu.edu.cn) on 2025/03/06
//

// disable clang-tidy IDE C++ check

#pragma once

#pragma clang diagnostic push
#pragma ide diagnostic ignored "modernize-deprecated-headers"
#pragma ide diagnostic ignored "modernize-use-auto"
#pragma ide diagnostic ignored "modernize-use-nullptr"
#pragma ide diagnostic ignored "UnusedLocalVariable"
#pragma ide diagnostic ignored "UnusedValue"

// designed header-only for ABAQUS subroutine development
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#define DIMENSION2 2
#define DIMENSION3 3
#define DIMENSION_CPS3 6
#define DIMENSION6 6
#define DIMENSION_C3D4 12

#define ZERO_TOLERANCE 1e-15

typedef enum {
  Vector2X1,
  Vector3X1,
  Vector6X1,
  Vector12X1,
  Matrix2X2,
  Matrix3X3,
  Matrix6X6,
  MatrixB3X6,
  MatrixB6X3,
  MatrixB6X12,
  MatrixB12X6
} MatrixType;

// Util functions
void fatal_error(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  fflush(stderr);
  va_end(args);
  abort();
}

// Basic type definition (Matrix and Vector)
// Matrix 2D -- use in 2D tensor
typedef struct Matrix2D {
  MatrixType type;
  double data[DIMENSION2][DIMENSION2];
} Matrix2D;

void matrix_2D_print(const Matrix2D* mat) {
  printf("Matrix2D: [[%f, %f], [%f, %f]]\n", mat->data[0][0], mat->data[0][1],
         mat->data[1][0], mat->data[1][1]);
}

double matrix_2D_get_element(const Matrix2D* mat, int row, int col) {
  if (col < 0 || col >= DIMENSION2 || row < 0 || row >= DIMENSION2) {
    fatal_error("FATAL: Index out of bounds (row: %d, col: %d)!", row, col);
  }
  return mat->data[row][col];
}

Matrix2D create_matrix_2D(double ele11, double ele12, double ele21,
                          double ele22) {
  Matrix2D mat;
  mat.type = Matrix2X2;
  mat.data[0][0] = ele11;
  mat.data[0][1] = ele12;
  mat.data[1][0] = ele21;
  mat.data[1][1] = ele22;
  return mat;
}

Matrix2D create_empty_matrix_2D() { return create_matrix_2D(0, 0, 0, 0); }

Matrix2D create_eye_matrix_2D() { return create_matrix_2D(1, 0, 0, 1); }

// Matrix 3D -- use in 3D tensor
typedef struct Matrix3D {
  MatrixType type;
  double data[DIMENSION3][DIMENSION3];
} Matrix3D;

void matrix_3D_print(const Matrix3D* mat) {
  printf("Matrix3D: [[%f, %f, %f], [%f, %f, %f], [%f, %f, %f]]\n",
         mat->data[0][0], mat->data[0][1], mat->data[0][2], mat->data[1][0],
         mat->data[1][1], mat->data[1][2], mat->data[2][0], mat->data[2][1],
         mat->data[2][2]);
}

double matrix_3D_get_element(const Matrix3D* mat, int row, int col) {
  if (col < 0 || col >= DIMENSION3 || row < 0 || row >= DIMENSION3) {
    fatal_error("FATAL: Index out of bounds (row: %d, col: %d)!\n", row, col);
  }
  return mat->data[row][col];
}

Matrix3D create_matrix_3D(double ele11, double ele12, double ele13,
                          double ele21, double ele22, double ele23,
                          double ele31, double ele32, double ele33) {
  Matrix3D mat;
  mat.type = Matrix3X3;
  mat.data[0][0] = ele11;
  mat.data[0][1] = ele12;
  mat.data[0][2] = ele13;
  mat.data[1][0] = ele21;
  mat.data[1][1] = ele22;
  mat.data[1][2] = ele23;
  mat.data[2][0] = ele31;
  mat.data[2][1] = ele32;
  mat.data[2][2] = ele33;
  return mat;
}

Matrix3D create_empty_matrix_3D() {
  return create_matrix_3D(0, 0, 0, 0, 0, 0, 0, 0, 0);
}

Matrix3D create_eye_matrix_3D() {
  return create_matrix_3D(1, 0, 0, 0, 1, 0, 0, 0, 1);
}

// Vector 2D -- use in 2D vector
typedef struct Vector2D {
  MatrixType type;
  double data[DIMENSION2];
} Vector2D;

void vector_2D_print(const Vector2D* mat) {
  printf("Vector2D: [%f, %f]\n", mat->data[0], mat->data[1]);
}

double vector_2D_get_element(const Vector2D* mat, int row) {
  if (row < 0 || row >= DIMENSION2) {
    fatal_error("FATAL: Index out of bounds (row: %d)!\n", row);
  }
  return mat->data[row];
}

Vector2D create_vector_2D(double ele11, double ele21) {
  Vector2D mat;
  mat.type = Vector2X1;
  mat.data[0] = ele11;
  mat.data[1] = ele21;
  return mat;
}

Vector2D create_empty_vector_2D() { return create_vector_2D(0, 0); }

// Vector 3D -- use in 3D vector
typedef struct Vector3D {
  MatrixType type;
  double data[DIMENSION3];
} Vector3D;

void vector_3D_print(const Vector3D* mat) {
  printf("Vector3D: [%f, %f, %f]\n", mat->data[0], mat->data[1], mat->data[2]);
}

double vector_3D_get_element(const Vector3D* mat, int row) {
  if (row < 0 || row >= DIMENSION3) {
    fatal_error("FATAL: Index out of bounds (row: %d)!\n", row);
  }
  return mat->data[row];
}

Vector3D create_vector_3D(double ele11, double ele21, double ele31) {
  Vector3D vec;
  vec.type = Vector3X1;
  vec.data[0] = ele11;
  vec.data[1] = ele21;
  vec.data[2] = ele31;
  return vec;
}

Vector3D create_empty_vector_3D() { return create_vector_3D(0, 0, 0); }

// Matrix definition for CPS3 element matrix B
typedef struct MatrixB36 {
  MatrixType type;
  double data[DIMENSION3][DIMENSION_CPS3];
} MatrixB36;

void matrix_B36_print(const MatrixB36* mat) {
  printf(
      "MatrixB36: [[%f, %f, %f, %f, %f, %f], [%f, %f, %f, %f, %f, %f], [%f, "
      "%f, %f, %f, %f, %f]]\n",
      mat->data[0][0], mat->data[0][1], mat->data[0][2], mat->data[0][3],
      mat->data[0][4], mat->data[0][5], mat->data[1][0], mat->data[1][1],
      mat->data[1][2], mat->data[1][3], mat->data[1][4], mat->data[1][5],
      mat->data[2][0], mat->data[2][1], mat->data[2][2], mat->data[2][3],
      mat->data[2][4], mat->data[2][5]);
}

double matrix_B36_get_element(const MatrixB36* mat, int row, int col) {
  if (col < 0 || col >= DIMENSION_CPS3 || row < 0 || row >= DIMENSION3) {
    fatal_error("FATAL: Index out of bounds (row: %d, col: %d)!\n", row, col);
  }
  return mat->data[row][col];
}

MatrixB36 create_matrix_B36(double ele11, double ele12, double ele13,
                            double ele14, double ele15, double ele16,
                            double ele21, double ele22, double ele23,
                            double ele24, double ele25, double ele26,
                            double ele31, double ele32, double ele33,
                            double ele34, double ele35, double ele36) {
  MatrixB36 mat;
  mat.type = MatrixB3X6;
  mat.data[0][0] = ele11;
  mat.data[0][1] = ele12;
  mat.data[0][2] = ele13;
  mat.data[0][3] = ele14;
  mat.data[0][4] = ele15;
  mat.data[0][5] = ele16;
  mat.data[1][0] = ele21;
  mat.data[1][1] = ele22;
  mat.data[1][2] = ele23;
  mat.data[1][3] = ele24;
  mat.data[1][4] = ele25;
  mat.data[1][5] = ele26;
  mat.data[2][0] = ele31;
  mat.data[2][1] = ele32;
  mat.data[2][2] = ele33;
  mat.data[2][3] = ele34;
  mat.data[2][4] = ele35;
  mat.data[2][5] = ele36;
  return mat;
}

MatrixB36 create_empty_matrix_B36() {
  return create_matrix_B36(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0);
}

typedef struct MatrixB63 {
  MatrixType type;
  double data[DIMENSION_CPS3][DIMENSION3];
} MatrixB63;

void matrix_B63_print(const MatrixB63* mat) {
  printf(
      "MatrixB63: [[%f, %f, %f], [%f, %f, %f], [%f, %f, %f], [%f, %f, %f], "
      "[%f, %f, %f], [%f, %f, %f]]\n",
      mat->data[0][0], mat->data[0][1], mat->data[0][2], mat->data[1][0],
      mat->data[1][1], mat->data[1][2], mat->data[2][0], mat->data[2][1],
      mat->data[2][2], mat->data[3][0], mat->data[3][1], mat->data[3][2],
      mat->data[4][0], mat->data[4][1], mat->data[4][2], mat->data[5][0],
      mat->data[5][1], mat->data[5][2]);
}

double matrix_B63_get_element(const MatrixB63* mat, int row, int col) {
  if (col < 0 || col >= DIMENSION3 || row < 0 || row >= DIMENSION_CPS3) {
    fatal_error("FATAL: Index out of bounds (row: %d, col: %d)!\n", row, col);
  }
  return mat->data[row][col];
}

MatrixB63 create_matrix_B63_from_B36(MatrixB36* mat_b36) {
  MatrixB63 mat_b63;
  mat_b63.type = MatrixB6X3;
  mat_b63.data[0][0] = mat_b36->data[0][0];
  mat_b63.data[0][1] = mat_b36->data[1][0];
  mat_b63.data[0][2] = mat_b36->data[2][0];
  mat_b63.data[1][0] = mat_b36->data[0][1];
  mat_b63.data[1][1] = mat_b36->data[1][1];
  mat_b63.data[1][2] = mat_b36->data[2][1];
  mat_b63.data[2][0] = mat_b36->data[0][2];
  mat_b63.data[2][1] = mat_b36->data[1][2];
  mat_b63.data[2][2] = mat_b36->data[2][2];
  mat_b63.data[3][0] = mat_b36->data[0][3];
  mat_b63.data[3][1] = mat_b36->data[1][3];
  mat_b63.data[3][2] = mat_b36->data[2][3];
  mat_b63.data[4][0] = mat_b36->data[0][4];
  mat_b63.data[4][1] = mat_b36->data[1][4];
  mat_b63.data[4][2] = mat_b36->data[2][4];
  mat_b63.data[5][0] = mat_b36->data[0][5];
  mat_b63.data[5][1] = mat_b36->data[1][5];
  mat_b63.data[5][2] = mat_b36->data[2][5];
  return mat_b63;
}

MatrixB63 create_empty_matrix_B63() {
  MatrixB36 mat_b36 =
      create_matrix_B36(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  return create_matrix_B63_from_B36(&mat_b36);
}

MatrixB63 matrix_B36_transpose(MatrixB36* mat_b36) {
  MatrixB63 mat_b63 = create_matrix_B63_from_B36(mat_b36);
  return mat_b63;
}

typedef struct MatrixB612 {
  MatrixType type;
  double data[DIMENSION6][DIMENSION_C3D4];
} MatrixB612;

void matrix_B612_print(const MatrixB612* mat) {
  printf(
      "MatrixB612: [[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], "
      "[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], "
      "[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], "
      "[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], "
      "[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], "
      "[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]]\n",
      mat->data[0][0], mat->data[0][1], mat->data[0][2], mat->data[0][3],
      mat->data[0][4], mat->data[0][5], mat->data[0][6], mat->data[0][7],
      mat->data[0][8], mat->data[0][9], mat->data[0][10], mat->data[0][11],
      mat->data[1][0], mat->data[1][1], mat->data[1][2], mat->data[1][3],
      mat->data[1][4], mat->data[1][5], mat->data[1][6], mat->data[1][7],
      mat->data[1][8], mat->data[1][9], mat->data[1][10], mat->data[1][11],
      mat->data[2][0], mat->data[2][1], mat->data[2][2], mat->data[2][3],
      mat->data[2][4], mat->data[2][5], mat->data[2][6], mat->data[2][7],
      mat->data[2][8], mat->data[2][9], mat->data[2][10], mat->data[2][11],
      mat->data[3][0], mat->data[3][1], mat->data[3][2], mat->data[3][3],
      mat->data[3][4], mat->data[3][5], mat->data[3][6], mat->data[3][7],
      mat->data[3][8], mat->data[3][9], mat->data[3][10], mat->data[3][11],
      mat->data[4][0], mat->data[4][1], mat->data[4][2], mat->data[4][3],
      mat->data[4][4], mat->data[4][5], mat->data[4][6], mat->data[4][7],
      mat->data[4][8], mat->data[4][9], mat->data[4][10], mat->data[4][11],
      mat->data[5][0], mat->data[5][1], mat->data[5][2], mat->data[5][3],
      mat->data[5][4], mat->data[5][5], mat->data[5][6], mat->data[5][7],
      mat->data[5][8], mat->data[5][9], mat->data[5][10], mat->data[5][11]);
}

double matrix_B612_get_element(const MatrixB612* mat, int row, int col) {
  if (col < 0 || col >= DIMENSION_C3D4 || row < 0 || row >= DIMENSION6) {
    fatal_error("FATAL: Index out of bounds (row: %d, col: %d)!\n", row, col);
  }
  return mat->data[row][col];
}

MatrixB612 create_matrix_B612(
    double ele11, double ele12, double ele13, double ele14, double ele15,
    double ele16, double ele17, double ele18, double ele19, double ele1_10,
    double ele1_11, double ele1_12, double ele21, double ele22, double ele23,
    double ele24, double ele25, double ele26, double ele27, double ele28,
    double ele29, double ele2_10, double ele2_11, double ele2_12, double ele31,
    double ele32, double ele33, double ele34, double ele35, double ele36,
    double ele37, double ele38, double ele39, double ele3_10, double ele3_11,
    double ele3_12, double ele41, double ele42, double ele43, double ele44,
    double ele45, double ele46, double ele47, double ele48, double ele49,
    double ele4_10, double ele4_11, double ele4_12, double ele51, double ele52,
    double ele53, double ele54, double ele55, double ele56, double ele57,
    double ele58, double ele59, double ele5_10, double ele5_11, double ele5_12,
    double ele61, double ele62, double ele63, double ele64, double ele65,
    double ele66, double ele67, double ele68, double ele69, double ele6_10,
    double ele6_11, double ele6_12) {
  MatrixB612 mat;
  mat.type = MatrixB6X12;
  mat.data[0][0] = ele11;
  mat.data[0][1] = ele12;
  mat.data[0][2] = ele13;
  mat.data[0][3] = ele14;
  mat.data[0][4] = ele15;
  mat.data[0][5] = ele16;
  mat.data[0][6] = ele17;
  mat.data[0][7] = ele18;
  mat.data[0][8] = ele19;
  mat.data[0][9] = ele1_10;
  mat.data[0][10] = ele1_11;
  mat.data[0][11] = ele1_12;
  mat.data[1][0] = ele21;
  mat.data[1][1] = ele22;
  mat.data[1][2] = ele23;
  mat.data[1][3] = ele24;
  mat.data[1][4] = ele25;
  mat.data[1][5] = ele26;
  mat.data[1][6] = ele27;
  mat.data[1][7] = ele28;
  mat.data[1][8] = ele29;
  mat.data[1][9] = ele2_10;
  mat.data[1][10] = ele2_11;
  mat.data[1][11] = ele2_12;
  mat.data[2][0] = ele31;
  mat.data[2][1] = ele32;
  mat.data[2][2] = ele33;
  mat.data[2][3] = ele34;
  mat.data[2][4] = ele35;
  mat.data[2][5] = ele36;
  mat.data[2][6] = ele37;
  mat.data[2][7] = ele38;
  mat.data[2][8] = ele39;
  mat.data[2][9] = ele3_10;
  mat.data[2][10] = ele3_11;
  mat.data[2][11] = ele3_12;
  mat.data[3][0] = ele41;
  mat.data[3][1] = ele42;
  mat.data[3][2] = ele43;
  mat.data[3][3] = ele44;
  mat.data[3][4] = ele45;
  mat.data[3][5] = ele46;
  mat.data[3][6] = ele47;
  mat.data[3][7] = ele48;
  mat.data[3][8] = ele49;
  mat.data[3][9] = ele4_10;
  mat.data[3][10] = ele4_11;
  mat.data[3][11] = ele4_12;
  mat.data[4][0] = ele51;
  mat.data[4][1] = ele52;
  mat.data[4][2] = ele53;
  mat.data[4][3] = ele54;
  mat.data[4][4] = ele55;
  mat.data[4][5] = ele56;
  mat.data[4][6] = ele57;
  mat.data[4][7] = ele58;
  mat.data[4][8] = ele59;
  mat.data[4][9] = ele5_10;
  mat.data[4][10] = ele5_11;
  mat.data[4][11] = ele5_12;
  mat.data[5][0] = ele61;
  mat.data[5][1] = ele62;
  mat.data[5][2] = ele63;
  mat.data[5][3] = ele64;
  mat.data[5][4] = ele65;
  mat.data[5][5] = ele66;
  mat.data[5][6] = ele67;
  mat.data[5][7] = ele68;
  mat.data[5][8] = ele69;
  mat.data[5][9] = ele6_10;
  mat.data[5][10] = ele6_11;
  mat.data[5][11] = ele6_12;
  return mat;
}

MatrixB612 create_empty_matrix_B612() {
  return create_matrix_B612(
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

typedef struct MatrixB126 {
  MatrixType type;
  double data[DIMENSION_C3D4][DIMENSION6];
} MatrixB126;

void matrix_B126_print(const MatrixB126* mat) {
  printf(
      "MatrixB126: [[%f, %f, %f, %f, %f, %f]\n[%f, %f, %f, %f, %f, %f]\n"
      "[%f, %f, %f, %f, %f, %f]\n[%f, %f, %f, %f, %f, %f]\n"
      "[%f, %f, %f, %f, %f, %f]\n[%f, %f, %f, %f, %f, %f]\n"
      "[%f, %f, %f, %f, %f, %f]\n[%f, %f, %f, %f, %f, %f]\n"
      "[%f, %f, %f, %f, %f, %f]\n[%f, %f, %f, %f, %f, %f]\n"
      "[%f, %f, %f, %f, %f, %f]\n[%f, %f, %f, %f, %f, %f]]\n",
      mat->data[0][0], mat->data[0][1], mat->data[0][2], mat->data[0][3],
      mat->data[0][4], mat->data[0][5], mat->data[1][0], mat->data[1][1],
      mat->data[1][2], mat->data[1][3], mat->data[1][4], mat->data[1][5],
      mat->data[2][0], mat->data[2][1], mat->data[2][2], mat->data[2][3],
      mat->data[2][4], mat->data[2][5], mat->data[3][0], mat->data[3][1],
      mat->data[3][2], mat->data[3][3], mat->data[3][4], mat->data[3][5],
      mat->data[4][0], mat->data[4][1], mat->data[4][2], mat->data[4][3],
      mat->data[4][4], mat->data[4][5], mat->data[5][0], mat->data[5][1],
      mat->data[5][2], mat->data[5][3], mat->data[5][4], mat->data[5][5],
      mat->data[6][0], mat->data[6][1], mat->data[6][2], mat->data[6][3],
      mat->data[6][4], mat->data[6][5], mat->data[7][0], mat->data[7][1],
      mat->data[7][2], mat->data[7][3], mat->data[7][4], mat->data[7][5],
      mat->data[8][0], mat->data[8][1], mat->data[8][2], mat->data[8][3],
      mat->data[8][4], mat->data[8][5], mat->data[9][0], mat->data[9][1],
      mat->data[9][2], mat->data[9][3], mat->data[9][4], mat->data[9][5],
      mat->data[10][0], mat->data[10][1], mat->data[10][2], mat->data[10][3],
      mat->data[10][4], mat->data[10][5], mat->data[11][0], mat->data[11][1],
      mat->data[11][2], mat->data[11][3], mat->data[11][4], mat->data[11][5]);
}

double matrix_B126_get_element(const MatrixB126* mat, int row, int col) {
  if (col < 0 || col >= DIMENSION6 || row < 0 || row >= DIMENSION_C3D4) {
    fatal_error("FATAL: Index out of bounds (row: %d, col: %d)!\n", row, col);
  }
  return mat->data[row][col];
}

MatrixB126 create_matrix_B126_from_B612(MatrixB612* mat_b612) {
  MatrixB126 mat_b126;
  mat_b126.type = MatrixB12X6;
  mat_b126.data[0][0] = mat_b612->data[0][0];
  mat_b126.data[0][1] = mat_b612->data[1][0];
  mat_b126.data[0][2] = mat_b612->data[2][0];
  mat_b126.data[0][3] = mat_b612->data[3][0];
  mat_b126.data[0][4] = mat_b612->data[4][0];
  mat_b126.data[0][5] = mat_b612->data[5][0];
  mat_b126.data[1][0] = mat_b612->data[0][1];
  mat_b126.data[1][1] = mat_b612->data[1][1];
  mat_b126.data[1][2] = mat_b612->data[2][1];
  mat_b126.data[1][3] = mat_b612->data[3][1];
  mat_b126.data[1][4] = mat_b612->data[4][1];
  mat_b126.data[1][5] = mat_b612->data[5][1];
  mat_b126.data[2][0] = mat_b612->data[0][2];
  mat_b126.data[2][1] = mat_b612->data[1][2];
  mat_b126.data[2][2] = mat_b612->data[2][2];
  mat_b126.data[2][3] = mat_b612->data[3][2];
  mat_b126.data[2][4] = mat_b612->data[4][2];
  mat_b126.data[2][5] = mat_b612->data[5][2];
  mat_b126.data[3][0] = mat_b612->data[0][3];
  mat_b126.data[3][1] = mat_b612->data[1][3];
  mat_b126.data[3][2] = mat_b612->data[2][3];
  mat_b126.data[3][3] = mat_b612->data[3][3];
  mat_b126.data[3][4] = mat_b612->data[4][3];
  mat_b126.data[3][5] = mat_b612->data[5][3];
  mat_b126.data[4][0] = mat_b612->data[0][4];
  mat_b126.data[4][1] = mat_b612->data[1][4];
  mat_b126.data[4][2] = mat_b612->data[2][4];
  mat_b126.data[4][3] = mat_b612->data[3][4];
  mat_b126.data[4][4] = mat_b612->data[4][4];
  mat_b126.data[4][5] = mat_b612->data[5][4];
  mat_b126.data[5][0] = mat_b612->data[0][5];
  mat_b126.data[5][1] = mat_b612->data[1][5];
  mat_b126.data[5][2] = mat_b612->data[2][5];
  mat_b126.data[5][3] = mat_b612->data[3][5];
  mat_b126.data[5][4] = mat_b612->data[4][5];
  mat_b126.data[5][5] = mat_b612->data[5][5];
  mat_b126.data[6][0] = mat_b612->data[0][6];
  mat_b126.data[6][1] = mat_b612->data[1][6];
  mat_b126.data[6][2] = mat_b612->data[2][6];
  mat_b126.data[6][3] = mat_b612->data[3][6];
  mat_b126.data[6][4] = mat_b612->data[4][6];
  mat_b126.data[6][5] = mat_b612->data[5][6];
  mat_b126.data[7][0] = mat_b612->data[0][7];
  mat_b126.data[7][1] = mat_b612->data[1][7];
  mat_b126.data[7][2] = mat_b612->data[2][7];
  mat_b126.data[7][3] = mat_b612->data[3][7];
  mat_b126.data[7][4] = mat_b612->data[4][7];
  mat_b126.data[7][5] = mat_b612->data[5][7];
  mat_b126.data[8][0] = mat_b612->data[0][8];
  mat_b126.data[8][1] = mat_b612->data[1][8];
  mat_b126.data[8][2] = mat_b612->data[2][8];
  mat_b126.data[8][3] = mat_b612->data[3][8];
  mat_b126.data[8][4] = mat_b612->data[4][8];
  mat_b126.data[8][5] = mat_b612->data[5][8];
  mat_b126.data[9][0] = mat_b612->data[0][9];
  mat_b126.data[9][1] = mat_b612->data[1][9];
  mat_b126.data[9][2] = mat_b612->data[2][9];
  mat_b126.data[9][3] = mat_b612->data[3][9];
  mat_b126.data[9][4] = mat_b612->data[4][9];
  mat_b126.data[9][5] = mat_b612->data[5][9];
  mat_b126.data[10][0] = mat_b612->data[0][10];
  mat_b126.data[10][1] = mat_b612->data[1][10];
  mat_b126.data[10][2] = mat_b612->data[2][10];
  mat_b126.data[10][3] = mat_b612->data[3][10];
  mat_b126.data[10][4] = mat_b612->data[4][10];
  mat_b126.data[10][5] = mat_b612->data[5][10];
  mat_b126.data[11][0] = mat_b612->data[0][11];
  mat_b126.data[11][1] = mat_b612->data[1][11];
  mat_b126.data[11][2] = mat_b612->data[2][11];
  mat_b126.data[11][3] = mat_b612->data[3][11];
  mat_b126.data[11][4] = mat_b612->data[4][11];
  mat_b126.data[11][5] = mat_b612->data[5][11];
  return mat_b126;
}

MatrixB126 matrix_B612_transpose(MatrixB612* mat_b612) {
  MatrixB126 mat_b126 = create_matrix_B126_from_B612(mat_b612);
  return mat_b126;
}

// Matrix of CPS3 Element stiffness matrix (6x6)
typedef struct Matrix6D {
  MatrixType type;
  double data[DIMENSION_CPS3][DIMENSION_CPS3];
} Matrix6D;

void matrix_6D_print(const Matrix6D* mat) {
  printf(
      "Matrix6D (CPS3 element stiffness matrix): [[%f, %f, %f, %f, %f, %f], "
      "[%f, %f, %f, %f, %f, %f], [%f, %f, %f, %f, %f, %f], [%f, %f, %f, %f, "
      "%f, %f], [%f, %f, %f, %f, %f, %f], [%f, %f, %f, %f, %f, %f]\n",
      mat->data[0][0], mat->data[0][1], mat->data[0][2], mat->data[0][3],
      mat->data[0][4], mat->data[0][5], mat->data[1][0], mat->data[1][1],
      mat->data[1][2], mat->data[1][3], mat->data[1][4], mat->data[1][5],
      mat->data[2][0], mat->data[2][1], mat->data[2][2], mat->data[2][3],
      mat->data[2][4], mat->data[2][5], mat->data[3][0], mat->data[3][1],
      mat->data[3][2], mat->data[3][3], mat->data[3][4], mat->data[3][5],
      mat->data[4][0], mat->data[4][1], mat->data[4][2], mat->data[4][3],
      mat->data[4][4], mat->data[4][5], mat->data[5][0], mat->data[5][1],
      mat->data[5][2], mat->data[5][3], mat->data[5][4], mat->data[5][5]);
}

Matrix6D create_matrix_6D(
    double ele11, double ele12, double ele13, double ele14, double ele15,
    double ele16, double ele21, double ele22, double ele23, double ele24,
    double ele25, double ele26, double ele31, double ele32, double ele33,
    double ele34, double ele35, double ele36, double ele41, double ele42,
    double ele43, double ele44, double ele45, double ele46, double ele51,
    double ele52, double ele53, double ele54, double ele55, double ele56,
    double ele61, double ele62, double ele63, double ele64, double ele65,
    double ele66) {
  Matrix6D mat;
  mat.type = Matrix6X6;
  mat.data[0][0] = ele11;
  mat.data[0][1] = ele12;
  mat.data[0][2] = ele13;
  mat.data[0][3] = ele14;
  mat.data[0][4] = ele15;
  mat.data[0][5] = ele16;
  mat.data[1][0] = ele21;
  mat.data[1][1] = ele22;
  mat.data[1][2] = ele23;
  mat.data[1][3] = ele24;
  mat.data[1][4] = ele25;
  mat.data[1][5] = ele26;
  mat.data[2][0] = ele31;
  mat.data[2][1] = ele32;
  mat.data[2][2] = ele33;
  mat.data[2][3] = ele34;
  mat.data[2][4] = ele35;
  mat.data[2][5] = ele36;
  mat.data[3][0] = ele41;
  mat.data[3][1] = ele42;
  mat.data[3][2] = ele43;
  mat.data[3][3] = ele44;
  mat.data[3][4] = ele45;
  mat.data[3][5] = ele46;
  mat.data[4][0] = ele51;
  mat.data[4][1] = ele52;
  mat.data[4][2] = ele53;
  mat.data[4][3] = ele54;
  mat.data[4][4] = ele55;
  mat.data[4][5] = ele56;
  mat.data[5][0] = ele61;
  mat.data[5][1] = ele62;
  mat.data[5][2] = ele63;
  mat.data[5][3] = ele64;
  mat.data[5][4] = ele65;
  mat.data[5][5] = ele66;
  return mat;
}

Matrix6D create_empty_matrix_6D() {
  return create_matrix_6D(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

double matrix_6D_get_element(const Matrix6D* mat, int row, int col) {
  if (col < 0 || col >= DIMENSION_CPS3 || row < 0 || row >= DIMENSION_CPS3) {
    fatal_error("FATAL: Index out of bounds (row: %d, col: %d)!\n", row, col);
  }
  return mat->data[row][col];
}

void matrix_6D_fill_abaqus_double_array(const Matrix6D* mat, double* dst) {
  // i loops over rows
  for (int i = 0; i < DIMENSION_CPS3; ++i) {
    // j loops over cols
    for (int j = 0; j < DIMENSION_CPS3; ++j) {
      dst[i * DIMENSION_CPS3 + j] = matrix_6D_get_element(mat, i, j);
    }
  }
}

// Vector 6 x 1 used by CPS3NodalInfo
typedef struct Vector6D {
  MatrixType type;
  double data[DIMENSION_CPS3];
} Vector6D;

void vector_6D_print(const Vector6D* mat) {
  printf("Vector6D: [%f, %f, %f, %f, %f, %f]\n", mat->data[0], mat->data[1],
         mat->data[2], mat->data[3], mat->data[4], mat->data[5]);
}

double vector_6D_get_element(const Vector6D* mat, int row) {
  if (row < 0 || row >= DIMENSION_CPS3) {
    fatal_error("FATAL: Index out of bounds (row: %d)!\n", row);
  }
  return mat->data[row];
}

Vector6D create_vector_6D(double ele11, double ele21, double ele31,
                          double ele41, double ele51, double ele61) {
  Vector6D vec;
  vec.type = Vector6X1;
  vec.data[0] = ele11;
  vec.data[1] = ele21;
  vec.data[2] = ele31;
  vec.data[3] = ele41;
  vec.data[4] = ele51;
  vec.data[5] = ele61;
  return vec;
}

Vector6D create_empty_vector_6D() { return create_vector_6D(0, 0, 0, 0, 0, 0); }

void vector_6D_fill_abaqus_double_array(const Vector6D* vec, double* dst) {
  for (int i = 0; i < DIMENSION_CPS3; ++i) {
    dst[i] = vector_6D_get_element(vec, i);
  }
}

// Vector 12 x 1 -- use for basic C3D4 vector
typedef struct Vector12D {
  MatrixType type;
  double data[DIMENSION_C3D4];
} Vector12D;

void vector_12D_print(const Vector12D* vec) {
  printf("Vector12D: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]\n",
         vec->data[0], vec->data[1], vec->data[2], vec->data[3], vec->data[4],
         vec->data[5], vec->data[6], vec->data[7], vec->data[8], vec->data[9],
         vec->data[10], vec->data[11]);
}

double vector_12D_get_element(const Vector12D* vec, int idx) {
  if (idx < 0 || idx >= DIMENSION_C3D4) {
    fatal_error("FATAL: Index out of bounds (index: %d)!", idx);
  }
  return vec->data[idx];
}

Vector12D create_vector_12D(double ele1, double ele2, double ele3, double ele4,
                            double ele5, double ele6, double ele7, double ele8,
                            double ele9, double ele10, double ele11,
                            double ele12) {
  Vector12D vec;
  vec.type = Vector12X1;
  vec.data[0] = ele1;
  vec.data[1] = ele2;
  vec.data[2] = ele3;
  vec.data[3] = ele4;
  vec.data[4] = ele5;
  vec.data[5] = ele6;
  vec.data[6] = ele7;
  vec.data[7] = ele8;
  vec.data[8] = ele9;
  vec.data[9] = ele10;
  vec.data[10] = ele11;
  vec.data[11] = ele12;

  return vec;
}

Vector12D create_empty_vector_12D() {
  return create_vector_12D(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

// Mathematica matrix operator (transpose add minus num_mul)
// Avoid use loop to be brief as to low dimension (<=3)
Matrix2D matrix_2D_transpose(const Matrix2D* mat) {
  return create_matrix_2D(mat->data[0][0], mat->data[1][0], mat->data[0][1],
                          mat->data[1][1]);
}

Matrix3D matrix_3D_transpose(const Matrix3D* mat) {
  return create_matrix_3D(mat->data[0][0], mat->data[1][0], mat->data[2][0],
                          mat->data[0][1], mat->data[1][1], mat->data[2][1],
                          mat->data[0][2], mat->data[1][2], mat->data[2][2]);
}

Matrix6D matrix_6D_transpose(const Matrix6D* mat) {
  return create_matrix_6D(
      mat->data[0][0], mat->data[1][0], mat->data[2][0], mat->data[3][0],
      mat->data[4][0], mat->data[5][0], mat->data[0][1], mat->data[1][1],
      mat->data[2][1], mat->data[3][1], mat->data[4][1], mat->data[5][1],
      mat->data[0][2], mat->data[1][2], mat->data[2][2], mat->data[3][2],
      mat->data[4][2], mat->data[5][2], mat->data[0][3], mat->data[1][3],
      mat->data[2][3], mat->data[3][3], mat->data[4][3], mat->data[5][3],
      mat->data[0][4], mat->data[1][4], mat->data[2][4], mat->data[3][4],
      mat->data[4][4], mat->data[5][4], mat->data[0][5], mat->data[1][5],
      mat->data[2][5], mat->data[3][5], mat->data[4][5], mat->data[5][5]);
}

Matrix2D matrix_2D_add(const Matrix2D* matrix_a, const Matrix2D* matrix_b) {
  return create_matrix_2D(matrix_a->data[0][0] + matrix_b->data[0][0],
                          matrix_a->data[0][1] + matrix_b->data[0][1],
                          matrix_a->data[1][0] + matrix_b->data[1][0],
                          matrix_a->data[1][1] + matrix_b->data[1][1]);
}

Matrix2D matrix_2D_number_multiplication(double num, const Matrix2D* mat) {
  return create_matrix_2D(num * mat->data[0][0], num * mat->data[0][1],
                          num * mat->data[1][0], num * mat->data[1][1]);
}

Matrix2D matrix_2D_negative(const Matrix2D* mat) {
  return matrix_2D_number_multiplication((double)-1, mat);
}

Matrix2D matrix_2D_minus(const Matrix2D* matrix_a, const Matrix2D* matrix_b) {
  const Matrix2D negative_matrix_b = matrix_2D_negative(matrix_b);
  return matrix_2D_add(matrix_a, &negative_matrix_b);
}

Matrix3D matrix_3D_add(const Matrix3D* matrix_a, const Matrix3D* matrix_b) {
  return create_matrix_3D(matrix_a->data[0][0] + matrix_b->data[0][0],
                          matrix_a->data[0][1] + matrix_b->data[0][1],
                          matrix_a->data[0][2] + matrix_b->data[0][2],
                          matrix_a->data[1][0] + matrix_b->data[1][0],
                          matrix_a->data[1][1] + matrix_b->data[1][1],
                          matrix_a->data[1][2] + matrix_b->data[1][2],
                          matrix_a->data[2][0] + matrix_b->data[2][0],
                          matrix_a->data[2][1] + matrix_b->data[2][1],
                          matrix_a->data[2][2] + matrix_b->data[2][2]);
}

Matrix3D matrix_3D_number_multiplication(double num, const Matrix3D* mat) {
  return create_matrix_3D(
      num * mat->data[0][0], num * mat->data[0][1], num * mat->data[0][2],
      num * mat->data[1][0], num * mat->data[1][1], num * mat->data[1][2],
      num * mat->data[2][0], num * mat->data[2][1], num * mat->data[2][2]);
}

Matrix3D matrix_3D_negative(const Matrix3D* mat) {
  return matrix_3D_number_multiplication((double)-1, mat);
}

Matrix3D matrix_3D_minus(const Matrix3D* matrix_a, const Matrix3D* matrix_b) {
  const Matrix3D negative_matrix_b = matrix_3D_negative(matrix_b);
  return matrix_3D_add(matrix_a, &negative_matrix_b);
}

Matrix6D matrix_6D_add(const Matrix6D* matrix_a, const Matrix6D* matrix_b) {
  return create_matrix_6D(matrix_a->data[0][0] + matrix_b->data[0][0],
                          matrix_a->data[0][1] + matrix_b->data[0][1],
                          matrix_a->data[0][2] + matrix_b->data[0][2],
                          matrix_a->data[0][3] + matrix_b->data[0][3],
                          matrix_a->data[0][4] + matrix_b->data[0][4],
                          matrix_a->data[0][5] + matrix_b->data[0][5],
                          matrix_a->data[1][0] + matrix_b->data[1][0],
                          matrix_a->data[1][1] + matrix_b->data[1][1],
                          matrix_a->data[1][2] + matrix_b->data[1][2],
                          matrix_a->data[1][3] + matrix_b->data[1][3],
                          matrix_a->data[1][4] + matrix_b->data[1][4],
                          matrix_a->data[1][5] + matrix_b->data[1][5],
                          matrix_a->data[2][0] + matrix_b->data[2][0],
                          matrix_a->data[2][1] + matrix_b->data[2][1],
                          matrix_a->data[2][2] + matrix_b->data[2][2],
                          matrix_a->data[2][3] + matrix_b->data[2][3],
                          matrix_a->data[2][4] + matrix_b->data[2][4],
                          matrix_a->data[2][5] + matrix_b->data[2][5],
                          matrix_a->data[3][0] + matrix_b->data[3][0],
                          matrix_a->data[3][1] + matrix_b->data[3][1],
                          matrix_a->data[3][2] + matrix_b->data[3][2],
                          matrix_a->data[3][3] + matrix_b->data[3][3],
                          matrix_a->data[3][4] + matrix_b->data[3][4],
                          matrix_a->data[3][5] + matrix_b->data[3][5],
                          matrix_a->data[4][0] + matrix_b->data[4][0],
                          matrix_a->data[4][1] + matrix_b->data[4][1],
                          matrix_a->data[4][2] + matrix_b->data[4][2],
                          matrix_a->data[4][3] + matrix_b->data[4][3],
                          matrix_a->data[4][4] + matrix_b->data[4][4],
                          matrix_a->data[4][5] + matrix_b->data[4][5],
                          matrix_a->data[5][0] + matrix_b->data[5][0],
                          matrix_a->data[5][1] + matrix_b->data[5][1],
                          matrix_a->data[5][2] + matrix_b->data[5][2],
                          matrix_a->data[5][3] + matrix_b->data[5][3],
                          matrix_a->data[5][4] + matrix_b->data[5][4],
                          matrix_a->data[5][5] + matrix_b->data[5][5]);
}

Matrix6D matrix_6D_number_multiplication(double num, const Matrix6D* mat) {
  return create_matrix_6D(
      num * mat->data[0][0], num * mat->data[0][1], num * mat->data[0][2],
      num * mat->data[0][3], num * mat->data[0][4], num * mat->data[0][5],
      num * mat->data[1][0], num * mat->data[1][1], num * mat->data[1][2],
      num * mat->data[1][3], num * mat->data[1][4], num * mat->data[1][5],
      num * mat->data[2][0], num * mat->data[2][1], num * mat->data[2][2],
      num * mat->data[2][3], num * mat->data[2][4], num * mat->data[2][5],
      num * mat->data[3][0], num * mat->data[3][1], num * mat->data[3][2],
      num * mat->data[3][3], num * mat->data[3][4], num * mat->data[3][5],
      num * mat->data[4][0], num * mat->data[4][1], num * mat->data[4][2],
      num * mat->data[4][3], num * mat->data[4][4], num * mat->data[4][5],
      num * mat->data[5][0], num * mat->data[5][1], num * mat->data[5][2],
      num * mat->data[5][3], num * mat->data[5][4], num * mat->data[5][5]);
}

Matrix6D matrix_6D_negative(const Matrix6D* mat) {
  return matrix_6D_number_multiplication(-1, mat);
}

Matrix6D matrix_6D_minus(const Matrix6D* matrix_a, const Matrix6D* matrix_b) {
  Matrix6D minus_matrix_b = matrix_6D_negative(matrix_b);
  return matrix_6D_add(matrix_a, &minus_matrix_b);
}

Vector2D vector_2D_add(const Vector2D* vector_a, const Vector2D* vector_b) {
  return create_vector_2D(vector_a->data[0] + vector_b->data[0],
                          vector_a->data[1] + vector_b->data[1]);
}

Vector2D vector_2D_number_multiplication(double num, const Vector2D* vec) {
  return create_vector_2D(num * vec->data[0], num * vec->data[1]);
}

Vector2D vector_2D_negative(const Vector2D* mat) {
  return vector_2D_number_multiplication((double)-1, mat);
}

Vector2D vector_2D_minus(const Vector2D* matrix_a, const Vector2D* matrix_b) {
  const Vector2D negative_matrix_b = vector_2D_negative(matrix_b);
  return vector_2D_add(matrix_a, &negative_matrix_b);
}

Vector3D vector_3D_add(const Vector3D* vector_a, const Vector3D* vector_b) {
  return create_vector_3D(vector_a->data[0] + vector_b->data[0],
                          vector_a->data[1] + vector_b->data[1],
                          vector_a->data[2] + vector_b->data[2]);
}

Vector3D vector_3D_number_multiplication(double num, const Vector3D* vec) {
  return create_vector_3D(num * vec->data[0], num * vec->data[1],
                          num * vec->data[2]);
}

Vector3D vector_3D_negative(const Vector3D* mat) {
  return vector_3D_number_multiplication((double)-1, mat);
}

Vector3D vector_3D_minus(const Vector3D* matrix_a, const Vector3D* matrix_b) {
  const Vector3D negative_matrix_b = vector_3D_negative(matrix_b);
  return vector_3D_add(matrix_a, &negative_matrix_b);
}

Vector6D vector_6D_add(const Vector6D* vector_a, const Vector6D* vector_b) {
  return create_vector_6D(vector_a->data[0] + vector_b->data[0],
                          vector_a->data[1] + vector_b->data[1],
                          vector_a->data[2] + vector_b->data[2],
                          vector_a->data[3] + vector_b->data[3],
                          vector_a->data[4] + vector_b->data[4],
                          vector_a->data[5] + vector_b->data[5]);
}

Vector6D vector_6D_number_multiplication(double num, const Vector6D* vec) {
  return create_vector_6D(num * vec->data[0], num * vec->data[1],
                          num * vec->data[2], num * vec->data[3],
                          num * vec->data[4], num * vec->data[5]);
}

Vector6D vector_6D_negative(const Vector6D* mat) {
  return vector_6D_number_multiplication((double)-1, mat);
}

Vector6D vector_6D_minus(const Vector6D* matrix_a, const Vector6D* matrix_b) {
  const Vector6D negative_matrix_b = vector_6D_negative(matrix_b);
  return vector_6D_add(matrix_a, &negative_matrix_b);
}

// matrix vector multiplication and inverse
Matrix2D matrix_2D_mul_matrix_2D(const Matrix2D* matrix_a,
                                 const Matrix2D* matrix_b) {
  Matrix2D result = create_empty_matrix_2D();
  for (int i = 0; i < DIMENSION2; ++i) {
    for (int j = 0; j < DIMENSION2; ++j) {
      for (int k = 0; k < DIMENSION2; ++k) {
        result.data[i][j] += matrix_a->data[i][k] * matrix_b->data[k][j];
      }
    }
  }
  return result;
}

Matrix3D matrix_3D_mul_matrix_3D(const Matrix3D* matrix_a,
                                 const Matrix3D* matrix_b) {
  Matrix3D result = create_empty_matrix_3D();
  for (int i = 0; i < DIMENSION3; ++i) {
    for (int j = 0; j < DIMENSION3; ++j) {
      for (int k = 0; k < DIMENSION3; ++k) {
        result.data[i][j] += matrix_a->data[i][k] * matrix_b->data[k][j];
      }
    }
  }
  return result;
}

Vector2D matrix_2D_mul_vector_2D(const Matrix2D* mat, const Vector2D* vec) {
  Vector2D result = create_empty_vector_2D();
  for (int i = 0; i < DIMENSION2; ++i) {
    for (int j = 0; j < DIMENSION2; ++j) {
      result.data[i] += mat->data[i][j] * vec->data[j];
    }
  }
  return result;
}

Vector3D matrix_3D_mul_vector_3D(const Matrix3D* mat, const Vector3D* vec) {
  Vector3D result = create_empty_vector_3D();
  for (int i = 0; i < DIMENSION3; ++i) {
    for (int j = 0; j < DIMENSION3; ++j) {
      result.data[i] += mat->data[i][j] * vec->data[j];
    }
  }
  return result;
}

double matrix_2D_determinant(const Matrix2D* mat) {
  return mat->data[0][0] * mat->data[1][1] - mat->data[0][1] * mat->data[1][0];
}

Matrix2D matrix_2D_inverse(const Matrix2D* mat) {
  double det = matrix_2D_determinant(mat);
  if (det < ZERO_TOLERANCE && det > -ZERO_TOLERANCE) {
    fatal_error("FATAL: Matrix is singular and cannot be inverted. det = %f\n",
                det);
  }

  Matrix2D result;
  result.type = Matrix2X2;
  result.data[0][0] = mat->data[1][1] / det;
  result.data[0][1] = -mat->data[0][1] / det;
  result.data[1][0] = -mat->data[1][0] / det;
  result.data[1][1] = mat->data[0][0] / det;

  return result;
}

double matrix_3D_determinant(const Matrix3D* mat) {
  double a = mat->data[0][0], b = mat->data[0][1], c = mat->data[0][2];
  double d = mat->data[1][0], e = mat->data[1][1], f = mat->data[1][2];
  double g = mat->data[2][0], h = mat->data[2][1], i = mat->data[2][2];

  return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

Matrix3D matrix_3D_inverse(const Matrix3D* mat) {
  double det = matrix_3D_determinant(mat);
  if (det < ZERO_TOLERANCE && det > -ZERO_TOLERANCE) {
    fatal_error("FATAL: Matrix is singular and cannot be inverted. det = %f\n",
                det);
  }

  Matrix3D adj;
  adj.type = Matrix3X3;

  for (int i = 0; i < DIMENSION3; ++i) {
    for (int j = 0; j < DIMENSION3; ++j) {
      double minor[DIMENSION2][DIMENSION2];
      int m = 0, n = 0;

      for (int r = 0; r < DIMENSION3; ++r) {
        for (int c = 0; c < DIMENSION3; ++c) {
          if (r != i && c != j) {
            minor[m][n] = mat->data[r][c];
            n++;
            if (n == DIMENSION2) {
              n = 0;
              m++;
            }
          }
        }
      }

      double cofactor = minor[0][0] * minor[1][1] - minor[0][1] * minor[1][0];
      adj.data[i][j] = ((i + j) % 2 == 0 ? 1 : -1) * cofactor;
    }
  }

  Matrix3D result;
  result.type = Matrix3X3;
  for (int r = 0; r < DIMENSION3; ++r) {
    for (int c = 0; c < DIMENSION3; ++c) {
      result.data[r][c] = adj.data[r][c] / det;
    }
  }

  return matrix_3D_transpose(&result);
}

double matrix_6D_determinant(const Matrix6D* const mat) {
  // 初始化排列为 {0, 1, 2, 3, 4, 5}
  int perm[DIMENSION_CPS3] = {0, 1, 2, 3, 4, 5};
  double det = 0.0;

  while (true) {
    // 计算当前排列对应的乘积
    double prod = 1.0;
    for (int i = 0; i < DIMENSION_CPS3; i++) {
      prod *= mat->data[i][perm[i]];
    }

    // 计算排列的逆序数以确定排列的符号
    int inv = 0;
    for (int i = 0; i < DIMENSION_CPS3; i++) {
      for (int j = i + 1; j < DIMENSION_CPS3; j++) {
        if (perm[i] > perm[j]) inv++;
      }
    }
    double sign = (inv % 2 == 0) ? 1.0 : -1.0;
    det += sign * prod;

    // 生成下一个排列（字典序排列算法）
    int i;
    for (i = DIMENSION_CPS3 - 2; i >= 0; i--) {
      if (perm[i] < perm[i + 1]) break;
    }
    if (i < 0)  // 已经遍历完所有排列
      break;

    int j;
    for (j = DIMENSION_CPS3 - 1; j > i; j--) {
      if (perm[j] > perm[i]) break;
    }
    // 交换 perm[i] 与 perm[j]
    int temp = perm[i];
    perm[i] = perm[j];
    perm[j] = temp;

    // 反转 perm[i+1...DIMENSION_CPS3-1]
    for (int k = i + 1, l = DIMENSION_CPS3 - 1; k < l; k++, l--) {
      temp = perm[k];
      perm[k] = perm[l];
      perm[l] = temp;
    }
  }
  return det;
}

Vector6D matrix_6D_mul_vector_6D(const Matrix6D* mat, const Vector6D* vec) {
  Vector6D result = create_empty_vector_6D();
  for (int i = 0; i < DIMENSION_CPS3; i++) {
    for (int j = 0; j < DIMENSION_CPS3; j++) {
      result.data[i] += mat->data[i][j] * vec->data[j];
    }
  }
  return result;
}

Matrix6D matrix_6D_mul_matrix_6D(const Matrix6D* mat1, const Matrix6D* mat2) {
  Matrix6D result = create_empty_matrix_6D();
  for (int i = 0; i < DIMENSION_CPS3; i++) {
    for (int j = 0; j < DIMENSION_CPS3; j++) {
      for (int k = 0; k < DIMENSION_CPS3; k++) {
        result.data[i][j] += mat1->data[i][k] * mat2->data[k][j];
      }
    }
  }
  return result;
}

// voigt vector and matrix transform
Vector3D voigt_2D_matrix_to_3D_vector(const Matrix2D* mat) {
  double sym_delta = mat->data[0][1] - mat->data[1][0];
  if (sym_delta > ZERO_TOLERANCE || sym_delta < -ZERO_TOLERANCE) {
    fatal_error("FATAL: only symmetric matrix can use voigt transformation.\n");
  }

  return create_vector_3D(mat->data[0][0], mat->data[1][1], mat->data[0][1]);
}

Matrix2D voigt_3D_vector_to_2D_matrix(const Vector3D* vec) {
  return create_matrix_2D(vec->data[0], vec->data[2], vec->data[2],
                          vec->data[1]);
}

Vector6D voigt_3D_matrix_to_6D_vector(const Matrix3D* mat) {
  // matrix symmetry check
  double sym_delta01 = mat->data[0][1] - mat->data[1][0];
  double sym_delta02 = mat->data[0][2] - mat->data[2][0];
  double sym_delta12 = mat->data[1][2] - mat->data[2][1];
  if (!((sym_delta01 <= ZERO_TOLERANCE) && (sym_delta01 >= -ZERO_TOLERANCE)) ||
      !((sym_delta02 <= ZERO_TOLERANCE) && (sym_delta02 >= -ZERO_TOLERANCE)) ||
      !((sym_delta12 <= ZERO_TOLERANCE) && (sym_delta12 >= -ZERO_TOLERANCE))) {
    fatal_error("FATAL: only symmetric matrix can use voigt transformation.\n");
  }

  // assemble voigt vector
  return create_vector_6D(mat->data[0][0],   // xx
                          mat->data[1][1],   // yy
                          mat->data[2][2],   // zz
                          mat->data[1][2],   // yz
                          mat->data[0][2],   // xz
                          mat->data[0][1]);  // xy
}

Matrix3D voigt_6D_vector_to_3D_matrix(const Vector6D* vec) {
  // [0]: xx, [1]: yy, [2]: zz, [3]: yz, [4]: xz, [5]: xy
  return create_matrix_3D(vec->data[0], vec->data[5], vec->data[4],
                          vec->data[5], vec->data[1], vec->data[3],
                          vec->data[4], vec->data[3], vec->data[2]);
}

//  mechanics method for UEL development
typedef struct CPS3NodalInfo {
  double node1_dof1;
  double node1_dof2;
  double node2_dof1;
  double node2_dof2;
  double node3_dof1;
  double node3_dof2;
} CPS3NodalInfo;

CPS3NodalInfo create_CPS3_nodal_info(double node1_dof1, double node1_dof2,
                                     double node2_dof1, double node2_dof2,
                                     double node3_dof1, double node3_dof2) {
  CPS3NodalInfo info;
  info.node1_dof1 = node1_dof1;
  info.node1_dof2 = node1_dof2;
  info.node2_dof1 = node2_dof1;
  info.node2_dof2 = node2_dof2;
  info.node3_dof1 = node3_dof1;
  info.node3_dof2 = node3_dof2;
  return info;
}

void print_CPS3_nodal_info(const CPS3NodalInfo* info) {
  printf(
      "CPS3 nodal info:\nnode1: [dof1 = [%f], dof2 = [%f]]\n"
      "node2: [dof1 = [%f], dof2 = [%f]]\nnode3: [dof1 = [%f], dof2 = [%f]]\n",
      info->node1_dof1, info->node1_dof2, info->node2_dof1, info->node2_dof2,
      info->node3_dof1, info->node3_dof2);
}

CPS3NodalInfo CPS3_nodal_info_add(const CPS3NodalInfo* info1,
                                  const CPS3NodalInfo* info2) {
  return create_CPS3_nodal_info(info1->node1_dof1 + info2->node1_dof1,
                                info1->node1_dof2 + info2->node1_dof2,
                                info1->node2_dof1 + info2->node2_dof1,
                                info1->node2_dof2 + info2->node2_dof2,
                                info1->node3_dof1 + info2->node3_dof1,
                                info1->node3_dof2 + info2->node3_dof2);
}

Vector6D CPS3_nodal_info_to_vector_6D(const CPS3NodalInfo* info) {
  return create_vector_6D(info->node1_dof1, info->node1_dof2, info->node2_dof1,
                          info->node2_dof2, info->node3_dof1, info->node3_dof2);
}

CPS3NodalInfo vector_6D_to_CPS3_nodal_info(const Vector6D* vec) {
  return create_CPS3_nodal_info(
      vector_6D_get_element(vec, 0), vector_6D_get_element(vec, 1),
      vector_6D_get_element(vec, 2), vector_6D_get_element(vec, 3),
      vector_6D_get_element(vec, 4), vector_6D_get_element(vec, 5));
}

double compute_CPS3_element_square(const CPS3NodalInfo* coords) {
  double x1 = coords->node1_dof1;
  double y1 = coords->node1_dof2;
  double x2 = coords->node2_dof1;
  double y2 = coords->node2_dof2;
  double x3 = coords->node3_dof1;
  double y3 = coords->node3_dof2;
  double square = 0.5 * ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1));

  if (square < 0) {
    print_CPS3_nodal_info(coords);
    fatal_error(
        "FATAL: node not in counter-clockwise arrangement while computing "
        "square.\n");
  }

  if (square == 0) {
    print_CPS3_nodal_info(coords);
    fatal_error("FATAL: three nodes of element in one line, square = 0!\n");
  }

  return square;
}

Matrix2D CPS3_nodal_disp_to_2D_F(const CPS3NodalInfo* X,
                                 const CPS3NodalInfo* u) {
  double X1 = X->node1_dof1;
  double Y1 = X->node1_dof2;
  double X2 = X->node2_dof1;
  double Y2 = X->node2_dof2;
  double X3 = X->node3_dof1;
  double Y3 = X->node3_dof2;

  double u1 = u->node1_dof1;
  double v1 = u->node1_dof2;
  double u2 = u->node2_dof1;
  double v2 = u->node2_dof2;
  double u3 = u->node3_dof1;
  double v3 = u->node3_dof2;

  Matrix2D pupX_part_1 = create_matrix_2D(u1 - u3, u2 - u3, v1 - v3, v2 - v3);
  Matrix2D pupX_part_2 = create_matrix_2D(X1 - X3, X2 - X3, Y1 - Y3, Y2 - Y3);
  Matrix2D pupX_part_2_inv = matrix_2D_inverse(&pupX_part_2);
  Matrix2D pupX = matrix_2D_mul_matrix_2D(&pupX_part_1, &pupX_part_2_inv);
  Matrix2D I = create_eye_matrix_2D();
  Matrix2D F = matrix_2D_add(&I, &pupX);

  return F;
}

Matrix2D CPS3_2D_F_to_2D_E(const Matrix2D* F) {
  Matrix2D FT = matrix_2D_transpose(F);
  Matrix2D right_cauchy_green_C = matrix_2D_mul_matrix_2D(&FT, F);
  Matrix2D I = create_eye_matrix_2D();
  Matrix2D E_times_2 = matrix_2D_minus(&right_cauchy_green_C, &I);
  Matrix2D E = matrix_2D_number_multiplication(0.5, &E_times_2);

  return E;
}

Matrix2D CPS3_tensor_strain_to_engineering_strain(
    const Matrix2D* tensor_strain) {
  return create_matrix_2D(matrix_2D_get_element(tensor_strain, 0, 0),
                          2 * matrix_2D_get_element(tensor_strain, 0, 1),
                          2 * matrix_2D_get_element(tensor_strain, 1, 0),
                          matrix_2D_get_element(tensor_strain, 1, 1));
}

Matrix2D CPS3_2D_strain_to_2D_stress(const Matrix2D* E,
                                     const Matrix3D* property) {
  Vector3D vector_E = voigt_2D_matrix_to_3D_vector(E);
  Vector3D T = matrix_3D_mul_vector_3D(property, &vector_E);
  return voigt_3D_vector_to_2D_matrix(&T);
}

Matrix2D CPS3_T_and_F_to_Cauchy(const Matrix2D* T, const Matrix2D* F) {
  double J = matrix_2D_determinant(F);
  Matrix2D FT = matrix_2D_transpose(F);
  Matrix2D F_times_T = matrix_2D_mul_matrix_2D(F, T);
  Matrix2D F_times_T_times_FT = matrix_2D_mul_matrix_2D(&F_times_T, &FT);
  Matrix2D sigma =
      matrix_2D_number_multiplication((1 / J), &F_times_T_times_FT);
  return sigma;
}

double CPS3_E_and_T_to_strain_energy_density(const Matrix2D* E,
                                             const Matrix2D* T) {
  double E11 = matrix_2D_get_element(E, 0, 0),
         E12 = matrix_2D_get_element(E, 0, 1),
         E21 = matrix_2D_get_element(E, 1, 0),
         E22 = matrix_2D_get_element(E, 1, 1);
  double T11 = matrix_2D_get_element(T, 0, 0),
         T12 = matrix_2D_get_element(T, 0, 1),
         T21 = matrix_2D_get_element(T, 1, 0),
         T22 = matrix_2D_get_element(T, 1, 1);
  double strain_energy_density =
      0.5 * (T11 * E11 + T12 * E12 + T21 * E21 + T22 * E22);
  return strain_energy_density;
}

// CPS3 stiffness matrix operator
MatrixB36 CPS3_compute_matrix_B(const CPS3NodalInfo* coords) {
  double x1 = coords->node1_dof1, y1 = coords->node1_dof2,
         x2 = coords->node2_dof1, y2 = coords->node2_dof2,
         x3 = coords->node3_dof1, y3 = coords->node3_dof2;
  double A = compute_CPS3_element_square(coords);
  double dN1dx = (y2 - y3) / (2 * A);
  double dN1dy = (x3 - x2) / (2 * A);
  double dN2dx = (y3 - y1) / (2 * A);
  double dN2dy = (x1 - x3) / (2 * A);
  double dN3dx = (y1 - y2) / (2 * A);
  double dN3dy = (x2 - x1) / (2 * A);

  return create_matrix_B36(dN1dx, 0, dN2dx, 0, dN3dx, 0, 0, dN1dy, 0, dN2dy, 0,
                           dN3dy, dN1dy, dN1dx, dN2dy, dN2dx, dN3dy, dN3dx);
}

MatrixB63 CPS3_matrixB63_mul_matrix_3D(const MatrixB63* matrix_BT,
                                       const Matrix3D* matrix_D) {
  MatrixB63 result = create_empty_matrix_B63();
  for (int i = 0; i < DIMENSION_CPS3; ++i) {
    for (int j = 0; j < DIMENSION3; ++j) {
      result.data[i][j] = 0.0;
      for (int k = 0; k < DIMENSION3; ++k) {
        result.data[i][j] += matrix_BT->data[i][k] * matrix_D->data[k][j];
      }
    }
  }
  return result;
}

Matrix6D CPS3_matrixB63_mul_matrix_B36(const MatrixB63* matrix_BTD,
                                       const MatrixB36* matrix_B) {
  Matrix6D result = create_empty_matrix_6D();

  for (int i = 0; i < DIMENSION_CPS3; ++i) {
    for (int j = 0; j < DIMENSION_CPS3; ++j) {
      result.data[i][j] = 0.0;
      for (int k = 0; k < DIMENSION3; ++k) {
        result.data[i][j] += matrix_BTD->data[i][k] * matrix_B->data[k][j];
      }
    }
  }

  return result;
}

Matrix6D CPS3_matrix_6D_multiplication(double num, const Matrix6D* mat) {
  Matrix6D result = create_empty_matrix_6D();

  for (int i = 0; i < DIMENSION_CPS3; ++i) {
    for (int j = 0; j < DIMENSION_CPS3; ++j) {
      result.data[i][j] = num * mat->data[i][j];
    }
  }

  return result;
}

Vector6D CPS3_matrix_B63_mul_vector_3D(const MatrixB63* mat,
                                       const Vector3D* vec) {
  Vector6D result = create_empty_vector_6D();

  for (int i = 0; i < DIMENSION_CPS3; i++) {
    for (int j = 0; j < DIMENSION3; j++) {
      result.data[i] += mat->data[i][j] * vec->data[j];
    }
  }
  return result;
}

Matrix6D CPS3_compute_initial_element_stiffness_matrix(
    const CPS3NodalInfo* X1Y1X2Y2X3Y3, const Matrix3D* property,
    double initial_volume) {
  MatrixB36 B = CPS3_compute_matrix_B(X1Y1X2Y2X3Y3);
  MatrixB63 B_T = create_matrix_B63_from_B36(&B);
  MatrixB63 BT_times_D = CPS3_matrixB63_mul_matrix_3D(&B_T, property);
  Matrix6D BT_times_D_times_B = CPS3_matrixB63_mul_matrix_B36(&BT_times_D, &B);
  Matrix6D K =
      CPS3_matrix_6D_multiplication(initial_volume, &BT_times_D_times_B);
  return K;
}

CPS3NodalInfo CPS3_compute_inner_force_use_E_and_T(
    const CPS3NodalInfo* X1Y1X2Y2X3Y3, const CPS3NodalInfo* u1v1u2v2u3v3,
    const Matrix3D* property, double initial_thickness) {
  // Assume E * property = T
  // other assumptions introduce different results
  CPS3NodalInfo x1y1x2y2x3y3_val =
      CPS3_nodal_info_add(X1Y1X2Y2X3Y3, u1v1u2v2u3v3);
  CPS3NodalInfo* x1y1x2y2x3y3 = &x1y1x2y2x3y3_val;  // make type same as X and u

  double current_area = compute_CPS3_element_square(x1y1x2y2x3y3);
  MatrixB36 B = CPS3_compute_matrix_B(x1y1x2y2x3y3);
  Matrix2D F = CPS3_nodal_disp_to_2D_F(X1Y1X2Y2X3Y3, u1v1u2v2u3v3);
  Matrix2D E_tensor = CPS3_2D_F_to_2D_E(&F);
  Matrix2D E_engineering = CPS3_tensor_strain_to_engineering_strain(&E_tensor);

  // as E and T use same voigt function, thus shear E x 2 first
  Matrix2D T = CPS3_2D_strain_to_2D_stress(&E_engineering, property);
  Matrix2D sigma = CPS3_T_and_F_to_Cauchy(&T, &F);
  Vector3D sigma_voigt = voigt_2D_matrix_to_3D_vector(&sigma);

  // use sigma to compute inner force not T
  MatrixB63 B_T = create_matrix_B63_from_B36(&B);
  Vector6D BT_times_sigma = CPS3_matrix_B63_mul_vector_3D(&B_T, &sigma_voigt);
  Vector6D f_inner_use_sigma = vector_6D_number_multiplication(
      current_area * initial_thickness, &BT_times_sigma);

  return vector_6D_to_CPS3_nodal_info(&f_inner_use_sigma);
}

// C3D4 element info
typedef struct C3D4NodalInfo {
  double node1_dof1;
  double node1_dof2;
  double node1_dof3;
  double node2_dof1;
  double node2_dof2;
  double node2_dof3;
  double node3_dof1;
  double node3_dof2;
  double node3_dof3;
  double node4_dof1;
  double node4_dof2;
  double node4_dof3;
} C3D4NodalInfo;

C3D4NodalInfo create_C3D4_nodal_info(double node1_dof1, double node1_dof2,
                                     double node1_dof3, double node2_dof1,
                                     double node2_dof2, double node2_dof3,
                                     double node3_dof1, double node3_dof2,
                                     double node3_dof3, double node4_dof1,
                                     double node4_dof2, double node4_dof3) {
  C3D4NodalInfo info;
  info.node1_dof1 = node1_dof1;
  info.node1_dof2 = node1_dof2;
  info.node1_dof3 = node1_dof3;
  info.node2_dof1 = node2_dof1;
  info.node2_dof2 = node2_dof2;
  info.node2_dof3 = node2_dof3;
  info.node3_dof1 = node3_dof1;
  info.node3_dof2 = node3_dof2;
  info.node3_dof3 = node3_dof3;
  info.node4_dof1 = node4_dof1;
  info.node4_dof2 = node4_dof2;
  info.node4_dof3 = node4_dof3;
  return info;
}

void print_C3D4_nodal_info(const C3D4NodalInfo* info) {
  printf(
      "C3D4 nodal info:\nnode1: [dof1 = [%f], dof2 = [%f], dof3 = "
      "[%f]]\nnode2: [dof1 = [%f], dof2 = [%f], dof3 = [%f]]\nnode3: [dof1 = "
      "[%f], dof2 = [%f], dof3 = [%f]]\nnode4: [dof1 = [%f], dof2 = [%f], dof3 "
      "= [%f]]\n",
      info->node1_dof1, info->node1_dof2, info->node1_dof3, info->node2_dof1,
      info->node2_dof2, info->node2_dof3, info->node3_dof1, info->node3_dof2,
      info->node3_dof3, info->node4_dof1, info->node4_dof2, info->node4_dof3);
}

C3D4NodalInfo C3D4_nodal_info_add(const C3D4NodalInfo* info1,
                                  const C3D4NodalInfo* info2) {
  return create_C3D4_nodal_info(info1->node1_dof1 + info2->node1_dof1,
                                info1->node1_dof2 + info2->node1_dof2,
                                info1->node1_dof3 + info2->node1_dof3,
                                info1->node2_dof1 + info2->node2_dof1,
                                info1->node2_dof2 + info2->node2_dof2,
                                info1->node2_dof3 + info2->node2_dof3,
                                info1->node3_dof1 + info2->node3_dof1,
                                info1->node3_dof2 + info2->node3_dof2,
                                info1->node3_dof3 + info2->node3_dof3,
                                info1->node4_dof1 + info2->node4_dof1,
                                info1->node4_dof2 + info2->node4_dof2,
                                info1->node4_dof3 + info2->node4_dof3);
}

Vector12D C3D4_nodal_info_to_vector_12D(const C3D4NodalInfo* info) {
  return create_vector_12D(
      info->node1_dof1, info->node1_dof2, info->node1_dof3, info->node2_dof1,
      info->node2_dof2, info->node2_dof3, info->node3_dof1, info->node3_dof2,
      info->node3_dof3, info->node4_dof1, info->node4_dof2, info->node4_dof3);
}

C3D4NodalInfo vector_12D_to_C3D4_nodal_info(const Vector12D* vec) {
  return create_C3D4_nodal_info(
      vector_12D_get_element(vec, 0), vector_12D_get_element(vec, 1),
      vector_12D_get_element(vec, 2), vector_12D_get_element(vec, 3),
      vector_12D_get_element(vec, 4), vector_12D_get_element(vec, 5),
      vector_12D_get_element(vec, 6), vector_12D_get_element(vec, 7),
      vector_12D_get_element(vec, 8), vector_12D_get_element(vec, 9),
      vector_12D_get_element(vec, 10), vector_12D_get_element(vec, 11));
}

// C3D4 displacement to deformation gradient
Matrix3D C3D4_nodal_disp_to_3D_F(const C3D4NodalInfo* X,
                                 const C3D4NodalInfo* u) {
  double X1 = X->node1_dof1;
  double Y1 = X->node1_dof2;
  double Z1 = X->node1_dof3;
  double X2 = X->node2_dof1;
  double Y2 = X->node2_dof2;
  double Z2 = X->node2_dof3;
  double X3 = X->node3_dof1;
  double Y3 = X->node3_dof2;
  double Z3 = X->node3_dof3;
  double X4 = X->node4_dof1;
  double Y4 = X->node4_dof2;
  double Z4 = X->node4_dof3;

  double u1 = u->node1_dof1;
  double v1 = u->node1_dof2;
  double w1 = u->node1_dof3;
  double u2 = u->node2_dof1;
  double v2 = u->node2_dof2;
  double w2 = u->node2_dof3;
  double u3 = u->node3_dof1;
  double v3 = u->node3_dof2;
  double w3 = u->node3_dof3;
  double u4 = u->node4_dof1;
  double v4 = u->node4_dof2;
  double w4 = u->node4_dof3;

  Matrix3D pupX_part_1 =
      create_matrix_3D(u1 - u4, u2 - u4, u3 - u4, v1 - v4, v2 - v4, v3 - v4,
                       w1 - w4, w2 - w4, w3 - w4);
  Matrix3D pupX_part_2 =
      create_matrix_3D(X1 - X4, X2 - X4, X3 - X4, Y1 - Y4, Y2 - Y4, Y3 - Y4,
                       Z1 - Z4, Z2 - Z4, Z3 - Z4);
  Matrix3D pupX_part_2_inv = matrix_3D_inverse(&pupX_part_2);
  Matrix3D pupX = matrix_3D_mul_matrix_3D(&pupX_part_1, &pupX_part_2_inv);
  Matrix3D I = create_eye_matrix_3D();
  Matrix3D F = matrix_3D_add(&I, &pupX);

  return F;
}

double compute_C3D4_element_volume(const C3D4NodalInfo* coords) {
  double x1 = coords->node1_dof1;
  double y1 = coords->node1_dof2;
  double z1 = coords->node1_dof3;
  double x2 = coords->node2_dof1;
  double y2 = coords->node2_dof2;
  double z2 = coords->node2_dof3;
  double x3 = coords->node3_dof1;
  double y3 = coords->node3_dof2;
  double z3 = coords->node3_dof3;
  double x4 = coords->node4_dof1;
  double y4 = coords->node4_dof2;
  double z4 = coords->node4_dof3;

  double a1 = x2 - x1;
  double b1 = y2 - y1;
  double c1 = z2 - z1;

  double a2 = x3 - x1;
  double b2 = y3 - y1;
  double c2 = z3 - z1;

  double a3 = x4 - x1;
  double b3 = y4 - y1;
  double c3 = z4 - z1;

  double det = a1 * (b2 * c3 - c2 * b3) - b1 * (a2 * c3 - c2 * a3) +
               c1 * (a2 * b3 - b2 * a3);
  if (det < 0) {
    det = (-det);
  }
  double volume = det / 6.0;
  return volume;
}

#pragma clang diagnostic pop
