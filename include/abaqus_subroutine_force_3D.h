// abaqus_subroutine_forge_3D.h -- header for ABAQUS user-subroutine development
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

#include "abaqus_subroutine_forge.h"

// Basic type definition for 3D (Matrix and Vector)
// Vector 12D -- use for basic C3D4 vector
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
