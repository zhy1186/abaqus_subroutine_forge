//
// Created by Hengyi Zhao on 2024/11/29.
//

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "abaqus_subroutine_forge.h"
#include "eigen_adaptor.h"

#define MECHANICS_TOLERANCE 1e-10

TEST(MechanicsTest, VoigtMatrix2DToVector3D) {
  Matrix2D mat = create_matrix_2D(1.0, 2.0, 2.0, 4.0);
  Vector3D vec = voigt_2D_matrix_to_3D_vector(&mat);
  Eigen::Vector3d vec_eigen = Eigen::Vector3d(1.0, 4.0, 2.0);
  EXPECT_EQ(vec_eigen, to_eigen(vec));

  mat = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
  EXPECT_DEATH(voigt_2D_matrix_to_3D_vector(&mat),
               "FATAL: only symmetric matrix can use voigt transformation.");
}

TEST(MechanicsTest, VoigtVector3DToVector2D) {
  Vector3D vec = create_vector_3D(1.0, 2.0, 3.0);
  Matrix2D mat = voigt_3D_vector_to_2D_matrix(&vec);
  Eigen::Matrix2d mat_eigen = to_eigen(create_matrix_2D(1.0, 3.0, 3.0, 2.0));

  EXPECT_EQ(mat_eigen, to_eigen(mat));
}

TEST(MechanicsTest, CPS3NodalInfo) {
  CPS3NodalInfo info = create_CPS3_nodal_info(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  EXPECT_EQ(info.node1_dof1, 1.0);
  EXPECT_EQ(info.node1_dof2, 2.0);
  EXPECT_EQ(info.node2_dof1, 3.0);
  EXPECT_EQ(info.node2_dof2, 4.0);
  EXPECT_EQ(info.node3_dof1, 5.0);
  EXPECT_EQ(info.node3_dof2, 6.0);
}

TEST(MechanicsTest, CPS3NodalInfoAddCPS3NodalInfo) {
  CPS3NodalInfo info1 = create_CPS3_nodal_info(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  CPS3NodalInfo info2 = create_CPS3_nodal_info(7.0, 8.0, 9.0, 10.0, 11.0, 12.0);
  CPS3NodalInfo res = CPS3_nodal_info_add(&info1, &info2);
  Vector6D res_vec = CPS3_nodal_info_to_vector_6D(&res);
  Eigen::Vector<double, 6> expect;
  expect << 8, 10, 12, 14, 16, 18;
  EXPECT_TRUE(to_eigen(res_vec).isApprox(expect, MECHANICS_TOLERANCE));
}

TEST(MechanicsTest, ComputeCPS3ElementSquare) {
  CPS3NodalInfo info = create_CPS3_nodal_info(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  EXPECT_DEATH(compute_CPS3_element_square(&info),
               "FATAL: three nodes of element in one line, square = 0!");

  info = create_CPS3_nodal_info(0, 0, 1, -1, 0, -2);
  EXPECT_DEATH(compute_CPS3_element_square(&info),
               "FATAL: node not in counter-clockwise arrangement while "
               "computing square.");

  info = create_CPS3_nodal_info(1.23, 1.23, 2.08, 0.01, 3.45, 3.45);
  double res = 2.2977;
  EXPECT_NEAR(res, compute_CPS3_element_square(&info), MECHANICS_TOLERANCE);
}

bool is_matrix_2d_close(const Matrix2D* mat1, const Matrix2D* mat2) {
  bool res = (to_eigen(*mat1) - to_eigen(*mat2)).cwiseAbs().maxCoeff() <=
             MECHANICS_TOLERANCE;

  if (!res) {
    printf("mat 1 = \n");
    matrix_2D_print(mat1);
    printf("mat 2 = \n");
    matrix_2D_print(mat2);
    printf("difference = \n");
    Matrix2D diff = matrix_2D_minus(mat1, mat2);
    matrix_2D_print(&diff);
  }
  return res;
}

// CPS3_nodal_disp_to_2D_F
TEST(MechanicsTest, CPS3NodalDispToF) {
  CPS3NodalInfo X = create_CPS3_nodal_info(0, 0, 1, 0, 0, 1);
  CPS3NodalInfo u = create_CPS3_nodal_info(0, 0, -0.2, 0, 0, -0.1);
  Matrix2D F = CPS3_nodal_disp_to_2D_F(&X, &u);
  Matrix2D res = create_matrix_2D(0.8, 0, 0, 0.9);
  EXPECT_EQ(to_eigen(F), to_eigen(res));

  X = create_CPS3_nodal_info(-36.25, -17.5, 21.25, 30, 53.75, -31.25);
  u = create_CPS3_nodal_info(0, 0, -10, 20, -5, 0);
  F = CPS3_nodal_disp_to_2D_F(&X, &u);
  res = create_matrix_2D(0.925971622455274, -0.120913016656385,
                         0.054287476866132, 1.35533621221468);
  EXPECT_TRUE(is_matrix_2d_close(&F, &res));
}

// CPS3_2D_F_to_2D_E
TEST(MechanicsTest, CPS3FToE) {
  Matrix2D F = create_matrix_2D(0.925971622455274, -0.120913016656385,
                                0.054287476866132, 1.35533621221468);
  Matrix2D E = CPS3_2D_F_to_2D_E(&F);
  Matrix2D res = create_matrix_2D(-0.0698147121315278, -0.0191921194714194,
                                  -0.0191921194714194, 0.425778102868695);
  EXPECT_TRUE(is_matrix_2d_close(&E, &res));
}

// CPS3_tensor_strain_to_engineering_strain
TEST(MechanicsTest, CPS3TensorStrainToEngineeringStrain) {
  Matrix2D tensor_strain = create_matrix_2D(1, 2, 3, 4);
  Matrix2D engineering_strain =
      CPS3_tensor_strain_to_engineering_strain(&tensor_strain);
  Eigen::Matrix2d expect;
  expect << 1, 4, 6, 4;

  EXPECT_TRUE(expect == to_eigen(engineering_strain));
}

// CPS3_2D_E_to_2D_T
TEST(MechanicsTest, CPS3EToT) {
  Matrix2D E = create_matrix_2D(-0.0698147121315278, -0.0191921194714194,
                                -0.0191921194714194, 0.425778102868695);
  Matrix3D property = create_matrix_3D(1, 2, 3, 4, 5, 6, 7, 8, 9);
  Matrix2D T = CPS3_2D_E_to_2D_T(&E, &property);
  Vector3D T_vector = voigt_2D_matrix_to_3D_vector(&T);
  Eigen::Vector3d res =
      to_eigen(property) * to_eigen(voigt_2D_matrix_to_3D_vector(&E));

  EXPECT_TRUE(to_eigen(T_vector).isApprox(res, MECHANICS_TOLERANCE));
}

// CPS3_T_and_F_to_Cauchy
TEST(MechanicsTest, CPS3TAndFToCauchy) {
  Matrix2D F = create_matrix_2D(1, 2, 3, 4);
  Matrix2D T = create_matrix_2D(5, 6, 7, 8);
  Matrix2D sigma = CPS3_T_and_F_to_Cauchy(&T, &F);

  EXPECT_DOUBLE_EQ(matrix_2D_get_element(&sigma, 0, 0), -31.5);
  EXPECT_DOUBLE_EQ(matrix_2D_get_element(&sigma, 0, 1), -72.5);
  EXPECT_DOUBLE_EQ(matrix_2D_get_element(&sigma, 1, 0), -71.5);
  EXPECT_DOUBLE_EQ(matrix_2D_get_element(&sigma, 1, 1), -164.5);
}

// CPS3_E_and_T_to_strain_energy_density
TEST(MechanicsTest, CPS3EAndTToStaainEnergyDensity) {
  Matrix2D E = create_matrix_2D(1, 2, 3, 4);
  Matrix2D T = create_matrix_2D(5, 6, 7, 8);
  double strain_energy_density = CPS3_E_and_T_to_strain_energy_density(&E, &T);

  EXPECT_DOUBLE_EQ(strain_energy_density, 35);
}

// CPS3_compute_matrix_B
TEST(MechanicsTest, CPS3ComputeMatrixB) {
  CPS3NodalInfo coords = create_CPS3_nodal_info(1, 1, 3, 1, 1, 4);
  MatrixB36 matrix_B = CPS3_compute_matrix_B(&coords);
  Eigen::Matrix<double, 3, 6> res;
  res << -0.5, 0, 0.5, 0, 0, 0, 0, -0.33333333333, 0, 0, 0, 0.33333333333,
      -0.33333333333, -0.5, 0, 0.5, 0.33333333333, 0;
  EXPECT_TRUE(to_eigen(matrix_B).isApprox(res, MECHANICS_TOLERANCE));

  coords = create_CPS3_nodal_info(1.23, 5.678, 4.321, -3.21, 9.876, 2.345);
  matrix_B = CPS3_compute_matrix_B(&coords);

  res.setZero();
  res << -0.083417073223, 0.0, -0.050049929103, 0.0, 0.133599886324, 0.0, 0.0,
      0.083417073223, 0.0, -0.129878977941, 0.0, 0.046412181327, 0.083417073223,
      -0.083417073223, -0.129878977941, -0.050049929103, 0.046412181327,
      0.133599886324;
  EXPECT_TRUE(to_eigen(matrix_B).isApprox(res, 1e-3));
}

TEST(MechanicsTest, CPS3MatrixMultiplicationIdentityMatrix) {
  MatrixB36 matrix_B =
      create_matrix_B36(1.0, 4.0, 7.0, 10.0, 13.0, 16.0, 2.0, 5.0, 8.0, 11.0,
                        14.0, 17.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0);
  MatrixB63 matrix_BT = create_matrix_B63_from_B36(&matrix_B);
  Matrix3D matrix_D = create_eye_matrix_3D();
  MatrixB63 result = CPS3_matrixB63_mul_matrix_3D(&matrix_BT, &matrix_D);

  EXPECT_TRUE(to_eigen(matrix_BT) == to_eigen(result));
}

TEST(MechanicsTest, CPS3MatrixMultiplicationGeneralMatrix) {
  MatrixB36 matrix_B =
      create_matrix_B36(1.0, 0.0, 4.0, 2.0, 3.0, 1.0, 0.0, 3.0, 1.0, 2.0, 1.0,
                        4.0, 2.0, 1.0, 0.0, 2.0, 1.0, 0.0);
  MatrixB63 matrix_BT = create_matrix_B63_from_B36(&matrix_B);
  Matrix3D matrix_D =
      create_matrix_3D(2.0, 0.0, 1.0, 1.0, 3.0, 0.0, 0.0, 2.0, 4.0);
  MatrixB63 expected = {
      MatrixB6X3,
      {{1.0 * 2.0 + 0.0 * 1.0 + 2.0 * 0.0, 1.0 * 0.0 + 0.0 * 3.0 + 2.0 * 2.0,
        1.0 * 1.0 + 0.0 * 0.0 + 2.0 * 4.0},
       {0.0 * 2.0 + 3.0 * 1.0 + 1.0 * 0.0, 0.0 * 0.0 + 3.0 * 3.0 + 1.0 * 2.0,
        0.0 * 1.0 + 3.0 * 0.0 + 1.0 * 4.0},
       {4.0 * 2.0 + 1.0 * 1.0 + 0.0 * 0.0, 4.0 * 0.0 + 1.0 * 3.0 + 0.0 * 2.0,
        4.0 * 1.0 + 1.0 * 0.0 + 0.0 * 4.0},
       {2.0 * 2.0 + 2.0 * 1.0 + 2.0 * 0.0, 2.0 * 0.0 + 2.0 * 3.0 + 2.0 * 2.0,
        2.0 * 1.0 + 2.0 * 0.0 + 2.0 * 4.0},
       {3.0 * 2.0 + 1.0 * 1.0 + 1.0 * 0.0, 3.0 * 0.0 + 1.0 * 3.0 + 1.0 * 2.0,
        3.0 * 1.0 + 1.0 * 0.0 + 1.0 * 4.0},
       {1.0 * 2.0 + 4.0 * 1.0 + 0.0 * 0.0, 1.0 * 0.0 + 4.0 * 3.0 + 0.0 * 2.0,
        1.0 * 1.0 + 4.0 * 0.0 + 0.0 * 4.0}}};

  MatrixB63 result = CPS3_matrixB63_mul_matrix_3D(&matrix_BT, &matrix_D);
  EXPECT_TRUE(
      to_eigen(expected).isApprox(to_eigen(result), MECHANICS_TOLERANCE));

  matrix_BT = {
      MatrixB6X3,
      {{2, -1, 3}, {0, 4, -2}, {5, 2, 1}, {-3, 3, 0}, {1, 0, 4}, {6, -2, -1}}};
  matrix_D = create_matrix_3D(1, 2, -1, 0, -3, 4, 5, 1, 2);
  expected = {MatrixB6X3,
              {{17, 10, 0},
               {-10, -14, 12},
               {10, 5, 5},
               {-3, -15, 15},
               {21, 6, 7},
               {1, 17, -16}}};
  result = CPS3_matrixB63_mul_matrix_3D(&matrix_BT, &matrix_D);
  EXPECT_TRUE(
      to_eigen(expected).isApprox(to_eigen(result), MECHANICS_TOLERANCE));
}

TEST(MechanicsTest, CPS3MatrixB63MulMatrixB36) {
  MatrixB36 mat_b36 =
      create_matrix_B36(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
                        12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0);

  MatrixB63 mat_b63 = create_matrix_B63_from_B36(&mat_b36);

  Matrix6D result = CPS3_matrixB63_mul_matrix_B36(&mat_b63, &mat_b36);

  Eigen::Matrix<double, 6, 6> expected;
  expected << 219.0, 240.0, 261.0, 282.0, 303.0, 324.0, 240.0, 264.0, 288.0,
      312.0, 336.0, 360.0, 261.0, 288.0, 315.0, 342.0, 369.0, 396.0, 282.0,
      312.0, 342.0, 372.0, 402.0, 432.0, 303.0, 336.0, 369.0, 402.0, 435.0,
      468.0, 324.0, 360.0, 396.0, 432.0, 468.0, 504.0;

  EXPECT_TRUE(to_eigen(result).isApprox(expected, MECHANICS_TOLERANCE));
}

TEST(MechanicsTest, CPS3NumMulMatrix6D) {
  Matrix6D m = create_matrix_6D(
      1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0,
      15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0,
      27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0);

  double factor = 2.5;
  Matrix6D result = CPS3_matrix_6D_multiplication(factor, &m);
  Matrix6D expected = create_matrix_6D(
      2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0, 27.5, 30.0, 32.5,
      35.0, 37.5, 40.0, 42.5, 45.0, 47.5, 50.0, 52.5, 55.0, 57.5, 60.0, 62.5,
      65.0, 67.5, 70.0, 72.5, 75.0, 77.5, 80.0, 82.5, 85.0, 87.5, 90.0);

  EXPECT_TRUE(
      to_eigen(result).isApprox(to_eigen(expected), MECHANICS_TOLERANCE));
}

TEST(MechanicsTest, CPS3MatrixB63MulVector3D) {
  MatrixB63 mat_b63 = {.type = MatrixB6X3,
                       .data = {{1.0, 2.0, 3.0},
                                {4.0, 5.0, 6.0},
                                {7.0, 8.0, 9.0},
                                {10.0, 11.0, 12.0},
                                {13.0, 14.0, 15.0},
                                {16.0, 17.0, 18.0}}};

  Vector3D vec_3d = create_vector_3D(2.0, -1.0, 3.0);

  Vector6D result_6d = CPS3_matrix_B63_mul_vector_3D(&mat_b63, &vec_3d);

  Eigen::Matrix<double, 6, 3> mat_eigen = to_eigen(mat_b63);
  Eigen::Vector3d vec_eigen = to_eigen(vec_3d);
  Eigen::Matrix<double, 6, 1> actual = to_eigen(result_6d);
  Eigen::Matrix<double, 6, 1> expect = mat_eigen * vec_eigen;

  EXPECT_TRUE(actual.isApprox(expect, MECHANICS_TOLERANCE));
}

TEST(MechanicsTest, CPS3InitialStiffnessMatrix) {
  CPS3NodalInfo X1Y1X2Y2X3Y3 = {7.8, 9.1, 5.67, 12.34, 1.23, 4.56};
  Matrix3D property =
      create_matrix_3D(196.489, 2.9621, 0, 2.9621, 9.8738, 0, 0, 0, 2.8451);
  double thickness = 1.05;
  double initial_area = compute_CPS3_element_square(&X1Y1X2Y2X3Y3);
  double initial_volume = initial_area * thickness;
  Matrix6D K = CPS3_compute_initial_element_stiffness_matrix(
      &X1Y1X2Y2X3Y3, &property, initial_volume);
  Eigen::Matrix<double, 6, 6> expect;
  expect << 202.64745531718, -3.4019645733114, -119.10686057069,
      3.5403106186161, -83.540594746487, -.13834604530478, -3.4019645733114,
      6.2215390581452, 3.4788856186161, -6.5888965454986, -.76921045304779E-01,
      0.36735748735342, -119.10686057069, 3.4788856186161, 70.765887939069,
      -2.9375714728171, 48.340972631626, -.54131414579901, 3.5403106186161,
      -6.5888965454986, -2.9375714728171, 8.2224672193526, -.60273914579901,
      -1.6335706738541, -83.540594746487, -.76921045304779E-01, 48.340972631626,
      -.60273914579901, 35.199622114861, 0.67966019110379, -.13834604530478,
      0.36735748735342, -.54131414579901, -1.6335706738541, 0.67966019110379,
      1.2662131865006;
  EXPECT_TRUE(to_eigen(K).isApprox(expect, MECHANICS_TOLERANCE));
}

TEST(MechanicsTest, CPS3ComputeInnerForce) {
  CPS3NodalInfo X1Y1X2Y2X3Y3 = {7.8, 9.1, 5.67, 12.34, 1.23, 4.56};
  CPS3NodalInfo u1v1u2v2u3v3 = {-4.16364e-2, -2.35390e-2, 0, 0, -9.86607e-2, 0};
  CPS3NodalInfo x1y1x2y2x3y3 =
      CPS3_nodal_info_add(&X1Y1X2Y2X3Y3, &u1v1u2v2u3v3);
  Matrix3D property =
      create_matrix_3D(196.489, 2.9621, 0, 2.9621, 9.8738, 0, 0, 0, 2.8451);
  double thickness = 1.05;
  double initial_area = compute_CPS3_element_square(&X1Y1X2Y2X3Y3);
  double initial_volume = initial_area * thickness;
  double current_area = compute_CPS3_element_square(&x1y1x2y2x3y3);
  double current_thickness = initial_volume / current_area;
  CPS3NodalInfo inner_force = CPS3_compute_inner_force(
      &X1Y1X2Y2X3Y3, &u1v1u2v2u3v3, &property, current_thickness);

  double error_tolerance = 0.01;
  EXPECT_NEAR(inner_force.node1_dof1, -0.1, 0.1 * error_tolerance);
  EXPECT_NEAR(inner_force.node1_dof2, 0.001, 0.001 * error_tolerance);
  EXPECT_NEAR(inner_force.node3_dof1, 0.0002, 0.0002 * error_tolerance * 10);
}
