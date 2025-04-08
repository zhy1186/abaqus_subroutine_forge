//
// Created by Hengyi Zhao on 2025/3/10.
//

#include "communication.h"

#include <gtest/gtest.h>

TEST(CPS3CommInfoTest, EmptyVolumeEnergyInfo) {
  CPS3VolumeEnergyInfo info = create_empty_CPS3_volume_energy_info();
  EXPECT_DOUBLE_EQ(info.area, 0);
  EXPECT_DOUBLE_EQ(info.thickness, 0);
  EXPECT_DOUBLE_EQ(info.volume, 0);
  EXPECT_DOUBLE_EQ(info.strain_energy_density, 0);
  EXPECT_DOUBLE_EQ(info.strain_energy, 0);
}

TEST(CPS3CommInfoTest, VolumeEnergyInfoCalculation) {
  double area = 10.0, thickness = 2.0, sed = 5.0;
  CPS3VolumeEnergyInfo info =
      create_CPS3_volume_energy_info(area, thickness, sed);
  EXPECT_DOUBLE_EQ(info.area, area);
  EXPECT_DOUBLE_EQ(info.thickness, thickness);
  EXPECT_DOUBLE_EQ(info.volume, area * thickness);
  EXPECT_DOUBLE_EQ(info.strain_energy_density, sed);
  EXPECT_DOUBLE_EQ(info.strain_energy, sed * area * thickness);
}

TEST(CPS3CommInfoTest, EmptyCommonInfo) {
  CPS3CommInfo info = create_empty_CPS3_common_info();
  EXPECT_EQ(info.status, DefaultInitialized);
  // deformation gradient
  EXPECT_DOUBLE_EQ(info.F11, 0);
  EXPECT_DOUBLE_EQ(info.F12, 0);
  EXPECT_DOUBLE_EQ(info.F21, 0);
  EXPECT_DOUBLE_EQ(info.F22, 0);
  // green strain
  EXPECT_DOUBLE_EQ(info.E11, 0);
  EXPECT_DOUBLE_EQ(info.E12, 0);
  EXPECT_DOUBLE_EQ(info.E21, 0);
  EXPECT_DOUBLE_EQ(info.E22, 0);
  // PK2 stress
  EXPECT_DOUBLE_EQ(info.T11, 0);
  EXPECT_DOUBLE_EQ(info.T12, 0);
  EXPECT_DOUBLE_EQ(info.T21, 0);
  EXPECT_DOUBLE_EQ(info.T22, 0);
  // Cauchy stress
  EXPECT_DOUBLE_EQ(info.sigma11, 0);
  EXPECT_DOUBLE_EQ(info.sigma12, 0);
  EXPECT_DOUBLE_EQ(info.sigma21, 0);
  EXPECT_DOUBLE_EQ(info.sigma22, 0);
  // volume and energy info
  EXPECT_DOUBLE_EQ(info.volume_energy_info.area, 0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.thickness, 0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.volume, 0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.strain_energy_density, 0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.strain_energy, 0);
}

TEST(CPS3CommInfoTest, CreateCommonInfoFromMatrix2Ds) {
  Matrix2D F = create_matrix_2D(1, 2, 3, 4);
  Matrix2D E = create_matrix_2D(0.1, 0.2, 0.3, 0.4);
  Matrix2D T = create_matrix_2D(10, 20, 30, 40);
  Matrix2D sigma = create_matrix_2D(100, 200, 300, 400);
  CPS3VolumeEnergyInfo vol_info = create_CPS3_volume_energy_info(5.0, 2.0, 3.0);
  CPS3CommInfo info = create_CPS3_common_info_from_matrix_2Ds(
      ContainInfo, &F, &E, &T, &sigma, vol_info);

  EXPECT_EQ(info.status, ContainInfo);
  // deformation gradient
  EXPECT_DOUBLE_EQ(info.F11, 1.0);
  EXPECT_DOUBLE_EQ(info.F12, 2.0);
  EXPECT_DOUBLE_EQ(info.F21, 3.0);
  EXPECT_DOUBLE_EQ(info.F22, 4.0);
  // green strain
  EXPECT_DOUBLE_EQ(info.E11, 0.1);
  EXPECT_DOUBLE_EQ(info.E12, 0.2);
  EXPECT_DOUBLE_EQ(info.E21, 0.3);
  EXPECT_DOUBLE_EQ(info.E22, 0.4);
  // PK2 stress
  EXPECT_DOUBLE_EQ(info.T11, 10.0);
  EXPECT_DOUBLE_EQ(info.T12, 20.0);
  EXPECT_DOUBLE_EQ(info.T21, 30.0);
  EXPECT_DOUBLE_EQ(info.T22, 40.0);
  // Cauchy stress
  EXPECT_DOUBLE_EQ(info.sigma11, 100.0);
  EXPECT_DOUBLE_EQ(info.sigma12, 200.0);
  EXPECT_DOUBLE_EQ(info.sigma21, 300.0);
  EXPECT_DOUBLE_EQ(info.sigma22, 400.0);
  // volume and energy info
  EXPECT_DOUBLE_EQ(info.volume_energy_info.area, 5.0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.thickness, 2.0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.volume, 10.0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.strain_energy_density, 3.0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.strain_energy, 30.0);
}

TEST(CPS3CommInfoTest, ClearCommonInfo) {
  Matrix2D F = create_matrix_2D(1, 2, 3, 4);
  Matrix2D E = create_matrix_2D(0.1, 0.2, 0.3, 0.4);
  Matrix2D T = create_matrix_2D(10, 20, 30, 40);
  Matrix2D sigma = create_matrix_2D(100, 200, 300, 400);
  CPS3VolumeEnergyInfo vol_info = create_CPS3_volume_energy_info(5.0, 2.0, 3.0);
  CPS3CommInfo info = create_CPS3_common_info_from_matrix_2Ds(
      ContainInfo, &F, &E, &T, &sigma, vol_info);

  clear_CPS3_common_info(&info);

  EXPECT_EQ(info.status, InfoCleared);
  // deformation gradient
  EXPECT_DOUBLE_EQ(info.F11, 0);
  EXPECT_DOUBLE_EQ(info.F12, 0);
  EXPECT_DOUBLE_EQ(info.F21, 0);
  EXPECT_DOUBLE_EQ(info.F22, 0);
  // green strain
  EXPECT_DOUBLE_EQ(info.E11, 0);
  EXPECT_DOUBLE_EQ(info.E12, 0);
  EXPECT_DOUBLE_EQ(info.E21, 0);
  EXPECT_DOUBLE_EQ(info.E22, 0);
  // PK2 stress
  EXPECT_DOUBLE_EQ(info.T11, 0);
  EXPECT_DOUBLE_EQ(info.T12, 0);
  EXPECT_DOUBLE_EQ(info.T21, 0);
  EXPECT_DOUBLE_EQ(info.T22, 0);
  // Cauchy stress
  EXPECT_DOUBLE_EQ(info.sigma11, 0);
  EXPECT_DOUBLE_EQ(info.sigma12, 0);
  EXPECT_DOUBLE_EQ(info.sigma21, 0);
  EXPECT_DOUBLE_EQ(info.sigma22, 0);
  // volume and energy info
  EXPECT_DOUBLE_EQ(info.volume_energy_info.area, 0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.thickness, 0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.volume, 0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.strain_energy_density, 0);
  EXPECT_DOUBLE_EQ(info.volume_energy_info.strain_energy, 0);
}

TEST(CPS3CommInfoTest, CPS3CommonInfoToDoubleArrayUseMalloc) {
  Matrix2D F = create_matrix_2D(1, 2, 3, 4);
  Matrix2D E = create_matrix_2D(0.1, 0.2, 0.3, 0.4);
  Matrix2D T = create_matrix_2D(10, 20, 30, 40);
  Matrix2D sigma = create_matrix_2D(100, 200, 300, 400);
  CPS3VolumeEnergyInfo vol_info = create_CPS3_volume_energy_info(5.0, 2.0, 3.0);
  CPS3CommInfo info = create_CPS3_common_info_from_matrix_2Ds(
      ContainInfo, &F, &E, &T, &sigma, vol_info);
  double *double_array_info =
      CPS3_common_info_to_double_array_use_malloc(&info);

  EXPECT_DOUBLE_EQ(double_array_info[0], info.F11);
  EXPECT_DOUBLE_EQ(double_array_info[1], info.F12);
  EXPECT_DOUBLE_EQ(double_array_info[2], info.F21);
  EXPECT_DOUBLE_EQ(double_array_info[3], info.F22);
  EXPECT_DOUBLE_EQ(double_array_info[4], info.E11);
  EXPECT_DOUBLE_EQ(double_array_info[5], info.E12);
  EXPECT_DOUBLE_EQ(double_array_info[6], info.E21);
  EXPECT_DOUBLE_EQ(double_array_info[7], info.E22);
  EXPECT_DOUBLE_EQ(double_array_info[8], info.T11);
  EXPECT_DOUBLE_EQ(double_array_info[9], info.T12);
  EXPECT_DOUBLE_EQ(double_array_info[10], info.T21);
  EXPECT_DOUBLE_EQ(double_array_info[11], info.T22);
  EXPECT_DOUBLE_EQ(double_array_info[12], info.sigma11);
  EXPECT_DOUBLE_EQ(double_array_info[13], info.sigma12);
  EXPECT_DOUBLE_EQ(double_array_info[14], info.sigma21);
  EXPECT_DOUBLE_EQ(double_array_info[15], info.sigma22);
  EXPECT_DOUBLE_EQ(double_array_info[16], info.volume_energy_info.area);
  EXPECT_DOUBLE_EQ(double_array_info[17], info.volume_energy_info.thickness);
  EXPECT_DOUBLE_EQ(double_array_info[18], info.volume_energy_info.volume);
  EXPECT_DOUBLE_EQ(double_array_info[19],
                   info.volume_energy_info.strain_energy_density);
  EXPECT_DOUBLE_EQ(double_array_info[20],
                   info.volume_energy_info.strain_energy);
  EXPECT_DOUBLE_EQ(double_array_info[21], (double)info.status);
  free(double_array_info);
}

TEST(C3D4CommInfoTest, C3D4CommonInfoToDoubleArrayUseMalloc) {
  Matrix3D F = create_matrix_3D(1, 2, 3, 4, 5, 6, 7, 8, 9);
  Matrix3D E = create_matrix_3D(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9);
  Matrix3D T = create_matrix_3D(10, 20, 30, 40, 50, 60, 70, 80, 90);
  Matrix3D sigma =
      create_matrix_3D(100, 200, 300, 400, 500, 600, 700, 800, 900);
  C3D4CommInfo info = create_C3D4_common_info_from_matrix_3Ds(
      ContainInfo, &F, &E, &T, &sigma, 1000, 2000);
  double *double_array_info =
      C3D4_common_info_to_double_array_use_malloc(&info);

  EXPECT_DOUBLE_EQ(double_array_info[0], info.F11);
  EXPECT_DOUBLE_EQ(double_array_info[1], info.F12);
  EXPECT_DOUBLE_EQ(double_array_info[2], info.F13);
  EXPECT_DOUBLE_EQ(double_array_info[3], info.F21);
  EXPECT_DOUBLE_EQ(double_array_info[4], info.F22);
  EXPECT_DOUBLE_EQ(double_array_info[5], info.F23);
  EXPECT_DOUBLE_EQ(double_array_info[6], info.F31);
  EXPECT_DOUBLE_EQ(double_array_info[7], info.F32);
  EXPECT_DOUBLE_EQ(double_array_info[8], info.F33);
  EXPECT_DOUBLE_EQ(double_array_info[9], info.E11);
  EXPECT_DOUBLE_EQ(double_array_info[10], info.E12);
  EXPECT_DOUBLE_EQ(double_array_info[11], info.E13);
  EXPECT_DOUBLE_EQ(double_array_info[12], info.E21);
  EXPECT_DOUBLE_EQ(double_array_info[13], info.E22);
  EXPECT_DOUBLE_EQ(double_array_info[14], info.E23);
  EXPECT_DOUBLE_EQ(double_array_info[15], info.E31);
  EXPECT_DOUBLE_EQ(double_array_info[16], info.E32);
  EXPECT_DOUBLE_EQ(double_array_info[17], info.E33);
  EXPECT_DOUBLE_EQ(double_array_info[18], info.T11);
  EXPECT_DOUBLE_EQ(double_array_info[19], info.T12);
  EXPECT_DOUBLE_EQ(double_array_info[20], info.T13);
  EXPECT_DOUBLE_EQ(double_array_info[21], info.T21);
  EXPECT_DOUBLE_EQ(double_array_info[22], info.T22);
  EXPECT_DOUBLE_EQ(double_array_info[23], info.T23);
  EXPECT_DOUBLE_EQ(double_array_info[24], info.T31);
  EXPECT_DOUBLE_EQ(double_array_info[25], info.T32);
  EXPECT_DOUBLE_EQ(double_array_info[26], info.T33);
  EXPECT_DOUBLE_EQ(double_array_info[27], info.sigma11);
  EXPECT_DOUBLE_EQ(double_array_info[28], info.sigma12);
  EXPECT_DOUBLE_EQ(double_array_info[29], info.sigma13);
  EXPECT_DOUBLE_EQ(double_array_info[30], info.sigma21);
  EXPECT_DOUBLE_EQ(double_array_info[31], info.sigma22);
  EXPECT_DOUBLE_EQ(double_array_info[32], info.sigma23);
  EXPECT_DOUBLE_EQ(double_array_info[33], info.sigma31);
  EXPECT_DOUBLE_EQ(double_array_info[34], info.sigma32);
  EXPECT_DOUBLE_EQ(double_array_info[35], info.sigma33);
  EXPECT_DOUBLE_EQ(double_array_info[36], info.volume);
  EXPECT_DOUBLE_EQ(double_array_info[37], info.strain_energy_density);
  EXPECT_DOUBLE_EQ(double_array_info[38], info.strain_energy);
  EXPECT_DOUBLE_EQ(double_array_info[39], (double)info.status);
  free(double_array_info);
}

TEST(UELInfoTest, UELBasicInfo) {
  int element_dof_num = 1;
  int solution_dependent_vars_num = 2;
  int properties_num = 3;
  int element_node_num = 4;
  int element_type_ID = 5;
  double current_step_time_and_total_time_values[2];
  current_step_time_and_total_time_values[0] = 6.0;
  current_step_time_and_total_time_values[1] = 7.0;
  double time_increment_value = 8.0;
  int current_step_num = 9;
  int current_increment_num = 10;
  int element_no_ID = 11;

  UELBasicInfo uel_basic_info = create_UEL_basic_info(
      &element_dof_num, &solution_dependent_vars_num, &properties_num,
      &element_node_num, &element_type_ID,
      current_step_time_and_total_time_values, &time_increment_value,
      &current_step_num, &current_increment_num, &element_no_ID);

  EXPECT_EQ(uel_basic_info.element_dof_num, element_dof_num);
  EXPECT_EQ(uel_basic_info.solution_dependent_vars_num,
            solution_dependent_vars_num);
  EXPECT_EQ(uel_basic_info.properties_num, properties_num);
  EXPECT_EQ(uel_basic_info.element_node_num, element_node_num);
  EXPECT_EQ(uel_basic_info.element_type_ID, element_type_ID);
  EXPECT_DOUBLE_EQ(uel_basic_info.current_step_time,
                   current_step_time_and_total_time_values[0]);
  EXPECT_DOUBLE_EQ(uel_basic_info.total_time,
                   current_step_time_and_total_time_values[1]);
  EXPECT_DOUBLE_EQ(uel_basic_info.time_increment_value, time_increment_value);
  EXPECT_EQ(uel_basic_info.current_step_num, current_step_num);
  EXPECT_EQ(uel_basic_info.current_increment_num, current_increment_num);
  EXPECT_EQ(uel_basic_info.element_no_ID, element_no_ID);

  print_UEL_basic_info(&uel_basic_info);
}

TEST(UELInfoTest, UELPassedInInfo) {
  const int properties_num = 4;
  double properties_array[4] = {1, 2, 3, 4};
  const int element_dof_num = 6;
  double original_coords_array[6] = {0, 1, 2, 3, 4, 5};
  double displacement_array[6] = {6, 7, 8, 9, 10, 11};
  UELPassedInInfo info = create_UEL_passed_in_info(
      &properties_num, properties_array, &element_dof_num,
      original_coords_array, displacement_array);

  EXPECT_EQ(info.properties_num, properties_num);
  for (int i = 1; i < properties_num; ++i) {
    EXPECT_DOUBLE_EQ(info.properties_array[i], properties_array[i]);
  }

  EXPECT_EQ(info.element_dof_num, element_dof_num);
  for (int i = 1; i < element_dof_num; ++i) {
    EXPECT_DOUBLE_EQ(info.original_coords_array[i], original_coords_array[i]);
    EXPECT_DOUBLE_EQ(info.displacement_array[i], displacement_array[i]);
  }

  print_UEL_passed_in_info(&info);
}

TEST(UELInfoTest, UELCriticalInfo) {
  const int element_no_ID = 3;
  const int element_type_ID = 2;
  const int current_step_num = 1;
  const int current_increment_num = 42;

  UELCriticalInfo info =
      create_UEL_critical_info(&element_no_ID, &element_type_ID,
                               &current_step_num, &current_increment_num);

  EXPECT_EQ(info.element_no_ID, element_no_ID);
  EXPECT_EQ(info.element_type_ID, element_type_ID);
  EXPECT_EQ(info.current_step_num, current_step_num);
  EXPECT_EQ(info.current_increment_num, current_increment_num);

  print_UEL_critical_info(&info);
}

TEST(UMATInfoTest, UMATBasicInfo) {
  const int element_no_ID = 1;
  const int integration_point_ID = 2;
  const int step_number = 3;
  const int increment_number = 42;

  UMATBasicInfo info = create_UMAT_basic_info(
      &element_no_ID, &integration_point_ID, &step_number, &increment_number);
  EXPECT_EQ(info.element_no_ID, element_no_ID);
  EXPECT_EQ(info.integration_point_ID, integration_point_ID);
  EXPECT_EQ(info.step_number, step_number);
  EXPECT_EQ(info.increment_number, increment_number);

  print_UMAT_basic_info(&info);
}

TEST(LogTest, LogInfoDebugAndLogArray) {
  log_info("GTest log info to stderr and flush %d", 1);
  int debug_info1 = 3;
  double debug_info2 = 3.1415;
  const char *debug_info3 = "Alice Bob.";
  log_debug("debug info 1 = %d, debug info 2 = %f, debug info 3 = %s",
            debug_info1, debug_info2, debug_info3);
  int debug_array1[5] = {1, 2, 3, 4};
  double debug_array2[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  log_debug_array("debug_array1", TYPE_INT, 5, debug_array1);
  log_debug_array("debug_array2", TYPE_DOUBLE, 6, debug_array2);

  SUCCEED();
}

TEST(C3D4CommonInfoTest, ComprehensiveTest) {
  // --- Part 1: Test create_C3D4_common_info_from_matrix_3Ds ---
  // Define four 3x3 matrices using create_matrix_3D.
  // For simplicity, we use:
  //  F = identity; E, T, sigma with arbitrary but distinct values.
  Matrix3D F = create_matrix_3D(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
  Matrix3D E =
      create_matrix_3D(0.1, 0.02, 0.03, 0.02, 0.15, 0.04, 0.03, 0.04, 0.2);
  Matrix3D T = create_matrix_3D(10.0, 2.0, 3.0, 2.0, 15.0, 4.0, 3.0, 4.0, 20.0);
  Matrix3D sigma =
      create_matrix_3D(12.0, 3.0, 4.0, 3.0, 18.0, 5.0, 4.0, 5.0, 24.0);
  double volume = 1.0;
  double strain_energy_density = 100.0;  // so energy should be 100.0

  // Create common info with a status of DefaultInitialized.
  UelInfoStatus input_status = DefaultInitialized;
  C3D4CommInfo info = create_C3D4_common_info_from_matrix_3Ds(
      input_status, &F, &E, &T, &sigma, volume, strain_energy_density);
  // Verify that the status was set correctly.
  EXPECT_EQ(info.status, input_status);

  // Check deformation gradient F components.
  EXPECT_DOUBLE_EQ(info.F11, matrix_3D_get_element(&F, 0, 0));
  EXPECT_DOUBLE_EQ(info.F12, matrix_3D_get_element(&F, 0, 1));
  EXPECT_DOUBLE_EQ(info.F13, matrix_3D_get_element(&F, 0, 2));
  EXPECT_DOUBLE_EQ(info.F21, matrix_3D_get_element(&F, 1, 0));
  EXPECT_DOUBLE_EQ(info.F22, matrix_3D_get_element(&F, 1, 1));
  EXPECT_DOUBLE_EQ(info.F23, matrix_3D_get_element(&F, 1, 2));
  EXPECT_DOUBLE_EQ(info.F31, matrix_3D_get_element(&F, 2, 0));
  EXPECT_DOUBLE_EQ(info.F32, matrix_3D_get_element(&F, 2, 1));
  EXPECT_DOUBLE_EQ(info.F33, matrix_3D_get_element(&F, 2, 2));

  // Check Green strain E components.
  EXPECT_DOUBLE_EQ(info.E11, matrix_3D_get_element(&E, 0, 0));
  EXPECT_DOUBLE_EQ(info.E12, matrix_3D_get_element(&E, 0, 1));
  EXPECT_DOUBLE_EQ(info.E13, matrix_3D_get_element(&E, 0, 2));
  EXPECT_DOUBLE_EQ(info.E21, matrix_3D_get_element(&E, 1, 0));
  EXPECT_DOUBLE_EQ(info.E22, matrix_3D_get_element(&E, 1, 1));
  EXPECT_DOUBLE_EQ(info.E23, matrix_3D_get_element(&E, 1, 2));
  EXPECT_DOUBLE_EQ(info.E31, matrix_3D_get_element(&E, 2, 0));
  EXPECT_DOUBLE_EQ(info.E32, matrix_3D_get_element(&E, 2, 1));
  EXPECT_DOUBLE_EQ(info.E33, matrix_3D_get_element(&E, 2, 2));

  // Check PK2 stress T components.
  EXPECT_DOUBLE_EQ(info.T11, matrix_3D_get_element(&T, 0, 0));
  EXPECT_DOUBLE_EQ(info.T12, matrix_3D_get_element(&T, 0, 1));
  EXPECT_DOUBLE_EQ(info.T13, matrix_3D_get_element(&T, 0, 2));
  EXPECT_DOUBLE_EQ(info.T21, matrix_3D_get_element(&T, 1, 0));
  EXPECT_DOUBLE_EQ(info.T22, matrix_3D_get_element(&T, 1, 1));
  EXPECT_DOUBLE_EQ(info.T23, matrix_3D_get_element(&T, 1, 2));
  EXPECT_DOUBLE_EQ(info.T31, matrix_3D_get_element(&T, 2, 0));
  EXPECT_DOUBLE_EQ(info.T32, matrix_3D_get_element(&T, 2, 1));
  EXPECT_DOUBLE_EQ(info.T33, matrix_3D_get_element(&T, 2, 2));

  // Check Cauchy stress sigma components.
  EXPECT_DOUBLE_EQ(info.sigma11, matrix_3D_get_element(&sigma, 0, 0));
  EXPECT_DOUBLE_EQ(info.sigma12, matrix_3D_get_element(&sigma, 0, 1));
  EXPECT_DOUBLE_EQ(info.sigma13, matrix_3D_get_element(&sigma, 0, 2));
  EXPECT_DOUBLE_EQ(info.sigma21, matrix_3D_get_element(&sigma, 1, 0));
  EXPECT_DOUBLE_EQ(info.sigma22, matrix_3D_get_element(&sigma, 1, 1));
  EXPECT_DOUBLE_EQ(info.sigma23, matrix_3D_get_element(&sigma, 1, 2));
  EXPECT_DOUBLE_EQ(info.sigma31, matrix_3D_get_element(&sigma, 2, 0));
  EXPECT_DOUBLE_EQ(info.sigma32, matrix_3D_get_element(&sigma, 2, 1));
  EXPECT_DOUBLE_EQ(info.sigma33, matrix_3D_get_element(&sigma, 2, 2));

  // Check volume and energy fields.
  EXPECT_DOUBLE_EQ(info.volume, volume);
  EXPECT_DOUBLE_EQ(info.strain_energy_density, strain_energy_density);
  EXPECT_DOUBLE_EQ(info.strain_energy, volume * strain_energy_density);

  // --- Part 2: Test create_empty_C3D4_common_info ---
  C3D4CommInfo empty_info = create_empty_C3D4_common_info();
  // Expect status to be DefaultInitialized.
  EXPECT_EQ(empty_info.status, DefaultInitialized);
  // Verify a few fields are zero.
  EXPECT_DOUBLE_EQ(empty_info.F11, 0.0);
  EXPECT_DOUBLE_EQ(empty_info.E11, 0.0);
  EXPECT_DOUBLE_EQ(empty_info.T11, 0.0);
  EXPECT_DOUBLE_EQ(empty_info.sigma11, 0.0);
  EXPECT_DOUBLE_EQ(empty_info.volume, 0.0);
  EXPECT_DOUBLE_EQ(empty_info.strain_energy_density, 0.0);
  EXPECT_DOUBLE_EQ(empty_info.strain_energy, 0.0);

  // --- Part 3: Test clear_C3D4_common_info ---
  // Clear the previously created info object.
  clear_C3D4_common_info(&info);
  // Status should be updated to InfoCleared.
  EXPECT_EQ(info.status, InfoCleared);
  // All fields should now be zero.
  EXPECT_DOUBLE_EQ(info.F11, 0.0);
  EXPECT_DOUBLE_EQ(info.F12, 0.0);
  EXPECT_DOUBLE_EQ(info.E11, 0.0);
  EXPECT_DOUBLE_EQ(info.T11, 0.0);
  EXPECT_DOUBLE_EQ(info.sigma11, 0.0);
  EXPECT_DOUBLE_EQ(info.volume, 0.0);
  EXPECT_DOUBLE_EQ(info.strain_energy_density, 0.0);
  EXPECT_DOUBLE_EQ(info.strain_energy, 0.0);
}
