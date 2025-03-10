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
