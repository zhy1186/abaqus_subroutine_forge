//
// Created by Hengyi Zhao on 2025/3/10.
//

#pragma once

#pragma clang diagnostic push
#pragma ide diagnostic ignored "modernize-use-auto"

#include "abaqus_subroutine_forge.h"

#define COMM_INFO_ENTRY_NUM 22

// design for easy-to-use for damn ABAQUS, not for elegance

typedef enum { DefaultInitialized, ContainInfo, InfoCleared } CPS3InfoStatus;

typedef struct CPS3VolumeEnergyInfo {
  double area;
  double thickness;
  double volume;
  double strain_energy_density;
  double strain_energy;
} CPS3VolumeEnergyInfo;

CPS3VolumeEnergyInfo create_empty_CPS3_volume_energy_info() {
  CPS3VolumeEnergyInfo info = {.area = 0,
                               .thickness = 0,
                               .volume = 0,
                               .strain_energy_density = 0,
                               .strain_energy = 0};
  return info;
}

typedef struct CPS3CommInfo {
  // status
  CPS3InfoStatus status;

  // deformation gradient
  double F11;
  double F12;
  double F21;
  double F22;

  // green strain
  double E11;
  double E12;
  double E21;
  double E22;

  // PK2 stress tensor
  double T11;
  double T12;
  double T21;
  double T22;

  // Cauchy stress tensor
  double sigma11;
  double sigma12;
  double sigma21;
  double sigma22;

  // Energy
  CPS3VolumeEnergyInfo volume_energy_info;
} CPS3CommInfo;

CPS3VolumeEnergyInfo create_CPS3_volume_energy_info(
    double area, double thickness, double strain_energy_density) {
  CPS3VolumeEnergyInfo info = {
      .area = area,
      .thickness = thickness,
      .volume = area * thickness,
      .strain_energy_density = strain_energy_density,
      .strain_energy = strain_energy_density * area * thickness};
  return info;
}

CPS3CommInfo create_CPS3_common_info_from_matrix_2Ds(
    const CPS3InfoStatus status, const Matrix2D* F, const Matrix2D* E,
    const Matrix2D* T, const Matrix2D* sigma,
    CPS3VolumeEnergyInfo volume_energy_info) {
  CPS3CommInfo info = {// status
                       .status = status,
                       // deformation gradient
                       .F11 = matrix_2D_get_element(F, 0, 0),
                       .F12 = matrix_2D_get_element(F, 0, 1),
                       .F21 = matrix_2D_get_element(F, 1, 0),
                       .F22 = matrix_2D_get_element(F, 1, 1),

                       // green strain
                       .E11 = matrix_2D_get_element(E, 0, 0),
                       .E12 = matrix_2D_get_element(E, 0, 1),
                       .E21 = matrix_2D_get_element(E, 1, 0),
                       .E22 = matrix_2D_get_element(E, 1, 1),

                       // PK2 stress
                       .T11 = matrix_2D_get_element(T, 0, 0),
                       .T12 = matrix_2D_get_element(T, 0, 1),
                       .T21 = matrix_2D_get_element(T, 1, 0),
                       .T22 = matrix_2D_get_element(T, 1, 1),

                       // Cauchy stress
                       .sigma11 = matrix_2D_get_element(sigma, 0, 0),
                       .sigma12 = matrix_2D_get_element(sigma, 0, 1),
                       .sigma21 = matrix_2D_get_element(sigma, 1, 0),
                       .sigma22 = matrix_2D_get_element(sigma, 1, 1),

                       // volume and energy info
                       .volume_energy_info = volume_energy_info};
  return info;
}

CPS3CommInfo create_empty_CPS3_common_info() {
  CPS3CommInfo info = {
      // status
      .status = DefaultInitialized,
      // deformation gradient
      .F11 = 0,
      .F12 = 0,
      .F21 = 0,
      .F22 = 0,

      // green strain
      .E11 = 0,
      .E12 = 0,
      .E21 = 0,
      .E22 = 0,

      // PK2 stress
      .T11 = 0,
      .T12 = 0,
      .T21 = 0,
      .T22 = 0,

      // Cauchy stress
      .sigma11 = 0,
      .sigma12 = 0,
      .sigma21 = 0,
      .sigma22 = 0,

      // volume and energy info
      .volume_energy_info = create_empty_CPS3_volume_energy_info()};
  return info;
}

void clear_CPS3_common_info(CPS3CommInfo* info) {
  info->status = InfoCleared;
  info->F11 = 0;
  info->F12 = 0;
  info->F21 = 0;
  info->F22 = 0;
  info->E11 = 0;
  info->E12 = 0;
  info->E21 = 0;
  info->E22 = 0;
  info->T11 = 0;
  info->T12 = 0;
  info->T21 = 0;
  info->T22 = 0;
  info->sigma11 = 0;
  info->sigma12 = 0;
  info->sigma21 = 0;
  info->sigma22 = 0;
  info->volume_energy_info = create_empty_CPS3_volume_energy_info();
}

// should only be used in dummy UMAT because of stack memory
double* CPS3_common_info_to_double_array_use_malloc(const CPS3CommInfo* info) {
  if (info->status == InfoCleared || info->status == DefaultInitialized) {
    fatal_error("CPS3CommInfo do not contain valid info!");
  }

  double* rtn = (double*)malloc(COMM_INFO_ENTRY_NUM * sizeof(double));
  rtn[0] = info->F11;
  rtn[1] = info->F12;
  rtn[2] = info->F21;
  rtn[3] = info->F22;

  rtn[4] = info->E11;
  rtn[5] = info->E12;
  rtn[6] = info->E21;
  rtn[7] = info->E22;

  rtn[8] = info->T11;
  rtn[9] = info->T12;
  rtn[10] = info->T21;
  rtn[11] = info->T22;

  rtn[12] = info->sigma11;
  rtn[13] = info->sigma12;
  rtn[14] = info->sigma21;
  rtn[15] = info->sigma22;

  rtn[16] = info->volume_energy_info.area;
  rtn[17] = info->volume_energy_info.thickness;
  rtn[18] = info->volume_energy_info.volume;
  rtn[19] = info->volume_energy_info.strain_energy_density;
  rtn[20] = info->volume_energy_info.strain_energy;

  rtn[21] = (double)info->status;
  return rtn;
}

#pragma clang diagnostic pop
