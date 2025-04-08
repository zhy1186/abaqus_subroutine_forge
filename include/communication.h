//
// Created by Hengyi Zhao on 2025/3/10.
//

#pragma once

#pragma clang diagnostic push
#pragma ide diagnostic ignored "modernize-use-auto"

#include "abaqus_subroutine_forge.h"

#define CPS3_COMM_INFO_ENTRY_NUM 22
#define C3D4_COMM_INFO_ENTRY_NUM 40

// design for easy-to-use for damn ABAQUS, not for elegance

void log_info(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  fflush(stderr);
  va_end(args);
}

void log_debug(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  fprintf(stderr,
          "\n\n==================== DEBUG output ====================\n\n");
  vfprintf(stderr, fmt, args);
  fprintf(stderr,
          "\n\n^^^^^^^^^^^^^^^^^^^^ ^^^^^ ^^^^^^ ^^^^^^^^^^^^^^^^^^^^\n\n");
  fflush(stderr);
  va_end(args);
}

typedef enum { TYPE_INT, TYPE_DOUBLE } DebugOutputArrayType;

void log_debug_array(const char *name, DebugOutputArrayType type, int length,
                     void *array) {
  fprintf(stderr,
          "\n\n==================== DEBUG output ====================\n\n");
  fprintf(stderr, "array name : %s\n", name);
  for (int i = 0; i < length; ++i) {
    if (type == TYPE_DOUBLE) {
      fprintf(stderr, "%s [%d] = %f\n", name, i, ((double *)array)[i]);
    } else if (type == TYPE_INT) {
      fprintf(stderr, "%s [%d] = %d\n", name, i, ((int *)array)[i]);
    } else {
      fatal_error("Unsupported array type : %d\n", type);
    }
  }
  fprintf(stderr,
          "\n\n^^^^^^^^^^^^^^^^^^^^ ^^^^^ ^^^^^^ ^^^^^^^^^^^^^^^^^^^^\n\n");
  fflush(stderr);
}

// todo : add support as to python derivative for K and finner
// todo : support negative eigen value calculate (util)

typedef enum { DefaultInitialized, ContainInfo, InfoCleared } UelInfoStatus;

typedef struct CPS3VolumeEnergyInfo {
  double area;
  double thickness;
  double volume;
  double strain_energy_density;
  double strain_energy;
} CPS3VolumeEnergyInfo;

CPS3VolumeEnergyInfo create_empty_CPS3_volume_energy_info() {
  CPS3VolumeEnergyInfo info;
  info.area = 0;
  info.thickness = 0;
  info.volume = 0;
  info.strain_energy_density = 0;
  info.strain_energy = 0;
  return info;
}

typedef struct CPS3CommInfo {
  // status
  UelInfoStatus status;

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
  CPS3VolumeEnergyInfo info;
  info.area = area;
  info.thickness = thickness;
  info.volume = area * thickness;
  info.strain_energy_density = strain_energy_density;
  info.strain_energy = strain_energy_density * area * thickness;
  return info;
}

CPS3CommInfo create_CPS3_common_info_from_matrix_2Ds(
    const UelInfoStatus status, const Matrix2D *F, const Matrix2D *E,
    const Matrix2D *T, const Matrix2D *sigma,
    CPS3VolumeEnergyInfo volume_energy_info) {
  CPS3CommInfo info;
  // status
  info.status = status;
  // deformation gradient
  info.F11 = matrix_2D_get_element(F, 0, 0);
  info.F12 = matrix_2D_get_element(F, 0, 1);
  info.F21 = matrix_2D_get_element(F, 1, 0);
  info.F22 = matrix_2D_get_element(F, 1, 1);

  // green strain
  info.E11 = matrix_2D_get_element(E, 0, 0);
  info.E12 = matrix_2D_get_element(E, 0, 1);
  info.E21 = matrix_2D_get_element(E, 1, 0);
  info.E22 = matrix_2D_get_element(E, 1, 1);

  // PK2 stress
  info.T11 = matrix_2D_get_element(T, 0, 0);
  info.T12 = matrix_2D_get_element(T, 0, 1);
  info.T21 = matrix_2D_get_element(T, 1, 0);
  info.T22 = matrix_2D_get_element(T, 1, 1);

  // Cauchy stress
  info.sigma11 = matrix_2D_get_element(sigma, 0, 0);
  info.sigma12 = matrix_2D_get_element(sigma, 0, 1);
  info.sigma21 = matrix_2D_get_element(sigma, 1, 0);
  info.sigma22 = matrix_2D_get_element(sigma, 1, 1);

  // volume and energy info
  info.volume_energy_info = volume_energy_info;
  return info;
}

CPS3CommInfo create_empty_CPS3_common_info() {
  CPS3CommInfo info;
  info.status = DefaultInitialized;

  // deformation gradient
  info.F11 = 0;
  info.F12 = 0;
  info.F21 = 0;
  info.F22 = 0;

  // green strain
  info.E11 = 0;
  info.E12 = 0;
  info.E21 = 0;
  info.E22 = 0;

  // PK2 stress
  info.T11 = 0;
  info.T12 = 0;
  info.T21 = 0;
  info.T22 = 0;

  // Cauchy stress
  info.sigma11 = 0;
  info.sigma12 = 0;
  info.sigma21 = 0;
  info.sigma22 = 0;

  info.volume_energy_info = create_empty_CPS3_volume_energy_info();
  return info;
}

void clear_CPS3_common_info(CPS3CommInfo *info) {
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
double *CPS3_common_info_to_double_array_use_malloc(const CPS3CommInfo *info) {
  if (info->status == InfoCleared || info->status == DefaultInitialized) {
    fatal_error("CPS3CommInfo do not contain valid info!\n");
  }

  double *rtn = (double *)malloc(CPS3_COMM_INFO_ENTRY_NUM * sizeof(double));
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

typedef struct C3D4CommInfo {
  // status
  UelInfoStatus status;

  // deformation gradient
  double F11;
  double F12;
  double F13;
  double F21;
  double F22;
  double F23;
  double F31;
  double F32;
  double F33;

  // green strain
  double E11;
  double E12;
  double E13;
  double E21;
  double E22;
  double E23;
  double E31;
  double E32;
  double E33;

  // PK2 stress tensor
  double T11;
  double T12;
  double T13;
  double T21;
  double T22;
  double T23;
  double T31;
  double T32;
  double T33;

  // Cauchy stress tensor
  double sigma11;
  double sigma12;
  double sigma13;
  double sigma21;
  double sigma22;
  double sigma23;
  double sigma31;
  double sigma32;
  double sigma33;

  // Energy and volume
  double volume;
  double strain_energy_density;
  double strain_energy;
} C3D4CommInfo;

C3D4CommInfo create_C3D4_common_info_from_matrix_3Ds(
    const UelInfoStatus status, const Matrix3D *F, const Matrix3D *E,
    const Matrix3D *T, const Matrix3D *sigma, double volume,
    double strain_energy_density) {
  C3D4CommInfo info;
  // status
  info.status = status;
  // deformation gradient
  info.F11 = matrix_3D_get_element(F, 0, 0);
  info.F12 = matrix_3D_get_element(F, 0, 1);
  info.F13 = matrix_3D_get_element(F, 0, 2);
  info.F21 = matrix_3D_get_element(F, 1, 0);
  info.F22 = matrix_3D_get_element(F, 1, 1);
  info.F23 = matrix_3D_get_element(F, 1, 2);
  info.F31 = matrix_3D_get_element(F, 2, 0);
  info.F32 = matrix_3D_get_element(F, 2, 1);
  info.F33 = matrix_3D_get_element(F, 2, 2);

  // green strain
  info.E11 = matrix_3D_get_element(E, 0, 0);
  info.E12 = matrix_3D_get_element(E, 0, 1);
  info.E13 = matrix_3D_get_element(E, 0, 2);
  info.E21 = matrix_3D_get_element(E, 1, 0);
  info.E22 = matrix_3D_get_element(E, 1, 1);
  info.E23 = matrix_3D_get_element(E, 1, 2);
  info.E31 = matrix_3D_get_element(E, 2, 0);
  info.E32 = matrix_3D_get_element(E, 2, 1);
  info.E33 = matrix_3D_get_element(E, 2, 2);

  // PK2 stress
  info.T11 = matrix_3D_get_element(T, 0, 0);
  info.T12 = matrix_3D_get_element(T, 0, 1);
  info.T13 = matrix_3D_get_element(T, 0, 2);
  info.T21 = matrix_3D_get_element(T, 1, 0);
  info.T22 = matrix_3D_get_element(T, 1, 1);
  info.T23 = matrix_3D_get_element(T, 1, 2);
  info.T31 = matrix_3D_get_element(T, 2, 0);
  info.T32 = matrix_3D_get_element(T, 2, 1);
  info.T33 = matrix_3D_get_element(T, 2, 2);

  // Cauchy stress
  info.sigma11 = matrix_3D_get_element(sigma, 0, 0);
  info.sigma12 = matrix_3D_get_element(sigma, 0, 1);
  info.sigma13 = matrix_3D_get_element(sigma, 0, 2);
  info.sigma21 = matrix_3D_get_element(sigma, 1, 0);
  info.sigma22 = matrix_3D_get_element(sigma, 1, 1);
  info.sigma23 = matrix_3D_get_element(sigma, 1, 2);
  info.sigma31 = matrix_3D_get_element(sigma, 2, 0);
  info.sigma32 = matrix_3D_get_element(sigma, 2, 1);
  info.sigma33 = matrix_3D_get_element(sigma, 2, 2);

  // volume and energy info
  info.volume = volume;
  info.strain_energy_density = strain_energy_density;
  info.strain_energy = volume * strain_energy_density;
  return info;
}

C3D4CommInfo create_empty_C3D4_common_info() {
  C3D4CommInfo info;
  info.status = DefaultInitialized;

  info.F11 = 0;
  info.F12 = 0;
  info.F13 = 0;
  info.F21 = 0;
  info.F22 = 0;
  info.F23 = 0;
  info.F31 = 0;
  info.F32 = 0;
  info.F33 = 0;

  info.E11 = 0;
  info.E12 = 0;
  info.E13 = 0;
  info.E21 = 0;
  info.E22 = 0;
  info.E23 = 0;
  info.E31 = 0;
  info.E32 = 0;
  info.E33 = 0;

  info.T11 = 0;
  info.T12 = 0;
  info.T13 = 0;
  info.T21 = 0;
  info.T22 = 0;
  info.T23 = 0;
  info.T31 = 0;
  info.T32 = 0;
  info.T33 = 0;

  info.sigma11 = 0;
  info.sigma12 = 0;
  info.sigma13 = 0;
  info.sigma21 = 0;
  info.sigma22 = 0;
  info.sigma23 = 0;
  info.sigma31 = 0;
  info.sigma32 = 0;
  info.sigma33 = 0;

  info.volume = 0;
  info.strain_energy_density = 0;
  info.strain_energy = 0;

  return info;
}

void clear_C3D4_common_info(C3D4CommInfo *info) {
  info->status = InfoCleared;
  info->F11 = 0;
  info->F12 = 0;
  info->F13 = 0;
  info->F21 = 0;
  info->F22 = 0;
  info->F23 = 0;
  info->F31 = 0;
  info->F32 = 0;
  info->F33 = 0;

  info->E11 = 0;
  info->E12 = 0;
  info->E13 = 0;
  info->E21 = 0;
  info->E22 = 0;
  info->E23 = 0;
  info->E31 = 0;
  info->E32 = 0;
  info->E33 = 0;

  info->T11 = 0;
  info->T12 = 0;
  info->T13 = 0;
  info->T21 = 0;
  info->T22 = 0;
  info->T23 = 0;
  info->T31 = 0;
  info->T32 = 0;
  info->T33 = 0;

  info->sigma11 = 0;
  info->sigma12 = 0;
  info->sigma13 = 0;
  info->sigma21 = 0;
  info->sigma22 = 0;
  info->sigma23 = 0;
  info->sigma31 = 0;
  info->sigma32 = 0;
  info->sigma33 = 0;

  info->volume = 0;
  info->strain_energy_density = 0;
  info->strain_energy = 0;
}

double *C3D4_common_info_to_double_array_use_malloc(const C3D4CommInfo *info) {
  if (info->status == InfoCleared || info->status == DefaultInitialized) {
    fatal_error("C3D4CommInfo do not contain valid info!\n");
  }

  double *rtn = (double *)malloc(C3D4_COMM_INFO_ENTRY_NUM * sizeof(double));
  rtn[0] = info->F11;
  rtn[1] = info->F12;
  rtn[2] = info->F13;
  rtn[3] = info->F21;
  rtn[4] = info->F22;
  rtn[5] = info->F23;
  rtn[6] = info->F31;
  rtn[7] = info->F32;
  rtn[8] = info->F33;

  rtn[9] = info->E11;
  rtn[10] = info->E12;
  rtn[11] = info->E13;
  rtn[12] = info->E21;
  rtn[13] = info->E22;
  rtn[14] = info->E23;
  rtn[15] = info->E31;
  rtn[16] = info->E32;
  rtn[17] = info->E33;

  rtn[18] = info->T11;
  rtn[19] = info->T12;
  rtn[20] = info->T13;
  rtn[21] = info->T21;
  rtn[22] = info->T22;
  rtn[23] = info->T23;
  rtn[24] = info->T31;
  rtn[25] = info->T32;
  rtn[26] = info->T33;

  rtn[27] = info->sigma11;
  rtn[28] = info->sigma12;
  rtn[29] = info->sigma13;
  rtn[30] = info->sigma21;
  rtn[31] = info->sigma22;
  rtn[32] = info->sigma23;
  rtn[33] = info->sigma31;
  rtn[34] = info->sigma32;
  rtn[35] = info->sigma33;

  rtn[36] = info->volume;
  rtn[37] = info->strain_energy_density;
  rtn[38] = info->strain_energy;
  rtn[39] = (double)info->status;

  return rtn;
}

typedef struct UELBasicInfo {
  int element_dof_num;
  int solution_dependent_vars_num;
  int properties_num;
  int element_node_num;
  int element_type_ID;
  double current_step_time;
  double total_time;
  double time_increment_value;
  int current_step_num;
  int current_increment_num;
  int element_no_ID;
} UELBasicInfo;

UELBasicInfo create_UEL_basic_info(
    const int *element_dof_num, const int *solution_dependent_vars_num,
    const int *properties_num, const int *element_node_num,
    const int *element_type_ID,
    const double *current_step_time_and_total_time_values,
    const double *time_increment_value, const int *current_step_num,
    const int *current_increment_num, const int *element_no_ID) {
  UELBasicInfo info;
  info.element_dof_num = *element_dof_num;
  info.solution_dependent_vars_num = *solution_dependent_vars_num;
  info.properties_num = *properties_num;
  info.element_node_num = *element_node_num;
  info.element_type_ID = *element_type_ID;
  info.current_step_time = current_step_time_and_total_time_values[0];
  info.total_time = current_step_time_and_total_time_values[1];
  info.time_increment_value = *time_increment_value;
  info.current_step_num = *current_step_num;
  info.current_increment_num = *current_increment_num;
  info.element_no_ID = *element_no_ID;
  return info;
}

void print_UEL_basic_info(const UELBasicInfo *info) {
  log_info("******************** UEL basic info ********************\n");
  log_info("* [UEL basic info] : Element ID: %d, Element Type: %d\n",
           info->element_no_ID, info->element_type_ID);
  log_info("* [UEL basic info] : SDV number : %d, Properties number : %d\n",
           info->solution_dependent_vars_num, info->properties_num);
  log_info(
      "* [UEL basic info] : Element node number : %d, Element dof number : "
      "%d\n",
      info->element_type_ID, info->element_node_num, info->element_dof_num);
  log_info(
      "* [Step info] : Current step time: %f, Total time : %f, Time increment "
      "value : %f\n",
      info->current_step_time, info->total_time, info->time_increment_value);
  log_info(
      "* [Step info] : Current step number : %d, current increment number : "
      "%d\n",
      info->current_step_num, info->current_increment_num);
  log_info("******************** ^^^ ^^^^^ ^^^^ ********************\n");
}

typedef struct UELPassedInInfo {
  int properties_num;
  double *properties_array;
  int element_dof_num;
  double *original_coords_array;
  double *displacement_array;
} UELPassedInInfo;

UELPassedInInfo create_UEL_passed_in_info(const int *properties_num,
                                          double *properties_array,
                                          const int *element_dof_num,
                                          double *original_coords_array,
                                          double *displacement_array) {
  UELPassedInInfo info;
  info.properties_num = *properties_num;
  info.properties_array = properties_array;
  info.element_dof_num = *element_dof_num;
  info.original_coords_array = original_coords_array;
  info.displacement_array = displacement_array;
  return info;
}

void print_UEL_passed_in_info(const UELPassedInInfo *info) {
  log_info("******************** UEL passed-in info ********************\n");
  log_info("* [UEL passed-in info] : Properties number: %d, Element dof: %d\n",
           info->properties_num, info->element_dof_num);
  log_info("* [UEL passed-in info] : Properties:\n");
  for (int i = 0; i < info->properties_num; ++i) {
    log_info("* [prop %d of %d] : %f\n", i, info->properties_num,
             info->properties_array[i]);
  }
  log_info("* [UEL passed-in info] : Original coordinates:\n");
  for (int i = 0; i < info->element_dof_num; ++i) {
    log_info("* [X %d of %d] : %f\n", i, info->element_dof_num,
             info->original_coords_array[i]);
  }
  log_info("* [UEL passed-in info] : Displacement:\n");
  for (int i = 0; i < info->element_dof_num; ++i) {
    log_info("* [X %d of %d] : %f\n", i, info->element_dof_num,
             info->displacement_array[i]);
  }
  log_info("******************** ^^^ ^^^^^^^^^ ^^^^ ********************\n");
}

typedef struct UELCriticalInfo {
  int element_no_ID;
  int element_type_ID;
  int current_step_num;
  int current_increment_num;
} UELCriticalInfo;

UELCriticalInfo create_UEL_critical_info(const int *element_no_ID,
                                         const int *element_type_ID,
                                         const int *current_step_num,
                                         const int *current_increment_num) {
  UELCriticalInfo info;
  info.element_no_ID = *element_no_ID;
  info.element_type_ID = *element_type_ID;
  info.current_step_num = *current_step_num;
  info.current_increment_num = *current_increment_num;
  return info;
}

void print_UEL_critical_info(const UELCriticalInfo *info) {
  log_info(
      "UEL called by element no. [%d], element type = [%d], in step [%d], "
      "increment [%d]\n",
      info->element_no_ID, info->element_type_ID, info->current_step_num,
      info->current_increment_num);
  log_info(
      "======================== [ELEMENT : %d] === [INC : "
      "%d] ========================\n",
      info->element_no_ID, info->current_increment_num);
}

typedef struct UMATBasicInfo {
  int element_no_ID;
  int integration_point_ID;
  int step_number;
  int increment_number;
} UMATBasicInfo;

UMATBasicInfo create_UMAT_basic_info(const int *element_no_ID,
                                     const int *integration_point_ID,
                                     const int *step_number,
                                     const int *increment_number) {
  UMATBasicInfo info;
  info.element_no_ID = *element_no_ID;
  info.integration_point_ID = *integration_point_ID;
  info.step_number = *step_number;
  info.increment_number = *increment_number;
  return info;
}

void print_UMAT_basic_info(const UMATBasicInfo *info) {
  log_info(
      "UMAT called by element no. [%d], integration point no. [%d], in step "
      "[%d], increment [%d]\n",
      info->element_no_ID, info->integration_point_ID, info->step_number,
      info->increment_number);
}

#pragma clang diagnostic pop
