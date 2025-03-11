#include "abaqus_subroutine_forge.h"
#include "communication.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "UnusedParameter"
#pragma ide diagnostic ignored "readability-non-const-parameter"

// configurations must be reviewed each time run ABAQUS with UEL
// element number of the inp file
#define ELEMENT_NUM 10l
// delta num of element no. - dummy element no. (note : element no is larger.)
#define DUMMY_ELEMENT_DELTA_NUM 100l

CPS3CommInfo comm_info[ELEMENT_NUM];

// old signature from ABAQUS docs:
// void uel(double *rhs, double *amatrx, double *svars, double *energy,
//         int *ndofel, int *nrhs, int *nsvars, double *props, int *nprops,
//         double *coords, int *mcrd, int *nnode, double *u, double *du,
//         double *v, double *a, int *jtype, double *time, double *dtime,
//         int *kstep, int *kinc, int *jelem, double *params, int *ndload,
//         int *jdltyp, double *adlmag, double *predef, int *npredf, int
//         *lflags, int *mlvarx, double *ddlmag, int *mdload, double *pnewdt,
//         int *jprops, int *njprop, double *period)
void uel(double *abaqus_residual_force_array, double *element_stiffness_matrix,
         double *solution_dependent_vars_array, double *energy_array,
         int *element_dof_num, int *load_vec_num,
         int *solution_dependent_vars_num, double *properties_array,
         int *properties_num, double *original_coords_array,
         int *max_node_dof_value, int *element_node_num,
         double *displacement_array,
         double *incremental_values_of_displacement_array,
         double *velocity_array, double *acceleration_array,
         int *element_type_ID, double *current_step_time_and_total_time_values,
         double *time_increment_value, int *current_step_num,
         int *current_increment_num, int *element_no_ID,
         double *params_for_implicit_dynamics, int *distributed_load_ID,
         int *distributed_load_type_integer,
         double *total_distributed_load_array,
         double *predefined_field_vars_array, int *predefined_field_vars_num,
         int *current_procedure_type_ID, int *rhs_vec_index_if_more_than_one,
         double *distribute_load_increment_mag,
         int *element_distributed_load_total_num,
         double *suggested_new_time_increment_ratio, int *int_properties_array,
         int *int_properties_num, double *time_period) {
  // basic passed-in variables
  double X1 = original_coords_array[0], Y1 = original_coords_array[1],
         X2 = original_coords_array[2], Y2 = original_coords_array[3],
         X3 = original_coords_array[4], Y3 = original_coords_array[5];
  double u1 = displacement_array[0], v1 = displacement_array[1],
         u2 = displacement_array[2], v2 = displacement_array[3],
         u3 = displacement_array[4], v3 = displacement_array[5];
  double C11 = properties_array[0], C12 = properties_array[1],
         C13 = properties_array[2], C21 = properties_array[3],
         C22 = properties_array[4], C23 = properties_array[5],
         C31 = properties_array[6], C32 = properties_array[7],
         C33 = properties_array[8];
  double initial_thickness = properties_array[9];
  CPS3NodalInfo X1Y1X2Y2X3Y3 = create_CPS3_nodal_info(X1, Y1, X2, Y2, X3, Y3);
  CPS3NodalInfo u1v1u2v2u3v3 = create_CPS3_nodal_info(u1, v1, u2, v2, u3, v3);
  CPS3NodalInfo x1y1x2y2x3y3 =
      CPS3_nodal_info_add(&X1Y1X2Y2X3Y3, &u1v1u2v2u3v3);
  Matrix3D C = create_matrix_3D(C11, C12, C13, C21, C22, C23, C31, C32, C33);

  // compute mechanical variables
  Matrix2D F = CPS3_nodal_disp_to_2D_F(&X1Y1X2Y2X3Y3, &u1v1u2v2u3v3);
  Matrix2D E = CPS3_2D_F_to_2D_E(&F);
  Matrix2D T = CPS3_2D_E_to_2D_T(&E, &C);
  Matrix2D sigma = CPS3_T_and_F_to_Cauchy(&T, &F);
  Matrix6D K = CPS3_compute_initial_element_stiffness_matrix(&X1Y1X2Y2X3Y3, &C,
                                                             initial_thickness);
  matrix_6D_fill_abaqus_double_array(&K, element_stiffness_matrix);

  double current_area = compute_CPS3_element_square(&x1y1x2y2x3y3);
  double current_thickness = initial_thickness / current_area;
  CPS3NodalInfo inner_force = CPS3_compute_inner_force(
      &X1Y1X2Y2X3Y3, &u1v1u2v2u3v3, &C, current_thickness);
  Vector6D inner_force_vec = CPS3_nodal_info_to_vector_6D(&inner_force);
  inner_force_vec = vector_6D_number_multiplication(-1, &inner_force_vec);
  vector_6D_fill_abaqus_double_array(&inner_force_vec,
                                     abaqus_residual_force_array);

  // fill in volume energy info
  double strain_energy_density = CPS3_E_and_T_to_strain_energy_density(&E, &T);
  CPS3VolumeEnergyInfo volume_energy_info = create_CPS3_volume_energy_info(
      current_area, current_thickness, strain_energy_density);

  // transfer info to umat for post-processing ODB results
  CPS3CommInfo element_comm_info = create_empty_CPS3_common_info();
  element_comm_info = create_CPS3_common_info_from_matrix_2Ds(
      ContainInfo, &F, &E, &T, &sigma, volume_energy_info);
  comm_info[*element_no_ID] = element_comm_info;
  clear_CPS3_common_info(&element_comm_info);
}

// only used for dummy element properties for viewing ODB results
// old signature from ABAQUS docs:
// void umat(double *stress_array, double *solution_dependent_vars_array, double
// *properties_matrix, double *sse,
//          double *spd, double *scd, double *rpl, double *ddsddt, double
//          *drplde, double *drpldt, double *strain_array, double
//          *strain_increment_array, double *time, double *dtime, double *temp,
//          double *dtemp, double *predef, double *dpred, char *cmname, int
//          *ndi, int *nshr, int *ntens, int *nstatv, double *props, int
//          *nprops, double *coords, double *drot, double *pnewdt, double
//          *celent, double *dfgrd0, double *dfgrd1, int *noel, int *npt, int
//          *layer, int *kspt, int *jstep, int *kinc)
void umat(double *stress_array, double *solution_dependent_vars_array,
          double *properties_matrix, double *elastic_strain_energy_value,
          double *plastic_dissipation_value, double *creep_dissipation_value,
          double *volumetric_heat_generation_rate_value,
          double *stress_increment_to_temperature,
          double *volumetric_heat_generation_to_strain,
          double *volumetric_heat_generation_to_temperature,
          double *strain_array, double *strain_increment_array,
          double *step_time_and_total_time, double *time_increment,
          double *temperature, double *temperature_increment,
          double *predefined_field_vars_array,
          double *predefined_field_vars_increment_array, char *material_name,
          int *direct_stress_components_num,
          int *engineering_shear_components_num,
          int *stress_or_strain_array_size, int *solution_dependent_vars_num,
          double *properties_array, int *properties_num,
          double *coordinates_array, double *rotation_increment_matrix,
          double *pnewdt, double *celent, double *begin_deformation_gradient,
          double *end_deformation_gradient, int *element_number_ID,
          int *integration_point_ID, int *layer_num_ID,
          int *section_point_num_within_layer, int *current_step_info_array,
          int *increment_num) {
  // elastic properties
  double E = properties_array[0];
  double nu = properties_array[1];

  // construct properties_matrix
  // initialization to avoid UB
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      properties_matrix[3 * i + j] = 0.0;
    }
  }
  // set values
  properties_matrix[0] = E / (1 - nu * nu);
  properties_matrix[4] = properties_matrix[0];
  properties_matrix[8] = E / (1 - nu * nu) * ((1 - nu) / 2.0);
  properties_matrix[1] = (E / (1 - nu * nu)) * nu;
  properties_matrix[3] = properties_matrix[1];

  // update stress_array
  stress_array[0] += properties_matrix[0] * strain_increment_array[0] +
                     properties_matrix[0] * nu * strain_increment_array[1];
  stress_array[1] += properties_matrix[0] * nu * strain_increment_array[0] +
                     properties_matrix[0] * strain_increment_array[1];
  stress_array[2] += properties_matrix[8] * strain_increment_array[2];

  // update SDVs info
  CPS3CommInfo CPS3_comm_info =
      comm_info[(long)element_number_ID - DUMMY_ELEMENT_DELTA_NUM];
  double *CPS3_info_double_malloc_array =
      CPS3_common_info_to_double_array_use_malloc(&CPS3_comm_info);
  for (int i = 0; i < COMM_INFO_ENTRY_NUM; ++i) {
    solution_dependent_vars_array[i] = CPS3_info_double_malloc_array[i];
  }
  free(CPS3_info_double_malloc_array);
}

#pragma clang diagnostic pop
