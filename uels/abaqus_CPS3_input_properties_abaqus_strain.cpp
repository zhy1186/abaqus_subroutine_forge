// warning : only valid for one-step calculation !

extern "C" {

#include "abaqus_subroutine_forge.h"
#include "communication.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "UnusedParameter"
#pragma ide diagnostic ignored "readability-non-const-parameter"

// configurations must be reviewed each time run ABAQUS with UEL
// element number of the inp file
#define ELEMENT_NUM 2

CPS3CommInfo comm_info[ELEMENT_NUM];

void uel(
    /* below 4 info should be updated */
    double *abaqus_residual_force_array /* RHS : contribution to rhs vector */,
    double *element_stiffness_matrix /* AMATRX : contribution to stiffness */,
    double
        *solution_dependent_vars_array /* SVARS : solution-dependent state */,
    double *energy_array /* ENERGY : energy quantities corresponding */,
    /* below variables passed in for information */
    int *element_dof_num /*NDOFEL : number of dof in the element */,
    int *load_vec_num /* NRHS : number of load vector, often = 1 for general */,
    int *solution_dependent_vars_num /* NSVARS : number of SDVs */,
    double *properties_array /* PROPS : real property values */,
    int *properties_num /* NPROPS : number of real property values */,
    double *original_coords_array /* COORDS : original coordinates of nodes */,
    int *max_node_dof_value /* MCRD : max{num_of_coords,largest_active_dof} */,
    int *element_node_num /* NNODE : number of nodes on element */,
    double *displacement_array /* U : array of displacement */,
    /* DU : incremental values of the current increment for rhs [? need test]*/
    double *incremental_values_of_displacement_array,
    double *velocity_array /* V : velocity array */,
    double *acceleration_array /* A : acceleration array */,
    int *element_type_ID /* JTYPE : element type */,
    /* TIME : array(2) contain : current value of step time and total time */
    double *current_step_time_and_total_time_values,
    double *time_increment_value /* DTIME : time increment */,
    int *current_step_num /* KSTEP : current step number */,
    int *current_increment_num /* KINC : current increment number */,
    int *element_no_ID /* JELEM : user-assigned element number */,
    double *params_for_implicit_dynamics /* PARAMS : implicit dynamics param */,
    /* below few vars contains distributed load and pre-defined field info */
    int *distributed_load_ID /* NDLOAD : ID of distribute load of element */,
    int *distributed_load_type_integer /* JDLTYP :  distribute load type */,
    double *total_distributed_load_array /* ADLMAG : total load magnitude */,
    double *predefined_field_vars_array /* PREDEF : predefined field vars */,
    int *predefined_field_vars_num /* NPREDF : num of predefined field vars */,
    int *current_procedure_type_ID /* LFLAGS : flags on step procedure type */,
    int *rhs_vec_index_if_more_than_one /* MLVARX : param on several rhs vec */,
    double *distribute_load_increment_mag /* DDLMAG : inc mag in dload */,
    int *element_distributed_load_total_num /* MDLOAD : total num of dload */,
    /* this info can be updated */
    double *suggested_new_time_increment_ratio /* PNEWDT : suggested new dt */,
    /* below 3 info make no sense in most case */
    int *int_properties_array /* JPROPS : array of int properties */,
    int *int_properties_num /* NJPROP : unm of int properties */,
    double *time_period /* PERIOD : time period of current step */) {
  // print head info
  UELCriticalInfo uel_critical_info = create_UEL_critical_info(
      element_no_ID, element_type_ID, current_step_num, current_increment_num);
  print_UEL_critical_info(&uel_critical_info);
  // print basic info
  UELBasicInfo uel_basic_info = create_UEL_basic_info(
      element_dof_num, solution_dependent_vars_num, properties_num,
      element_node_num, element_type_ID,
      current_step_time_and_total_time_values, time_increment_value,
      current_step_num, current_increment_num, element_no_ID);
  print_UEL_basic_info(&uel_basic_info);
  // print passed-in info
  UELPassedInInfo uel_passed_in_info = create_UEL_passed_in_info(
      properties_num, properties_array, element_dof_num, original_coords_array,
      displacement_array);
  print_UEL_passed_in_info(&uel_passed_in_info);

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
  double F11 = matrix_2D_get_element(&F, 0, 0);
  double F12 = matrix_2D_get_element(&F, 0, 1);
  double F21 = matrix_2D_get_element(&F, 1, 0);
  double F22 = matrix_2D_get_element(&F, 1, 1);
  Matrix2D Ft_mid = create_matrix_2D(1 + (F11 - 1) * 0.5, F12 * 0.5, F21 * 0.5,
                                     1 + (F22 - 1) * 0.5);
  Matrix2D Ft_mid_inv = matrix_2D_inverse(&Ft_mid);
  Matrix2D I = create_eye_matrix_2D();
  Matrix2D F_point = matrix_2D_minus(&Ft_mid, &I);
  Matrix2D Gamma = matrix_2D_mul_matrix_2D(&F_point, &Ft_mid_inv);
  Matrix2D GammaT = matrix_2D_transpose(&Gamma);
  Matrix2D deformation_rate_D_times_2 = matrix_2D_add(&Gamma, &GammaT);
  Matrix2D deformation_rate_D =
      matrix_2D_number_multiplication(0.5, &deformation_rate_D_times_2);
  double D11 = matrix_2D_get_element(&deformation_rate_D, 0, 0);
  double D12 = matrix_2D_get_element(&deformation_rate_D, 0, 1);
  double D21 = matrix_2D_get_element(&deformation_rate_D, 1, 0);
  double D22 = matrix_2D_get_element(&deformation_rate_D, 1, 1);
  Matrix2D strain = create_matrix_2D(2 * D11, 4 * D12, 4 * D21, 2 * D22);
  Matrix2D stress = CPS3_2D_strain_to_2D_stress(&strain, &C);
  Vector3D sigma_voigt = voigt_2D_matrix_to_3D_vector(&stress);
  Matrix6D K = CPS3_compute_initial_element_stiffness_matrix(&X1Y1X2Y2X3Y3, &C,
                                                             initial_thickness);
  matrix_6D_fill_abaqus_double_array(&K, element_stiffness_matrix);

  double current_area = compute_CPS3_element_square(&x1y1x2y2x3y3);
  double initial_area = compute_CPS3_element_square(&X1Y1X2Y2X3Y3);
  double current_thickness = initial_thickness * initial_area / current_area;
  MatrixB36 B = CPS3_compute_matrix_B(&x1y1x2y2x3y3);
  MatrixB63 B_T = create_matrix_B63_from_B36(&B);
  Vector6D BT_times_sigma = CPS3_matrix_B63_mul_vector_3D(&B_T, &sigma_voigt);
  Vector6D inner_force = vector_6D_number_multiplication(
      initial_thickness * current_area, &BT_times_sigma);

  inner_force = vector_6D_number_multiplication(-1, &inner_force);
  vector_6D_fill_abaqus_double_array(&inner_force, abaqus_residual_force_array);

  // fill in volume energy info
  double strain_energy_density =
      CPS3_E_and_T_to_strain_energy_density(&strain, &stress);
  CPS3VolumeEnergyInfo volume_energy_info = create_CPS3_volume_energy_info(
      current_area, current_thickness, strain_energy_density);
  // update energy info to ABAQUS
  // todo : energy(8) (distributed work done) is not update for
  // now.
  energy_array[0] = 0;  // kinetic energy
  energy_array[1] = strain_energy_density * current_area * current_thickness;

  // transfer info to umat for post-processing ODB results
  CPS3CommInfo element_comm_info = create_empty_CPS3_common_info();
  element_comm_info = create_CPS3_common_info_from_matrix_2Ds(
      ContainInfo, &F, &strain, &stress, &stress, volume_energy_info);
  comm_info[*element_no_ID] = element_comm_info;
  clear_CPS3_common_info(&element_comm_info);
}

// only used for dummy element properties for viewing ODB results
void umat(
    /* below variables need to be defined */
    double *stress_array /* STRESS : stress tensor, must be updated */,
    double *solution_dependent_vars_array /* STATEV : SDVs */,
    double *properties_matrix /* DDSDDE : Jacobian (constitutive model) */,
    double *elastic_strain_energy_value /* SSE : elastic strain energy */,
    double *plastic_dissipation_value /* SPD :plastic dissipation */,
    double *creep_dissipation_value /* SCD : creep dissipation */,
    /* below variables only in a full or coupled thermal stress analysis */
    double *volumetric_heat_generation_rate_value /* RPL : heat per time */,
    double *stress_increment_to_temperature /* DDSDDT : stress to temp */,
    double *volumetric_heat_generation_to_strain /* DRPLDE : var of RPL */,
    double *volumetric_heat_generation_to_temperature /* DRPLDT : variation */,
    /* below variables passed in for information */
    double *strain_array /* STRAN : total strain tensor */,
    double *strain_increment_array /* DSTRAN : strain increment */,
    double *step_time_and_total_time /* TIME : time step and total time*/,
    double *time_increment /* DTIME : time increment */,
    double *temperature /* TEMP : temperature */,
    double *temperature_increment /* DTEMP : increment of temperature */,
    double *predefined_field_vars_array /* PREDEF : predefined field array */,
    double *predefined_field_vars_increment_array /* DPRED : inc of PREDEF */,
    char *material_name /* CNAME : User-defined material name */,
    /* NDI : number of direct stress components */
    int *direct_stress_components_num /* NDI */,
    /* NSHR : number of engineering shear stress components */
    int *engineering_shear_components_num /* NSHR */,
    /* NTENS : size of the stress or strain component array */
    int *stress_or_strain_array_size /* NTENS = NDI + NSHR */,
    int *solution_dependent_vars_num /* NSTATV : number of SDV variables */,
    /* PROPS : User-specified array of material constants*/
    double *properties_array /* PROPS */,
    int *properties_num /* NPROPS : User-defined consts number */,
    double *coordinates_array /* COORDS: coordinates of this point */,
    /* note : COORDS are current coordinates if geometric nonlinear is ON,
       otherwise contains the original coordinates of the point */
    double *rotation_increment_matrix /* DROT : rotation increment matrix */,
    /* this info can be updated */
    double *suggested_new_time_increment_ratio /* PNEWDT : suggested new dt */,
    /* below info may be used */
    double *celent /* CELENT : characteristic element length */,
    /* note : F has the dimension of 3 x 3 */
    double *begin_deformation_gradient /* DFGRD0 : F at begin of increment */,
    double *end_deformation_gradient /* DFGRD1 : F at end of the increment */,
    int *element_no_ID /* NOEL : element number */,
    int *integration_point_ID /* NPT : integration point number */,
    int *layer_num_ID /* LAYER : layer number for composites */,
    int *section_point_num_within_layer /* KSPT : point number with layer */,
    /* JSTEP[0] : step number */
    /* JSTEP[1] : procedure type key */
    /* JSTEP[2] : NLGEOM = YES is 1, other is 0 */
    /* JSTEP[3] : if current step is linear perturbation procedure */
    int *current_step_info_array /* JSTEP : step info */,
    int *increment_num /* KINC : increment number */) {
  // print basic info for UMAT
  UMATBasicInfo umat_basic_info =
      create_UMAT_basic_info(element_no_ID, integration_point_ID,
                             &current_step_info_array[0], increment_num);
  print_UMAT_basic_info(&umat_basic_info);

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
  CPS3CommInfo CPS3_comm_info = comm_info[(*element_no_ID) - ELEMENT_NUM];
  double *CPS3_info_double_malloc_array =
      CPS3_common_info_to_double_array_use_malloc(&CPS3_comm_info);
  for (int i = 0; i < CPS3_COMM_INFO_ENTRY_NUM; ++i) {
    solution_dependent_vars_array[i] = CPS3_info_double_malloc_array[i];
  }
  free(CPS3_info_double_malloc_array);
}

#pragma clang diagnostic pop
}