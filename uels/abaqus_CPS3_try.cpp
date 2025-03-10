#include "abaqus_subroutine_forge.h"
#include "communication.h"

#define ELEMENT_NUM 10

double comm_info[ELEMENT_NUM][COMM_INFO_NUM];

void uel(double *rhs, double *amatrx, double *svars, double *energy,
         int *ndofel, int *nrhs, int *nsvars, double *props, int *nprops,
         double *coords, int *mcrd, int *nnode, double *u, double *du,
         double *v, double *a, int *jtype, double *time, double *dtime,
         int *kstep, int *kinc, int *jelem, double *params, int *ndload,
         int *jdltip, double *adlmag, double *predef, int *npredf, int *lflags,
         int *mlvarx, double *ddlmag, int *mdload, double *pnewdt, int *jprops,
         int *njprop, double *period) {

  // transfer info to umat for post-processing ODB results


}

// only used for dummpy element properties for viewing ODB results
void umat(double *stress, double *statev, double *ddsdde, double *sse,
          double *spd, double *scd, double *rpl, double *ddsddt, double *drplde,
          double *drpldt, double *stran, double *dstran, double *time,
          double *dtime, double *temp, double *dtemp, double *predef,
          double *dpred, char *cmname, int *ndi, int *nshr, int *ntens,
          int *nstatv, double *props, int *nprops, double *coords, double *drot,
          double *pnewdt, double *celent, double *dfgrd0, double *dfgrd1,
          int *noel, int *npt, int *layer, int *kspt, int *kstep, int *kinc,
          short cmname_len) {
  // elastic properties
  double E = props[0];
  double nu = props[1];

  // construct ddsdde
  // initialization to avoid UB
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ddsdde[3 * i + j] = 0.0;
    }
  }
  // set values
  ddsdde[0] = E / (1 - nu * nu);
  ddsdde[4] = ddsdde[0];
  ddsdde[8] = E / (1 - nu * nu) * ((1 - nu) / 2.0);
  ddsdde[1] = (E / (1 - nu * nu)) * nu;
  ddsdde[3] = ddsdde[1];

  // update stress
  stress[0] += ddsdde[0] * dstran[0] + ddsdde[0] * nu * dstran[1];
  stress[1] += ddsdde[0] * nu * dstran[0] + ddsdde[0] * dstran[1];
  stress[2] += ddsdde[8] * dstran[2];

  // update SDVs info
}