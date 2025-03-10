// uel_demo.c -- first UEL demo for black-box tests
// Created by Hengyi Zhao (zhaohy19@mails.thu.edu.cn) on 2025/03/06
//

extern "C" {

double common[20];

void uel(double *rhs, double *amatrx, double *svars, double *energy,
         int *ndofel, int *nrhs, int *nsvars, double *props, int *nprops,
         double *coords, int *mcrd, int *nnode, double *u, double *du,
         double *v, double *a, int *jtype, double *time, double *dtime,
         int *kstep, int *kinc, int *jelem, double *params, int *ndload,
         int *jdltip, double *adlmag, double *predef, int *npredf, int *lflags,
         int *mlvarx, double *ddlmag, int *mdload, double *pnewdt, int *jprops,
         int *njprop, double *period) {
  // 1. 材料属性
  double E = props[0];   // 杨氏模量
  double nu = props[1];  // 泊松比

  // 平面应力刚度矩阵的材料常数矩阵 D
  double D[3][3];
  double factor = E / (1.0 - nu * nu);
  D[0][0] = factor;
  D[0][1] = factor * nu;
  D[1][0] = factor * nu;
  D[1][1] = factor;
  D[2][2] = factor * (1.0 - nu) / 2.0;

  // 2. 节点坐标
  double x1 = coords[0], y1 = coords[1];
  double x2 = coords[2], y2 = coords[3];
  double x3 = coords[4], y3 = coords[5];

  // 3. 计算单元面积
  double area = 0.5 * ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1));

  // 4. 计算形函数的导数
  double B[3][6];  // B矩阵，3行6列
  B[0][0] = (y2 - y3) / (2.0 * area);
  B[0][2] = (y3 - y1) / (2.0 * area);
  B[0][4] = (y1 - y2) / (2.0 * area);
  B[1][1] = (x3 - x2) / (2.0 * area);
  B[1][3] = (x1 - x3) / (2.0 * area);
  B[1][5] = (x2 - x1) / (2.0 * area);
  B[2][0] = B[1][1];
  B[2][1] = B[0][0];
  B[2][2] = B[1][3];
  B[2][3] = B[0][2];
  B[2][4] = B[1][5];
  B[2][5] = B[0][4];

  // 5. 计算刚度矩阵 Ke = B^T * D * B * area
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      amatrx[i * 6 + j] = 0.0;
      for (int k = 0; k < 3; k++) {
        for (int l = 0; l < 3; l++) {
          amatrx[i * 6 + j] += B[k][i] * D[k][l] * B[l][j] * area;
        }
      }
    }
  }

  // 6. 将右端项置零
  for (int i = 0; i < 6; i++) {
    rhs[i] = 0.0;
  }

  double strain[3];   // 存储应变
  strain[0] = du[0];  // 假设 du 存储的是位移增量
  strain[1] = du[1];
  strain[2] = du[2];

  // 应力更新，假设为简单的线性弹性模型
  double stress[3];
  stress[0] = D[0][0] * strain[0] + D[0][1] * strain[1];
  stress[1] = D[1][0] * strain[0] + D[1][1] * strain[1];
  stress[2] = D[2][2] * strain[2];

  // // 根据应力计算残差力
  // for (int i = 0; i < 6; i++) {
  //     for (int j = 0; j < 3; j++) {
  //         rhs[i] += B[j][i] * stress[j] * area;  //
  //         这里将应力作用于形函数导数，得到残差力
  //     }
  // }

  common[0] = -0.0547;
  common[1] = 0.1823;
  common[2] = 0;

  common[3] = 0;
  common[4] = 0.1823;
  common[5] = 0;
}

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

  statev[0] = common[0];
  statev[1] = common[1];
  statev[2] = common[2];
  statev[3] = common[3];
  statev[4] = common[4];
  statev[5] = common[5];
}

}  // extern "C"
