import numpy as np


def tetrahedron_stiffness_matrix(coords, E=1, nu=0.3):
    # 1. 计算单元体积
    v = np.abs(np.linalg.det(np.array([
        [coords[1, 0] - coords[0, 0], coords[2, 0] - coords[0, 0], coords[3, 0] - coords[0, 0]],
        [coords[1, 1] - coords[0, 1], coords[2, 1] - coords[0, 1], coords[3, 1] - coords[0, 1]],
        [coords[1, 2] - coords[0, 2], coords[2, 2] - coords[0, 2], coords[3, 2] - coords[0, 2]]
    ]))) / 6.0

    # 2. 构造形函数系数矩阵
    # 对于线性四面体单元，形函数可以写成 N_i = a_i + b_i*x + c_i*y + d_i*z
    # 构造 4x4 矩阵 M，每行为 [1, x, y, z]，然后求逆得到各个系数
    M = np.hstack((np.ones((4, 1)), coords))
    invM = np.linalg.inv(M)
    # invM 的每一列对应一个节点形函数的系数 [a_i, b_i, c_i, d_i]^T

    # 3. 构造 B 矩阵（6 x 12）
    B = np.zeros((6, 12))
    for i in range(4):
        dN_dx = invM[1, i]
        dN_dy = invM[2, i]
        dN_dz = invM[3, i]
        # ε_xx 分量
        B[0, 3 * i] = dN_dx
        # ε_yy 分量
        B[1, 3 * i + 1] = dN_dy
        # ε_zz 分量
        B[2, 3 * i + 2] = dN_dz
        # γ_xy 分量： dU_y/dx + dU_x/dy
        B[3, 3 * i] = dN_dy
        B[3, 3 * i + 1] = dN_dx
        # γ_yz 分量： dU_z/dy + dU_y/dz
        B[4, 3 * i + 1] = dN_dz
        B[4, 3 * i + 2] = dN_dy
        # γ_zx 分量： dU_x/dz + dU_z/dx
        B[5, 3 * i] = dN_dz
        B[5, 3 * i + 2] = dN_dx

    # 4. 构造材料刚度矩阵 D (3D 各向同性线弹性)
    coef = E / ((1 + nu) * (1 - 2 * nu))
    D = coef * np.array([
        [1 - nu, nu, nu, 0, 0, 0],
        [nu, 1 - nu, nu, 0, 0, 0],
        [nu, nu, 1 - nu, 0, 0, 0],
        [0, 0, 0, (1 - 2 * nu) / 2, 0, 0],
        [0, 0, 0, 0, (1 - 2 * nu) / 2, 0],
        [0, 0, 0, 0, 0, (1 - 2 * nu) / 2]
    ])

    # 5. 计算单元刚度矩阵 Ke = B^T * D * B * V
    Ke = B.T @ D @ B * v

    return Ke


def compute_internal_force(K, u):
    f = K @ u
    return f


if __name__ == "__main__":
    # Job : AbaqusForce_InitialTests_IsotropicLinear_C3D4_ConcentrateForceLoad
    # coordinates information:
    #   1,         100.,         100.,         100.
    #   2,         100.,           0.,         100.
    #   3,         100.,           0.,           0.
    #   4,         100.,         100.,           0.
    #   5,           0.,           0.,         100.
    #   6,           0.,           0.,           0.
    #   7,           0.,         100.,           0.
    #   8,           0.,         100.,         100.
    #   9,   48.8728256,   50.2414932,   49.9863815
    # element connectivity
    # 1, 3, 4, 9, 1
    # 2, 2, 1, 9, 8
    # 3, 2, 8, 9, 5
    # 4, 2, 5, 9, 6
    # 5, 1, 9, 3, 2
    # 6, 5, 9, 6, 7
    # 7, 8, 9, 5, 7
    # 8, 1, 8, 4, 9
    # 9, 3, 7, 9, 4
    # 10, 6, 9, 3, 7
    # 11, 7, 4, 8, 9
    # 12, 2, 6, 9, 3
    # displacement info:
    #       Part Instance     Node ID  Attached elements           U, U1           U, U2           U, U3
    # -----------------------------------------------------------------------------------------------
    #     PART-1-1           1               1    -65.4589E-03,    -60.8189E-03,     240.597E-03,
    #     PART-1-1           2               2    -50.3685E-03,    -14.1452E-03,     179.899E-03,
    #     PART-1-1           3               9    -43.7817E-03,     14.7087E-39,      6.0266E-36,
    #     PART-1-1           4               1    -45.2277E-03,    -73.5207E-03,     1.98914E-36,
    #     PART-1-1           5               7     21.4124E-03,    -12.3483E-03,     241.398E-03,
    #     PART-1-1           6               4     116.756E-39,    -136.818E-39,     1.95039E-36,
    #     PART-1-1           7               7    -1.21027E-03,    -73.5836E-03,     6.03637E-36,
    #     PART-1-1           8               8     5.85927E-03,    -59.7669E-03,     180.451E-03,
    #     PART-1-1           9               8     -0.0216563,     -0.036995,        0.0954731,
    # For element 1
    coords1 = np.array([
        [100., 0., 0.],
        [100., 100., 0.],
        [48.8728256, 50.2414932, 49.9863815],
        [100., 100., 100.]
    ])
    u1 = np.array([
        -43.7817E-03, 14.7087E-39, 6.0266E-36,
        -45.2277E-03, -73.5207E-03, 1.98914E-36,
        -0.0216563, -0.036995, 0.0954731,
        -65.4589E-03, -60.8189E-03, 240.597E-03
    ])
    K1 = tetrahedron_stiffness_matrix(coords1)
    print("单元1刚度矩阵 Ke:")
    # print(K1)
    f1 = compute_internal_force(K1, u1)
    print("单元1内部力向量 f = K * u:")
    print(f1)

    # For element 2
    coords2 = np.array([
        [100., 0., 100.],
        [100., 100., 100.],
        [48.8728256, 50.2414932, 49.9863815],
        [0., 100., 100.]
    ])
    u2 = np.array([
        -50.3685E-03, -14.1452E-03, 179.899E-03,
        -65.4589E-03, -60.8189E-03, 240.597E-03,
        -0.0216563, -0.036995, 0.0954731,
        5.85927E-03, -59.7669E-03, 180.451E-03
    ])
    K2 = tetrahedron_stiffness_matrix(coords2)
    print("单元2刚度矩阵 Ke:")
    # print(K2)
    f2 = compute_internal_force(K2, u2)
    print("单元2内部力向量 f = K * u:")
    print(f2)
    # For element 3
    coords3 = np.array([
        [100., 0., 100.],
        [0., 100., 100.],
        [48.8728256, 50.2414932, 49.9863815],
        [0., 0., 100.]
    ])
    u3 = np.array([
        -50.3685E-03, -14.1452E-03, 179.899E-03,
        5.85927E-03, -59.7669E-03, 180.451E-03,
        -0.0216563, -0.036995, 0.0954731,
        21.4124E-03, -12.3483E-03, 241.398E-03
    ])
    K3 = tetrahedron_stiffness_matrix(coords3)
    print("单元3刚度矩阵 Ke:")
    # print(K3)
    f3 = compute_internal_force(K3, u3)
    print("单元3内部力向量 f = K * u:")
    print(f3)
    # For element 4
    coords4 = np.array([
        [100., 0., 100.],
        [0., 0., 100.],
        [48.8728256, 50.2414932, 49.9863815],
        [0., 0., 0.]
    ])
    u4 = np.array([
        -50.3685E-03, -14.1452E-03, 179.899E-03,
        21.4124E-03, -12.3483E-03, 241.398E-03,
        -0.0216563, -0.036995, 0.0954731,
        116.756E-39, -136.818E-39, 1.95039E-36
    ])
    K4 = tetrahedron_stiffness_matrix(coords4)
    print("单元4刚度矩阵 Ke:")
    # print(K4)
    f4 = compute_internal_force(K4, u4)
    print("单元4内部力向量 f = K * u:")
    print(f4)
    # For element 5
    coords5 = np.array([
        [100.0, 100.0, 100.0],
        [48.878256, 50.2414932, 48.9863815],
        [100.0, 0.0, 0.0],
        [100.0, 0.0, 100.0]
    ])
    u5 = np.array(
        [-65.4589E-03, -60.8189E-03, 240.597E-03,
         -0.0216563, -0.036995, 0.0954731,
         -43.7817E-03, 14.7087E-39, 6.0266E-36,
         - 50.3685E-03, -14.1452E-03, 179.899E-03])
    K5 = tetrahedron_stiffness_matrix(coords5)
    print("单元5刚度矩阵 Ke:")
    # print(K5)
    f5 = compute_internal_force(K5, u5)
    print("单元5内部力向量 f = K * u:")
    print(f5)

    # For element 6
    coords6 = np.array([
        [0., 0., 100.],
        [48.8728256, 50.2414932, 49.9863815],
        [0., 0., 0.],
        [0., 100., 0.]
    ])
    u6 = np.array([
        21.4124E-03, -12.3483E-03, 241.398E-03,
        -0.0216563, -0.036995, 0.0954731,
        116.756E-39, -136.818E-39, 1.95039E-36,
        -1.21027E-03, -73.5836E-03, 6.03637E-36,
    ])
    K6 = tetrahedron_stiffness_matrix(coords6)
    print("单元6刚度矩阵 Ke:")
    # print(K6)
    f6 = compute_internal_force(K6, u6)
    print("单元6内部力向量 f = K * u:")
    print(f6)

    # For element 7
    coords7 = np.array([
        [0., 100., 100.],
        [48.8728256, 50.2414932, 49.9863815],
        [0., 0., 100.],
        [0., 100., 0.]
    ])
    u7 = np.array([
        5.85927E-03, -59.7669E-03, 180.451E-03,
        -0.0216563, -0.036995, 0.0954731,
        21.4124E-03, -12.3483E-03, 241.398E-03,
        -1.21027E-03, -73.5836E-03, 6.03637E-36
    ])
    K7 = tetrahedron_stiffness_matrix(coords7)
    print("单元7刚度矩阵 Ke:")
    # print(K7)
    f7 = compute_internal_force(K7, u7)
    print("单元7内部力向量 f = K * u:")
    print(f7)

    # For element 8

    coords8 = np.array([
        [100., 100., 100.],
        [0., 100., 100.],
        [100., 100., 0.],
        [48.8728256, 50.2414932, 49.9863815]
    ])
    u8 = np.array([
        -65.4589E-03, -60.8189E-03, 240.597E-03,
        5.85927E-03, -59.7669E-03, 180.451E-03,
        -45.2277E-03, -73.5207E-03, 1.98914E-36,
        -0.0216563, -0.036995, 0.0954731
    ])
    K8 = tetrahedron_stiffness_matrix(coords8)
    print("单元8刚度矩阵 Ke:")
    # print(K8)
    f8 = compute_internal_force(K8, u8)
    print("单元8内部力向量 f = K * u:")
    print(f8)

    # For element 11
    coords11 = np.array([
        [0., 100., 0.],
        [100., 100., 0.],
        [0., 100., 100.],
        [48.8728256, 50.2414932, 49.9863815]
    ])
    u11 = np.array([
        -1.21027E-03, -73.5836E-03, 6.03637E-36,
        -45.2277E-03, -73.5207E-03, 1.98914E-36,
        5.85927E-03, -59.7669E-03, 180.451E-03,
        -0.0216563, -0.036995, 0.0954731
    ])
    K11 = tetrahedron_stiffness_matrix(coords11)
    print("单元11刚度矩阵 Ke:")
    # print(K11)
    f11 = compute_internal_force(K11, u11)
    print("单元11内部力向量 f = K * u:")
    print(f11)

    # For element 12
    coords12 = np.array([
        [100., 0., 100.],
        [0., 0., 0.],
        [48.8728256, 50.2414932, 49.9863815],
        [100., 0., 0.]
    ])
    u12 = np.array([
        -50.3685E-03, -14.1452E-03, 179.899E-03,
        116.756E-39, -136.818E-39, 1.95039E-36,
        -0.0216563, -0.036995, 0.0954731,
        -43.7817E-03, 14.7087E-39, 6.0266E-36
    ])
    K12 = tetrahedron_stiffness_matrix(coords12)
    print("单元12刚度矩阵 Ke:")
    # print(K12)
    f12 = compute_internal_force(K12, u12)
    print("单元12内部力向量 f = K * u:")
    print(f12)

    # Research Node 8 z : [ele2::4th, ele3::2nd, ele7::2nd, ele8::2nd, ele11::3rd]
    f8z = f2[11] + f3[5] + f7[5] + f8[5] + f11[8]
    print(f"f8z = {f8z}")

    # Research Node 1 z : [ele1::4th, ele2::2nd, ele5::1st, ele8::1st]
    f1z = f1[11] + f2[5] + f5[2] + f8[2]
    print(f"f1z = {f1z}")
