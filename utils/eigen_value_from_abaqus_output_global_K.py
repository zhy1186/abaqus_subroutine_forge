# eigen_value_prober.py -- python util to compute and identify negative eigen values of equivalent stiffness natrix
# created by Hengyi Zhao @ 03192025
# Note : After carefully research, choose FORMAT=COORDINATE as input matrix format
#        This format is denote by "row_index col_index entry" (3 terms)
#        Contains all non-zero entry (including symmetric terms)
# Note : How to identify displacement BC dof :
#        Abaqus use large number (1e36) to identify this dof
#        Including displacement = 0 and displacement != 0 dof

import sys
import os
import numpy as np
from enum import Enum
from numpy.typing import NDArray
from typing import List


def read_K_from_coordinate_file(file_path: str) -> NDArray:
    entries = []
    max_row = 0
    max_col = 0

    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()

            if not line:
                raise ValueError("File contains empty line.")

            parts = line.split()
            if len(parts) != 3:
                raise RuntimeError(f"Invalid line: {line}")
            row_index = int(parts[0])
            col_index = int(parts[1])
            value = float(parts[2])
            entries.append((row_index, col_index, value))
            max_row = max(max_row, row_index)
            max_col = max(max_col, col_index)
        if max_row != max_col:
            raise RuntimeError(f"Invalid file, row = {max_row}, col = {max_col}.")

    matrix = np.zeros((max_row, max_col))

    for row_index, col_index, value in entries:
        matrix[row_index - 1, col_index - 1] = value

    return matrix


def cross_out_all_displacement_BC_lines(K: NDArray, large_value_criteria=1e30) -> NDArray:
    if K.shape[0] != K.shape[1]:
        raise ValueError("passed in matrix must be square.")

    diag = np.diag(K)
    indices_to_remove = np.where(diag >= large_value_criteria)[0]

    print(f"Entry >= large_value_criteria (indicating displacement BC, 1-based): {indices_to_remove}")

    new_matrix = np.delete(K, indices_to_remove, axis=0)
    new_matrix = np.delete(new_matrix, indices_to_remove, axis=1)

    return new_matrix


class Dof(Enum):
    DofX = 1
    DofY = 2


class ExemptUnit:
    def __init__(self, nodes_index: NDArray, dof: Dof):
        self.nodes_index = nodes_index
        self.dof = dof


def cross_out_displacement_BC_exempt_force_lines(K: NDArray, exempt_units: List[ExemptUnit],
                                                 large_value_criteria=1e30) -> NDArray:
    n = K.shape[0]
    if K.shape[0] != K.shape[1]:
        raise ValueError("Input matrix K must be square.")

    exempt_indices = set()
    for unit in exempt_units:
        for node in unit.nodes_index:
            global_index = (int(node) - 1) * 2 + (unit.dof.value - 1)
            exempt_indices.add(global_index)

    indices_to_remove = []
    for i in range(n):
        if i not in exempt_indices and K[i, i] > large_value_criteria:
            indices_to_remove.append(i)

    new_K = np.delete(K, indices_to_remove, axis=0)
    new_K = np.delete(new_K, indices_to_remove, axis=1)

    return new_K


def main():
    if len(sys.argv) < 2:
        print("Usage: python eigen_value_prober.py <inp_filename>")
        sys.exit(1)

    filename = sys.argv[1]
    abs_path = os.path.abspath(filename)

    K: NDArray = read_K_from_coordinate_file(abs_path)

    print(f"最终全局刚度矩阵:\n{K}")

    print("未划掉任何行列的特征值为（升序）：")
    print(np.sort(np.linalg.eigvals(K)))

    equivalent_K: NDArray = cross_out_all_displacement_BC_lines(K)

    print("等效刚度矩阵的特征值为（升序）")
    print(np.sort(np.linalg.eigvals(equivalent_K)))


if __name__ == "__main__":
    main()
