import copy

import numpy as np
import re
import os
import sys


def reorder_matrix_to_global_order(nodes, matrix):
    original_dofs = [dof for node in nodes for dof in (node * 2 - 1, node * 2)]
    sorted_nodes = sorted(nodes)
    sorted_dofs = [dof for node in sorted_nodes for dof in (node * 2 - 1, node * 2)]
    index_map = [original_dofs.index(dof) for dof in sorted_dofs]
    reordered_matrix = matrix[np.ix_(index_map, index_map)]
    return sorted_nodes, sorted_dofs, reordered_matrix


def read_stiffness_matrices(filepath: str):
    matrices = {}
    with open(filepath, 'r') as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("** ELEMENT NUMBER"):
            match = re.search(r'ELEMENT NUMBER\s+(\d+)', line, re.IGNORECASE)
            element_id = int(match.group(1)) if match else None

            # 关键修复点
            if element_id is None:
                i += 1
                continue

            nodes = []
            while i < len(lines) and not lines[i].strip().startswith("*MATRIX"):
                if lines[i].strip().startswith("** ELEMENT NODES"):
                    i += 1
                    nodes_line = lines[i].strip().lstrip("*").strip()
                    nodes = [int(num) for num in re.findall(r'\d+', nodes_line)]
                i += 1

            dof_list = [node * 2 - 1 for node in nodes] + [node * 2 for node in nodes]
            n = len(dof_list)

            if i >= len(lines) or not lines[i].strip().upper().startswith("*MATRIX"):
                raise ValueError(f"单元 {element_id} 找不到矩阵数据标记 '*MATRIX'")

            i += 1
            matrix_data = []
            expected_size = n * (n + 1) // 2

            while len(matrix_data) < expected_size:
                if i >= len(lines):
                    raise ValueError(
                        f"单元 {element_id} 的矩阵数据不足，预期 {expected_size} 个元素，但读取到 {len(matrix_data)}")
                data_line = lines[i].strip()
                if data_line == "" or data_line.startswith("**"):
                    i += 1
                    continue
                data_line = data_line.replace(",", " ")
                row_numbers = [float(num) for num in data_line.split()]
                matrix_data.extend(row_numbers)
                i += 1

            if len(matrix_data) != expected_size:
                raise ValueError(
                    f"单元 {element_id} 数据错误: 期望 {expected_size} 个元素, 实际读取 {len(matrix_data)}")

            matrix = np.zeros((n, n))
            idx = 0
            for row in range(n):
                for col in range(row + 1):
                    matrix[row, col] = matrix_data[idx]
                    matrix[col, row] = matrix_data[idx]
                    idx += 1

            nodes, dof_list, matrix = reorder_matrix_to_global_order(nodes, matrix)

            matrices[element_id] = {
                "nodes": nodes,
                "dofs": dof_list,
                "matrix": matrix
            }
        else:
            i += 1

    return matrices


def assemble_global_stiffness_matrix(stiffness_info):
    max_dof = max(max(elem["dofs"]) for elem in stiffness_info.values())
    K_global = np.zeros((max_dof, max_dof))

    for elem_data in stiffness_info.values():
        dofs = elem_data["dofs"]
        k_local = elem_data["matrix"]
        n = len(dofs)
        for i in range(n):
            for j in range(n):
                global_i = dofs[i] - 1
                global_j = dofs[j] - 1
                K_global[global_i, global_j] += k_local[i, j]

    return K_global


def main(abs_path, cross_out_lines):
    stiffness_info = read_stiffness_matrices(abs_path)
    print("读入的单元刚度矩阵信息（可用于校验）：")
    for element_id, info in stiffness_info.items():
        print(20 * '-' + f" element id = {element_id} " + 20 * '-')
        print(f"dof (1 based) is {info['dofs']}")
        print(f"element stiffness matrix = \n{info['matrix']}")
        print(60 * '=')
    global_K = assemble_global_stiffness_matrix(stiffness_info)

    print("最终全局刚度矩阵:")
    print(global_K)

    print("未划掉任何行列的特征值为（升序）：")
    print(np.sort(np.linalg.eigvals(global_K)))

    cross_out_indexes = [x - 1 for x in cross_out_lines]
    new_matrix = np.delete(copy.deepcopy(global_K), cross_out_indexes, axis=0)
    new_matrix = np.delete(new_matrix, cross_out_indexes, axis=1)

    print("等效刚度矩阵的特征值为（升序）")
    print(np.sort(np.linalg.eigvals(new_matrix)))


if __name__ == "__main__":
    # Configurations need to config
    file_name = "file_name.inp"
    cross_out_lines_1_based = [2, 3, 5, 6, 9, 10, 11, 12, 14, 15, 17, 18]  # (1-based)

    abs_path = os.path.abspath(file_name)
    main(abs_path, cross_out_lines_1_based)
