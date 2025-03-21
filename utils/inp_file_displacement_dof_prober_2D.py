import os.path
import re
import sys
import numpy as np


# 提取节点集（包括普通节点集和generate模式下的节点集）
def extract_node_sets_from_inp(inp_file_path):
    node_sets = {}
    capturing = False  # 是否正在捕获节点数据
    generate_mode = False  # 是否采用generate模式
    current_set_name = None

    with open(inp_file_path, 'r') as file:
        lines = file.readlines()

    for line in lines:
        # 检查是否为*Nset行（不区分大小写）
        if line.strip().upper().startswith('*NSET'):
            capturing = True
            generate_mode = False  # 默认不是generate模式
            # 提取节点集名称（nset关键字后面的名称）
            match = re.search(r'(?:nset|nest)=([^\s,]+)', line, re.IGNORECASE)
            if match:
                current_set_name = match.group(1)
                node_sets[current_set_name] = []
            # 如果这一行包含generate关键字，则设置generate模式
            if 'generate' in line.lower():
                generate_mode = True
            continue

        # 如果正在捕获节点数据
        if capturing:
            # 如果遇到以*开头的行，说明节点集数据结束
            if line.strip().startswith('*'):
                capturing = False
                continue

            # 如果是generate模式，只读取一行内容并解析成起始节点、终止节点和步长
            if generate_mode:
                numbers = [int(num) for num in re.findall(r'\d+', line)]
                if len(numbers) == 3:
                    start, end, step = numbers
                    # range上界是不包含的，所以用end+1来包含终止节点
                    node_sets[current_set_name].extend(range(start, end + 1, step))
                # 处理完generate后退出捕获状态
                generate_mode = False
                capturing = False
            else:
                # 普通情况，可能跨行写入，捕获每一行中的所有数字
                numbers = [int(num) for num in re.findall(r'\d+', line)]
                if numbers:
                    node_sets[current_set_name].extend(numbers)
    return node_sets


# 提取边界条件，并计算每个节点的自由度索引（dof_idx）
def extract_boundary_conditions_from_inp(inp_file_path):
    node_sets = extract_node_sets_from_inp(inp_file_path)  # 提取所有节点集
    boundary_conditions = []
    capture_boundary = False

    with open(inp_file_path, 'r') as file:
        lines = file.readlines()

    for line in lines:
        # 检测是否为*Boundary行（不区分大小写）
        if line.strip().upper().startswith('*BOUNDARY'):
            capture_boundary = True
            continue  # 跳过*Boundary行本身
        if capture_boundary:
            # 遇到空行或者遇到新的以*开头的行则停止捕获当前边界数据
            if line.strip() == '' or line.strip().startswith('*'):
                capture_boundary = False
                continue
            parts = re.split(r',\s*', line.strip())
            if len(parts) in [3, 4]:
                node_or_set, dof_start, dof_end = parts[:3]
                displacement = 0  # 默认固定位移为0
                if len(parts) == 4:
                    displacement = parts[3]

                # 判断节点是单个节点还是节点集名称
                nodes = []
                if node_or_set in node_sets:
                    nodes = node_sets[node_or_set]
                else:
                    try:
                        nodes = [int(node_or_set)]
                    except ValueError:
                        continue  # 如果既不是数字也不在节点集内，则跳过

                # DOF范围（1代表X方向，2代表Y方向）
                dofs = list(range(int(dof_start), int(dof_end) + 1))
                for node in nodes:
                    for dof in dofs:
                        dof_idx = 2 * node - 1 if dof == 1 else 2 * node
                        boundary_conditions.append({
                            'Node': node,
                            'DOF': dof,
                            'Displacement': float(displacement) if displacement != '0' else 0,
                            'dof_idx': dof_idx
                        })

    return boundary_conditions


def main():
    if len(sys.argv) < 2:
        print("Usage: python eigen_value_prober.py <mtx_filename>")
        sys.exit(1)

    filename = sys.argv[1]
    abs_path = os.path.abspath(filename)

    boundary_conditions = extract_boundary_conditions_from_inp(abs_path)
    fixed_dof = []
    displacement_bc_dof = []
    for boundary_condition in boundary_conditions:
        displacement = boundary_condition['Displacement']
        dof_idx = boundary_condition['dof_idx']
        displacement_bc_dof.append(dof_idx)
        if displacement == 0:
            fixed_dof.append(dof_idx)
    print("请注意，虽然已经经过了集成测试，但由于投用时间较短，以下结果应当经过手动检查：")
    print(f"所有位移边界自由度(1-based)为\t{np.array2string(np.sort(displacement_bc_dof), separator=',')}")
    print(f"固定位移自由度(1-based)为\t{np.array2string(np.sort(fixed_dof), separator=',')}")


if __name__ == "__main__":
    main()
