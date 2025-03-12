
# ABAQUS Subroutine Forge - User-Subroutine Development

## Overview
This is a lightweight and header-only C library designed to facilitate ABAQUS user-defined element (UEL) development. It provides fundamental operations for matrix and vector manipulations, supporting 2D and 3D tensor calculations, including stiffness matrix computation, transformation operations, and element-wise calculations.

The library includes functionalities such as matrix creation, vector creation, element-wise arithmetic operations, Voigt transformations, CPS3 nodal information handling, and inner force computations for finite element analysis in ABAQUS.

## Installation and Usage

### 1. Include the Header File
Copy `abaqus_subroutine_forge.h` into your project directory and include it in your ABAQUS UEL subroutine as follows:
```c
#include "abaqus_subroutine_forge.h"
// if you need to share information between UEL and UMAT
#include "communication.h" 
```

### 2. Compile with ABAQUS
Ensure that your ABAQUS environment supports C compilation. Use the following command to compile your UEL with this library:
```bash
abaqus job=yourJob user=uel.c
```
If additional C source files are required, include them in the command, such as:
```bash
abaqus job=yourJob user=uel.c,forge_library.c
```

### 3. Using the Library
This library provides numerous utility functions for matrix and vector manipulation. Functions can be called directly in your UEL implementation. Ensure proper memory management if additional dynamic allocations are introduced.

## Functionality Overview

### Utility Functions
- **`fatal_error(const char* fmt, ...)`**: Prints an error message and aborts execution.

### Matrix and Vector Definitions
- `Matrix2D`, `Matrix3D`, `Vector2D`, `Vector3D`, `Vector6D`: Structs representing 2D/3D matrices and vectors.
- `MatrixB36`, `MatrixB63`, `Matrix6D`: Special struct definitions for stiffness matrix and transformation matrices.

### Matrix and Vector Operations
- **Matrix Creation**
  - `create_matrix_2D()`, `create_matrix_3D()`: Create a 2D or 3D matrix.
  - `create_empty_matrix_2D()`, `create_empty_matrix_3D()`: Create an empty matrix (all zero elements).
  - `create_eye_matrix_2D()`, `create_eye_matrix_3D()`: Create an identity matrix.

- **Matrix Operations**
  - `matrix_2D_add()`, `matrix_3D_add()`: Element-wise matrix addition.
  - `matrix_2D_minus()`, `matrix_3D_minus()`: Element-wise matrix subtraction.
  - `matrix_2D_number_multiplication()`, `matrix_3D_number_multiplication()`: Scalar multiplication.
  - `matrix_2D_transpose()`, `matrix_3D_transpose()`: Matrix transpose.
  - `matrix_2D_determinant()`, `matrix_3D_determinant()`: Compute determinant.
  - `matrix_2D_inverse()`, `matrix_3D_inverse()`: Compute inverse of a matrix.

- **Vector Operations**
  - `create_vector_2D()`, `create_vector_3D()`, `create_vector_6D()`: Create vectors of given dimensions.
  - `vector_2D_add()`, `vector_3D_add()`, `vector_6D_add()`: Vector addition.
  - `vector_2D_minus()`, `vector_3D_minus()`, `vector_6D_minus()`: Vector subtraction.
  - `vector_2D_number_multiplication()`, `vector_3D_number_multiplication()`, `vector_6D_number_multiplication()`: Scalar multiplication.

- **Matrix-Vector Multiplication**
  - `matrix_2D_mul_vector_2D()`, `matrix_3D_mul_vector_3D()`: Multiply a matrix with a vector.

### Finite Element Mechanics Functions
- **CPS3 Element Nodal Handling**
  - `CPS3NodalInfo`: Struct to store nodal displacement and force information.
  - `compute_CPS3_element_square()`: Computes the element area.
  - `CPS3_nodal_disp_to_2D_F()`: Computes deformation gradient tensor F.
  - `CPS3_2D_F_to_2D_E()`: Computes strain tensor E from deformation gradient F.
  - `CPS3_2D_E_to_2D_T()`: Computes the stress tensor from the strain tensor.
  - `CPS3_compute_inner_force()`: Computes inner force for an element.

### Stiffness Matrix Computation
- **Matrix B for CPS3 Elements**
  - `CPS3_compute_matrix_B()`: Computes matrix B for CPS3 elements.
  - `CPS3_matrixB63_mul_matrix_3D()`: Matrix multiplication for B and material property matrices.
  - `CPS3_matrixB63_mul_matrix_B36()`: Computes stiffness matrix multiplication for CPS3 elements.
  - `CPS3_matrix_6D_multiplication()`: Scalar multiplication for a 6x6 stiffness matrix.

## Example Usage

```c
#include <stdio.h>
#include "abaqus_subroutine_forge.h"

int main() {
    // Creating a 2D Matrix
    Matrix2D A = create_matrix_2D(1.0, 2.0, 3.0, 4.0);
    matrix_2D_print(&A);

    // Transpose of A
    Matrix2D A_T = matrix_2D_transpose(&A);
    matrix_2D_print(&A_T);

    // Determinant of A
    double detA = matrix_2D_determinant(&A);
    printf("Determinant: %f
", detA);

    // Inverse of A (if non-singular)
    if (detA != 0) {
        Matrix2D A_inv = matrix_2D_inverse(&A);
        matrix_2D_print(&A_inv);
    }

    return 0;
}
```

## Known Issues and Limitations
- **Input validation**: The library assumes correct input dimensions; passing incorrect matrix/vector dimensions may cause undefined behavior.
- **Memory management**: The current implementation does not dynamically allocate memory. If extended for heap-based allocation, users must manage memory deallocation properly.
- **Matrix inversion limitations**: Singular matrices (determinant = 0) cannot be inverted and will trigger a fatal error.
- **Performance considerations**: This implementation is designed for small-scale matrix operations common in finite element computations. For large matrices, consider optimized libraries such as LAPACK or Eigen.

## License and Contact
- Created by **Hengyi Zhao (zhaohy19@mails.tsinghua.edu.cn)**
- Version: **1.1.0**
- Designed for ABAQUS user-defined element (UEL) subroutine development.

This document serves as a technical reference for using the ABAQUS Subroutine Forge library in UEL subroutines. If you have any questions or require modifications, please contact the author.
