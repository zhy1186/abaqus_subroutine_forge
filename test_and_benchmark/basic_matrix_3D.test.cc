//
// Created by Hengyi Zhao on 2025/4/2.
//

#include <gtest/gtest.h>

#include "abaqus_subroutine_force_3D.h"

TEST(BasicMatrix3D, Vector12D) {
  Vector12D vec =
      create_vector_12D(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7, 8, 9, 10, 11, 12);

  ASSERT_NO_THROW(vector_12D_print(&vec));
  // check vector_12D_get_element
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    EXPECT_EQ(vector_12D_get_element(&vec, i), i + 1);
  }
  // check vector_12D_get_element error
  EXPECT_DEATH(vector_12D_get_element(&vec, DIMENSION_C3D4 + 1),
               "FATAL: Index out of bounds \\(index: 13\\)!");
  EXPECT_DEATH(vector_12D_get_element(&vec, -1),
               "FATAL: Index out of bounds \\(index: -1\\)!");
  // check empty vector 12D
  vec = create_empty_vector_12D();
  for (int i = 0; i < DIMENSION_C3D4; ++i) {
    EXPECT_EQ(vector_12D_get_element(&vec, i), 0);
  }
}
