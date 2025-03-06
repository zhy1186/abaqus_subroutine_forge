include_directories(${CMAKE_SOURCE_DIR})

enable_testing()

add_executable(basic_matrix basic_matrix.test.cc)
target_link_libraries(basic_matrix GTest::gtest_main)

add_executable(basic_math basic_math.test.cc)
target_link_libraries(basic_math GTest::gtest_main Eigen3::Eigen)

add_executable(mechanics mechanics.test.cc)
target_link_libraries(mechanics GTest::gtest_main Eigen3::Eigen)

add_executable(mechanical_matrix mechanical_matrix.test.cc)
target_link_libraries(mechanical_matrix GTest::gtest_main Eigen3::Eigen)

include(GoogleTest)
gtest_discover_tests(basic_matrix)
gtest_discover_tests(basic_math)
gtest_discover_tests(mechanics)
gtest_discover_tests(mechanical_matrix)
