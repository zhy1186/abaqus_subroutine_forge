cmake_minimum_required(VERSION 3.29)
project(abaqus_subroutine_forge)

set(CMAKE_CXX_STANDARD 20)

find_package(GTest REQUIRED)
find_package(Eigen3)

enable_testing()

add_subdirectory(test_and_benchmark)
add_subdirectory(uels)
