cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(estimating_normals)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (estimating_normals estimating_normals.cpp)
target_link_libraries (estimating_normals ${PCL_LIBRARIES})
