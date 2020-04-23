cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)

project(pcl_reg)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(icp icp.cc)
add_executable(pairwise_icp pairwise_icp.cc)
add_executable(narf_features narf_feat.cpp)

target_link_libraries( icp ${PCL_LIBRARIES})
target_link_libraries( pairwise_icp ${PCL_LIBRARIES})
target_link_libraries( narf_features ${PCL_LIBRARIES})

set (CMAKE_EXPORT_COMPILE_COMMANDS ON) 