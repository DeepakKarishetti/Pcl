cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

project(pcd_read)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_read pcd_rotate.cc)
add_executable (bag_csv extract_data.cpp)

target_link_libraries (pcd_read ${PCL_LIBRARIES})
target_link_libraries (bag_csv ${PCL_LIBRARIES})
