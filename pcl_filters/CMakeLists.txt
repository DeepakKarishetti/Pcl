cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(trial)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pass_thru pass_through.cpp)
add_executable(voxel_grid voxel_grid.cpp)
add_executable(stat_removal statistical_removal.cpp)
add_executable(proj_inliers project_inliers.cpp)
add_executable(plane_model_seg plane_model_seg.cpp)
add_executable(outlier_rem outlier_removal.cpp)


target_link_libraries(pass_thru ${PCL_LIBRARIES})
target_link_libraries(voxel_grid ${PCL_LIBRARIES})
target_link_libraries(stat_removal ${PCL_LIBRARIES})
target_link_libraries(proj_inliers ${PCL_LIBRARIES})
target_link_libraries(plane_model_seg ${PCL_LIBRARIES})
target_link_libraries(outlier_rem ${PCL_LIBRARIES})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)