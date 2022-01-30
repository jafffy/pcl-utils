cmake_minimum_required(VERSION 3.16.3 FATAL_ERROR)
project(pcl-utils)

find_package(PCL 1.10.0 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_write_test pcd_write.cc)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})

add_executable(pcd_read_test pcd_read.cc)
target_link_libraries(pcd_read_test ${PCL_LIBRARIES})