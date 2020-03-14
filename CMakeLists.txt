cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(texture_mapping)
find_package(PCL REQUIRED COMPONENTS
  common
  features
  io
  kdtree
  search
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(texture_mapping texture_mapping.cpp)
target_link_libraries(texture_mapping
  ${OPENNI_LIBRARIES}
  ${PCL_LIBRARIES}
  boost_filesystem
  boost_system
  pcl_common
  pcl_features
  pcl_io
  pcl_kdtree
  pcl_surface
  pcl_visualization
)

install(TARGETS texture_mapping DESTINATION texture_mapping)
