cmake_minimum_required(VERSION 2.8.3)
project(fanuc_m16ib_moveit_plugins)

#enable c++11
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_EXTENSIONS FALSE)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

find_package(LAPACK REQUIRED)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

add_subdirectory(m16ib20_kinematics)

install(FILES readme.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
