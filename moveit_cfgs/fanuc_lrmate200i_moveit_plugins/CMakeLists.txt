cmake_minimum_required(VERSION 3.0.2)
project(fanuc_lrmate200i_moveit_plugins)

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

add_subdirectory(lrmate200i_kinematics)

install(FILES readme.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
