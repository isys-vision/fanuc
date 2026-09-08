cmake_minimum_required(VERSION 3.0.2)

project(fanuc_driver)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# let industrial_robot_client pull in all other dependencies
find_package(catkin REQUIRED COMPONENTS industrial_robot_client simple_message)

catkin_package(
  CATKIN_DEPENDS industrial_robot_client
)

##Note: the following block becomes superflous when we use an updated version of industrial-core

# these nodes run on the host computer, so use Linux sockets for
# simple_message sockets
add_definitions(-DLINUXSOCKETS=1)

include_directories(
  include
  include/fanuc_driver
  include/fanuc_driver/simple_message
  include/fanuc_driver/simple_message/messages
  include/fanuc_driver/simple_message/classes
  ${catkin_INCLUDE_DIRS}
)

set(MSG_SRC_FILES
  src/simple_message/classes/mikado_connection_info.cpp
  src/simple_message/classes/mikado_dynamic_joints_traj_pt.cpp
  src/simple_message/classes/mikado_dynamic_joints.cpp
  src/simple_message/messages/mikado_dynamic_joints_traj_pt_message.cpp
  src/simple_message/messages/mikado_dynamic_joints_message.cpp
  src/simple_message/messages/mikado_connection_info_message.cpp
)

add_library(fanuc_interfaces
    src/fanuc_joint_trajectory_interface.cpp
    src/fanuc_joint_trajectory_downloader.cpp
    src/fanuc_simple_comms_fault_handler.cpp
    src/fanuc_message_manager.cpp
    src/fanuc_robot_state_interface.cpp
)

add_library(fanuc_mikado_simple_message ${MSG_SRC_FILES})
target_link_libraries(fanuc_mikado_simple_message
  simple_message)


# non-swapping Fanuc specific robot state node
add_executable(${PROJECT_NAME}_robot_state
  src/fanuc_robot_state_node.cpp
  src/fanuc_utils.cpp
  src/dynamic_joints_relay_handler.cpp
  src/fanuc_robot_state_interface.cpp
  src/fanuc_message_manager.cpp
  src/fanuc_simple_comms_fault_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}_robot_state
  fanuc_mikado_simple_message
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_robot_state
  PROPERTIES OUTPUT_NAME robot_state
  PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_robot_state
  PRIVATE
  ${industrial_robot_client_DEFINITIONS})


# byte-swapping Fanuc specific robot state node
add_executable(${PROJECT_NAME}_robot_state_bswap
  src/fanuc_robot_state_node.cpp
  src/fanuc_utils.cpp
  src/dynamic_joints_relay_handler.cpp
  src/fanuc_robot_state_interface.cpp
  src/fanuc_message_manager.cpp
  src/fanuc_simple_comms_fault_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}_robot_state_bswap
  fanuc_mikado_simple_message
  industrial_robot_client_bswap
  simple_message_bswap
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_robot_state_bswap
  PROPERTIES OUTPUT_NAME robot_state_bswap
  PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_robot_state_bswap
  PRIVATE
  ${industrial_robot_client_bswap_DEFINITIONS})


# non-swapping Fanuc specific motion streaming interface
add_executable(${PROJECT_NAME}_motion_streaming_interface
  src/fanuc_joint_streamer_node.cpp
  src/fanuc_utils.cpp)
target_link_libraries(${PROJECT_NAME}_motion_streaming_interface
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_streaming_interface
  PROPERTIES OUTPUT_NAME motion_streaming_interface
  PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_motion_streaming_interface
  PRIVATE
  ${industrial_robot_client_DEFINITIONS})

# non-swapping Fanuc specific motion streaming interface
add_executable(${PROJECT_NAME}_motion_downloading_interface
  src/fanuc_joint_downloader_node.cpp
  src/fanuc_utils.cpp)
target_link_libraries(${PROJECT_NAME}_motion_downloading_interface
  fanuc_interfaces
  fanuc_mikado_simple_message
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_downloading_interface
  PROPERTIES OUTPUT_NAME motion_downloading_interface
  PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_motion_downloading_interface
  PRIVATE
  ${industrial_robot_client_DEFINITIONS})


# byte-swapping Fanuc specific motion streaming interface
add_executable(${PROJECT_NAME}_motion_streaming_interface_bswap
  src/fanuc_joint_streamer_node.cpp
  src/fanuc_utils.cpp)
target_link_libraries(${PROJECT_NAME}_motion_streaming_interface_bswap
  industrial_robot_client_bswap
  simple_message_bswap
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_streaming_interface_bswap
  PROPERTIES OUTPUT_NAME motion_streaming_interface_bswap
  PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_motion_streaming_interface_bswap
  PRIVATE
  ${industrial_robot_client_bswap_DEFINITIONS})

  # byte-swapping Fanuc specific motion streaming interface
add_executable(${PROJECT_NAME}_motion_downloading_interface_bswap
src/fanuc_joint_downloader_node.cpp
src/fanuc_utils.cpp)
target_link_libraries(${PROJECT_NAME}_motion_downloading_interface_bswap
  fanuc_interfaces
  fanuc_mikado_simple_message
  industrial_robot_client_bswap
  simple_message_bswap
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_downloading_interface_bswap
PROPERTIES OUTPUT_NAME motion_downloading_interface_bswap
PREFIX "")
target_compile_definitions(
${PROJECT_NAME}_motion_downloading_interface_bswap
PRIVATE
${industrial_robot_client_bswap_DEFINITIONS})


if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  set(rl_test_deps
    ${PROJECT_NAME}_robot_state
    ${PROJECT_NAME}_robot_state_bswap
    ${PROJECT_NAME}_motion_streaming_interface
    ${PROJECT_NAME}_motion_streaming_interface_bswap
    ${PROJECT_NAME}_motion_downloading_interface
    ${PROJECT_NAME}_motion_downloading_interface_bswap)

  if (TARGET joint_trajectory_action)
    # not nice, but industrial_robot_client doesn't prefix its targets
    list(APPEND rl_test_deps joint_trajectory_action)
  endif()

  roslaunch_add_file_check(tests/roslaunch_test.xml
    DEPENDENCIES ${rl_test_deps})
endif()


install(
  TARGETS
    ${PROJECT_NAME}_motion_streaming_interface
    ${PROJECT_NAME}_motion_streaming_interface_bswap
    ${PROJECT_NAME}_motion_downloading_interface
    ${PROJECT_NAME}_motion_downloading_interface_bswap

    ${PROJECT_NAME}_robot_state
    ${PROJECT_NAME}_robot_state_bswap

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    fanuc_mikado_simple_message
    fanuc_interfaces
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY karel launch tpe
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES readme.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
