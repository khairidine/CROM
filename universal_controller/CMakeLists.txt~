cmake_minimum_required(VERSION 2.8.3)
project(universal_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
   FILES
   PoseRPY.msg
   RPY.msg
   MultiPriorityTask.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    control_toolbox
    realtime_tools
    urdf
    forward_command_controller
    kdl_parser
    message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
	
 

  src/dynamic_sliding_mode_controller.cpp
  src/joint_impedance_controller_.cpp
 
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
install(FILES universal_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
