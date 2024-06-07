cmake_minimum_required(VERSION 3.0.2)
project(tm12_bottle_sorting)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  sensor_msgs
  std_msgs
  tm_msgs
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Declare a catkin package
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES tm12_bottle_sorting
  CATKIN_DEPENDS cv_bridge image_transport rospy sensor_msgs std_msgs tm_msgs roscpp
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable for the move_robot node
add_executable(move_robot_node src/move_robot.cpp)

## Add cmake target dependencies of the executable
add_dependencies(move_robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(move_robot_node
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for the node_1 node
add_executable(node_1 src/node_1.cpp)

## Add cmake target dependencies of the executable
add_dependencies(node_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(node_1
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for the node_3 (set_positions_with_feedback) node
add_executable(node_3 src/node_3.cpp)

## Add cmake target dependencies of the executable
add_dependencies(node_3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(node_3
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for the gripper_ON node
add_executable(gripper_ON_node src/gripper_ON.cpp)

## Add cmake target dependencies of the executable
add_dependencies(gripper_ON_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(gripper_ON_node
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for the node_6 node
add_executable(node_6 src/node_6.cpp)

## Add cmake target dependencies of the executable
add_dependencies(node_6 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(node_6
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for the node_7 node
add_executable(node_7 src/node_7.cpp)

## Add cmake target dependencies of the executable
add_dependencies(node_7 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(node_7
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for the node_8 (turn off gripper) node
add_executable(node_8 src/node_8.cpp)

## Add cmake target dependencies of the executable
add_dependencies(node_8 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(node_8
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS move_robot_node node_1 node_3 gripper_ON_node node_6 node_7 node_8
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark python scripts for installation
catkin_install_python(PROGRAMS scripts/bottle_position.py scripts/brand_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tm12_bottle_sorting.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
