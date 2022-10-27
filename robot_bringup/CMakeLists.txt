cmake_minimum_required(VERSION 3.0.2)
project(robot_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  amcl
  control_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  robot_state_publisher
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(${PROJECT_NAME}_Cmd_Vel_to_Arduino src/cmd_vel_mux.cpp)
add_executable(${PROJECT_NAME}_Differential_TF src/differential.cpp)



set_target_properties(${PROJECT_NAME}_Cmd_Vel_to_Arduino PROPERTIES OUTPUT_NAME Cmd_Vel_to_Arduino PREFIX "")
set_target_properties(${PROJECT_NAME}_Differential_TF PROPERTIES OUTPUT_NAME Differential_TF PREFIX "")




add_dependencies(${PROJECT_NAME}_Cmd_Vel_to_Arduino ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_Differential_TF ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against


target_link_libraries(${PROJECT_NAME}_Cmd_Vel_to_Arduino
   ${catkin_LIBRARIES}
 )

 target_link_libraries(${PROJECT_NAME}_Differential_TF
 ${catkin_LIBRARIES}
)
