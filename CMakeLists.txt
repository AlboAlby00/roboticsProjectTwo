cmake_minimum_required(VERSION 3.0.2)
project(robotics_project_two)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  rospy
  tf2
  tf2_ros
  dynamic_reconfigure
  std_msgs
  message_generation
)




add_service_files(
  FILES
  CreateMapAndTrajectoryImage.srv
  ResetTrajectoryToDraw.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

catkin_install_python(PROGRAMS
  scripts/trajectory_drawer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf src/odom_tf.cpp)
add_dependencies(odom_tf ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_tf ${catkin_LIBRARIES})



