cmake_minimum_required(VERSION 3.0.2)
project(second_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry_to_tf src/odometry_to_tf.cpp)
target_link_libraries(odometry_to_tf ${catkin_LIBRARIES})