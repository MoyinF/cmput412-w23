cmake_minimum_required(VERSION 2.8.3)
project(digit_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  duckietown_msgs
  roscpp
  rospy
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  DetectionService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  duckietown_msgs
)

add_dependencies(${catkin_EXPORTED_TARGETS})

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

