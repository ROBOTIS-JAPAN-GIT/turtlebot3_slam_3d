cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_slam_3d)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation)

add_service_files(FILES GetObjectLocation.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

include_directories()
