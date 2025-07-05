cmake_minimum_required(VERSION 3.0.2)
project(slam_gmapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  gmapping
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gmapping_node src/gmapping_node.cpp)
target_link_libraries(gmapping_node ${catkin_LIBRARIES})
