cmake_minimum_required(VERSION 3.0.2)
project(amcl_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  amcl
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(amcl_node src/amcl_node.cpp)
target_link_libraries(amcl_node ${catkin_LIBRARIES})
