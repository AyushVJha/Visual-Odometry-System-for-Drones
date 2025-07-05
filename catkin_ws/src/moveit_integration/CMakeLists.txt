cmake_minimum_required(VERSION 3.0.2)
project(moveit_integration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(moveit_node src/moveit_node.cpp)
target_link_libraries(moveit_node ${catkin_LIBRARIES})
