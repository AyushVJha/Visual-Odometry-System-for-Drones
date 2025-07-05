cmake_minimum_required(VERSION 3.0.2)
project(custom_nodes)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sensor_processing_node src/sensor_processing.cpp)
target_link_libraries(sensor_processing_node ${catkin_LIBRARIES})
