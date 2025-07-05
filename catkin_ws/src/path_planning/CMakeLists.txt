cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar_planner src/astar_planner.cpp)
target_link_libraries(astar_planner ${catkin_LIBRARIES})

add_executable(dwa_planner src/dwa_planner.cpp)
target_link_libraries(dwa_planner ${catkin_LIBRARIES})
