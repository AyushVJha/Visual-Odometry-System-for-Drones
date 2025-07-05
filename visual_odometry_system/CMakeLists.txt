cmake_minimum_required(VERSION 3.10)
project(visual_odometry_system)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
)

add_executable(visual_odometry src/visual_odometry.cpp)

target_link_libraries(visual_odometry
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  Eigen3::Eigen
)
