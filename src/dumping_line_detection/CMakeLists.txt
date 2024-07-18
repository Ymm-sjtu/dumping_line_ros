cmake_minimum_required(VERSION 3.0.2)
project(dumping_line_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  visualization_msgs
)

## Eigen3
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dumping_line_detection
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs tf visualization_msgs
)

## 指定头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 添加可执行文件
add_executable(${PROJECT_NAME}_node src/main.cpp src/convolution_core.cpp src/b_spline.cpp src/TSPSolver.cpp)

## 添加依赖库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
