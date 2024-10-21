cmake_minimum_required(VERSION 3.5)
project(ros2_icp)

## Compile as C++14
 add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
 
###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/icp/utils.cpp
  src/icp/solver.cpp
  src/ros/conversion.cpp
)
 
add_executable(icp_node
  src/nodes/icp_node.cpp)

target_link_libraries(
  icp_node ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)
ament_package()