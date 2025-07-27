cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  fsd_common_msgs
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS fsd_common_msgs geometry_msgs nav_msgs roscpp tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit_bicycle src/pure_pursuit_bicycle.cpp)
target_link_libraries(pure_pursuit_bicycle ${catkin_LIBRARIES})