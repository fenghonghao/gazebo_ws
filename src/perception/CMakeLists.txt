cmake_minimum_required(VERSION 3.02)
project(perception)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  fsd_common_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs
    fsd_common_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_processor src/lidar_processor.cpp)

target_link_libraries(lidar_processor
  ${catkin_LIBRARIES}
)

add_dependencies(lidar_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})