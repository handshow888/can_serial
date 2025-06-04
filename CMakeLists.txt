cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)
project(can_serial)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # boost
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  # boost
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_serial_node 
  src/CanSerialNode.cpp
  src/CanSerialCore.cpp
)
target_link_libraries(can_serial_node
  ${catkin_LIBRARIES}
)