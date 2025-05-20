cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
project(can_serial)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(Boost_COMPONENTS system)
find_package(Boost REQUIRED ${Boost_COMPONENTS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/CanSerialNode.cpp
  src/CanSerialCore.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${Boost_INCLUDE_DIRS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "CanSerialNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)