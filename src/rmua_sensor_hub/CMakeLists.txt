cmake_minimum_required(VERSION 3.0.2)
project(rmua_sensor_hub)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rmua_msgs
  std_msgs
  topic_tools
)

catkin_package(
  CATKIN_DEPENDS roscpp rmua_msgs std_msgs topic_tools
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensor_hub_node
  src/sensor_hub_node.cpp
)

target_link_libraries(sensor_hub_node
  ${catkin_LIBRARIES}
)

install(TARGETS sensor_hub_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
