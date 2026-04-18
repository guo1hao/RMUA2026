cmake_minimum_required(VERSION 3.0.2)
project(rmua_flight_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  airsim_ros
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS airsim_ros geometry_msgs nav_msgs roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(basic_pwm_controller_node
  src/basic_pwm_controller_node.cpp
)

add_executable(path_mission_node
  src/path_mission_node.cpp
)

add_executable(obstacle_avoidance_node
  src/obstacle_avoidance_node.cpp
)

add_executable(official_eskf_node
  src/official_eskf.cpp
  src/official_eskf_node.cpp
)

add_executable(official_pwm_controller_node
  src/official_pwm_controller_node.cpp
)

add_dependencies(basic_pwm_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(path_mission_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(obstacle_avoidance_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(official_eskf_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(official_pwm_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(basic_pwm_controller_node
  ${catkin_LIBRARIES}
)

target_link_libraries(path_mission_node
  ${catkin_LIBRARIES}
)

target_link_libraries(obstacle_avoidance_node
  ${catkin_LIBRARIES}
)

target_link_libraries(official_eskf_node
  ${catkin_LIBRARIES}
)

target_link_libraries(official_pwm_controller_node
  ${catkin_LIBRARIES}
)

install(TARGETS
  basic_pwm_controller_node
  path_mission_node
  obstacle_avoidance_node
  official_eskf_node
  official_pwm_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
