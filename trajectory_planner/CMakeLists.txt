cmake_minimum_required(VERSION 2.8.3)
project(trajectory_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  roscpp
  geometry_msgs
  tf
  std_msgs
  std_srvs
  message_generation
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(trajectory_planner src/trajectory_planner.cpp)
add_dependencies(trajectory_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_planner
   ${catkin_LIBRARIES}
 )

install(TARGETS trajectory_planner
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
