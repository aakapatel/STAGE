cmake_minimum_required(VERSION 3.0.2)
project(stage_planner)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(octomap_DIR "/opt/ros/noetic/share/octomap")
##set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)

find_package (PCL 1.10 REQUIRED)

set(PCL_INCLUDE_DIRS /usr/include/pcl-1.10 /usr/include/eigen3/)
# set(CHOLMOD_LIBRARY /usr/lib/suitesparse)

include_directories(${PCL_INCLUDE_DIRS})
##target_link_libraries( ... ${PCL_LIBRARIES})


include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message("OpenMP enabled")
endif()
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ufomap_msgs
  ufomap_ros
  tf2_ros
  roslib
  geometry_msgs
  octomap_msgs
  octomap_ros
  mav_msgs
  mav_planning_msgs
  std_srvs
  roscpp
  pcl_ros
  tf_conversions
  std_msgs
  nav_msgs
  move_base_msgs
  actionlib
  message_generation
  ufomap_mapping
)
find_package(ufomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost 1.71.0 REQUIRED)

include_directories( ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  Frontiers.msg
  Frontier.msg
  path.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
 CATKIN_DEPENDS
 message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    $(OCTOMAP_INCLUDE_DIRS)
    include/
    ${Eigen_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
# src/frontiers_extraction.cpp
# src/stage_interface.cpp
# )


add_executable(frontiers_extraction_node src/frontiers_extraction_node.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(
   frontiers_extraction_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_LIBRARIES}
    ${CMAKE_DL_LIBS}
    UFO::Map
   )


add_executable(graph_management_node src/graph_management_node.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(
   graph_management_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_LIBRARIES}
    ${CMAKE_DL_LIBS}
    UFO::Map
   )

add_executable(stage_interface_node src/stage_interface_node.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(
   stage_interface_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_LIBRARIES}
  ${CMAKE_DL_LIBS}
  UFO::Map
)


# add_executable(aerial_robot_node src/aerial_robot.cpp)
# add_executable(ground_robot_node src/ground_robot.cpp)

# target_link_libraries(aerial_robot_node
  # ${catkin_LIBRARIES}
  # ${CMAKE_DL_LIBS}
  # UFO::Map
# )

# target_link_libraries(ground_robot_node
  # ${catkin_LIBRARIES}
  # ${CMAKE_DL_LIBS}
  # UFO::Map
# )

