cmake_minimum_required(VERSION 3.0.2)
project(stage_planner)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(octomap_DIR "/opt/ros/noetic/share/octomap")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)

find_package (PCL 1.10 REQUIRED)

set(PCL_INCLUDE_DIRS /usr/include/pcl-1.10 /usr/include/eigen3/)

include_directories(${PCL_INCLUDE_DIRS})


include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message("OpenMP enabled")
endif()

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
  actionlib_msgs
  message_generation
  ufomap_mapping
  roscpp_serialization
  kdtree
)

find_package(ufomap REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(roscpp_serialization REQUIRED)


include_directories( ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  Frontiers.msg
  Frontier.msg
  path.msg
  RobotStatus.msg
  RectangleShape.msg
  BoundMode.msg
  PlanningMode.msg
  ExecutionPathMode.msg
  TriggerMode.msg
  PlannerStatus.msg
  PlanningBound.msg
  Vertex.msg
  Edge.msg
  Graph.msg
  CoveragePlannerLogger.msg
  BehaviourPlannerLogger.msg
  SemanticClass.msg
  SemanticPoint.msg
  SemanticPolygon.msg
)

add_service_files(
  FILES
  frontierRequest.srv  
  planner_srv.srv
  planner_global.srv
  planner_search.srv
  planner_homing.srv
  planner_set_homing_pos.srv
  planner_geofence.srv
  planner_request_path.srv
  pci_trigger.srv
  pci_homing_trigger.srv
  pci_set_homing_pos.srv
  pci_initialization.srv
  pci_search.srv
  pci_global.srv
  pci_stop.srv
  pci_geofence.srv
  pci_to_waypoint.srv
  planner_set_global_bound.srv
  planner_set_vel.srv
  planner_set_exp_mode.srv
  planner_string_trigger.srv
  planner_set_search_mode.srv
  planner_go_to_waypoint.srv
  planner_dynamic_global_bound.srv
  planner_set_planning_mode.srv
)


add_action_files(
 FILES
 pathFollowerAction.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs 
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
 CATKIN_DEPENDS
 message_runtime
 actionlib_msgs
)
include_directories(
    ${catkin_INCLUDE_DIRS}
    $(OCTOMAP_INCLUDE_DIRS)
    include/
    ${Eigen_INCLUDE_DIRS}
	  ${Boost_INCLUDE_DIRS}
)


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



add_library(${PROJECT_NAME}
  src/graph_manager.cpp
  src/graph.cpp
  src/params.cpp
  src/trajectory.cpp
  src/random_sampler.cpp
  src/geofence_manager.cpp

)

# cs_install()
# cs_export(LIBRARIES)


