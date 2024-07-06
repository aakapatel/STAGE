cmake_minimum_required(VERSION 3.0.2)
project(stage_planner)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package (PCL 1.10 REQUIRED)

# set(PCL_INCLUDE_DIRS /usr/include/pcl-1.10 /usr/include/eigen3/)


# Point collision checking:
#   0 - Orignal
#   1 - Direct TSDF/ESDF distance check (requires continuous path check during execution)
#   2 - Using interpolated tsdf distance
add_definitions(-DCOL_CHECK_METHOD=0)
# Line collision checking: 
#   0 - Orignal
#   1 - Cuboid around the edge
#   2 - Direct TSDF/ESDF distance check (requires continuous path check during execution)
#   3 - Using interpolated tsdf distance
add_definitions(-DEDGE_CHECK_METHOD=1)
# Ray casting: 
#   0 - Original
#   1 - Iterative ray casting
add_definitions(-DRAY_CAST_METHOD=1)

# Verify the values of the definitions
get_directory_property( DirDefs COMPILE_DEFINITIONS ) 
message( "COMPILE DEFINITIONS: ${DirDefs}" )



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
  tf2_ros
  roslib
  geometry_msgs
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
  kdtree
  voxblox_ros
  voxblox
)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories( ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  Vertex.msg
  Edge.msg
  Graph.msg 
)

add_service_files(
  FILES
  frontierRequest.srv  
  isFrontier.srv
  explorationPath.srv
  initializePlanning.srv
  stopPlanning.srv
  homePlanning.srv
  shareGraph.srv
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
    include/
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/graph.cpp
  src/graph_utils.cpp
  src/voxblox/voxblox_common_impl.cpp
  src/voxblox/voxblox_alt_impl.cpp
  src/params.cpp
  src/trajectory.cpp
  src/random_sampler.cpp
  src/geofence_manager.cpp
  src/path_tracker.cpp
  src/visualization_server.cpp
  src/graph_management.cpp
)


add_executable(exploration_manager_node src/exploration_manager_node.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(
   exploration_manager_node
   stage_planner
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
  ${CMAKE_DL_LIBS}
)
add_executable(stage_interface_node src/stage_interface_node.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(
   stage_interface_node
   stage_planner
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
  ${CMAKE_DL_LIBS}
)
add_executable(control_interface src/control_interface.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(
   control_interface
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

add_executable(spot_path_tracker_ros_node src/path_tracker_node.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(
   spot_path_tracker_ros_node
   stage_planner
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

cs_install()
# cs_export(LIBRARIES)
