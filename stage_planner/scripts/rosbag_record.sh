#!/bin/bash

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -n|--namespace)
        ROBOT_NAMESPACE="$2"
        shift # past argument
        shift # past value
        ;;
        -b|--bagname)
        BAG_NAME="$2"
        shift # past argument
        shift # past value
        ;;
        *)    # unknown option
        shift # past argument
        ;;
    esac
done

# Check if required arguments are provided
if [ -z "$ROBOT_NAMESPACE" ] || [ -z "$BAG_NAME" ]; then
    echo "Usage: ./record_bag.sh -n <robot_namespace> -b <bag_name>"
    exit 1
fi

rosbag record -o $BAG_NAME /tf /tf_static /$ROBOT_NAMESPACE/velodyne_points /$ROBOT_NAMESPACE/ouster/points /$ROBOT_NAMESPACE/odometry/imu /$ROBOT_NAMESPACE/stage_interface_path/goal /$ROBOT_NAMESPACE/sparse_graph /$ROBOT_NAMESPACE/stage_interface_node/occupied_nodes /$ROBOT_NAMESPACE/stage_interface_node/mesh /$ROBOT_NAMESPACE/stage_interface_node/surface_pointcloud /$ROBOT_NAMESPACE/raw_path /$ROBOT_NAMESPACE/pose_graph /$ROBOT_NAMESPACE/path_ways_graph /$ROBOT_NAMESPACE/modified_path /$ROBOT_NAMESPACE/local_planning_graph /$ROBOT_NAMESPACE/local_dijkstra_paths /$ROBOT_NAMESPACE/lio_sam/mapping/path /$ROBOT_NAMESPACE/lio_sam/mapping/map_global /$ROBOT_NAMESPACE/improved_global_dijkstra_paths /$ROBOT_NAMESPACE/homing_path /$ROBOT_NAMESPACE/global_repos_path /$ROBOT_NAMESPACE/global_planning_graph /$ROBOT_NAMESPACE/global_dijkstra_paths /$ROBOT_NAMESPACE/exploration_path /$ROBOT_NAMESPACE/exploration_direction /$ROBOT_NAMESPACE/command_path /$ROBOT_NAMESPACE/command/trajectory /$ROBOT_NAMESPACE/command/pose /$ROBOT_NAMESPACE/carrot_point /move_base_simple/goal /$ROBOT_NAMESPACE/camera/color/image_raw /$ROBOT_NAMESPACE/camera/color/camera_info /$ROBOT_NAMESPACE/camera/depth/camera_info /$ROBOT_NAMESPACE/camera/aligned_depth_to_color/image_raw  
