#ifndef VISUALIZATION_SEREVR_H_
#define VISUALIZATION_SEREVR_H_



#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

#include "geofence_manager.h"
#include "graph.h"
#include "graph_utils.h"
#include "graph_management.h"
#include "map_manager.h"
#include "params.h"

class VisualizationServer {
public:
  VisualizationServer(const ros::NodeHandle&nh, const ros::NodeHandle& nh_private);
  
  std_msgs::ColorRGBA getGainColor(double h);
  void visualizePathWaysGraph(std::shared_ptr<GraphUtils> path_ways_graph);
  void visualizeVoxelGain (std::shared_ptr<GraphUtils> graph);
  void visualizeLocalGraph (std::shared_ptr<GraphUtils> local_graph);
  void visualizeGlobalGraph (std::shared_ptr<GraphUtils> global_graph);
  void visualizeSparseGraph (std::shared_ptr<GraphUtils> sparse_graph);
  void visualizePoseGraph (std::shared_ptr<GraphUtils> pose_graph);
  void visualizeLocalPaths (std::vector<std::pair<std::vector<Vertex *>, double>> 
                            local_paths);
  void visualizeGlobalPaths (std::vector<std::pair<std::vector<Vertex *>, double>> 
                             global_paths);
  void visualizeExplorationDirection (geometry_msgs::PoseStamped 
                                      exploration_dir);
  void visualizeHomingPath (nav_msgs::Path homing_path);
  void visualizeHomingPath (nav_msgs::Path homing_path);
  void visualizeExplorationPath (nav_msgs::Path exploration_path);
  void visualizePclMap (const pcl::PointCloud<pcl::PointXYZ>* pcl);



private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;


  ros::Publisher modified_Path_Vis_Pub;
  ros::Publisher homing_Path_Vis_Pub;
  ros::Publisher local_Paths_Vis_Pub_;
  ros::Publisher voxel_Gain_Vis_Pub_;
  ros::Publisher global_Paths_Vis_Pub_;
  ros::Publisher reposition_Path_Vis_Pub_;
  ros::Publisher local_Graph_Vis_Pub_;
  ros::Publisher global_Graph_Vis_Pub_;
  ros::Publisher path_Ways_Graph_Vis_Pub_;
  ros::Publisher pose_Graph_Vis_Pub_;
  ros::Publisher overlap_Vis_Pub_;
  ros::Publisher random_Nodes_Vis_Pub_;
  ros::Publisher sparse_Graph_Vis_Pub_;
  ros::Publisher exploration_Direction_Vis_Pub;
  ros::Publisher pcl_Pub_;

  std::string world_frame_id; 
};

























#endif // VISUALIZATION_SEREVR_H_
