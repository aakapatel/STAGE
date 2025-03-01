#include "visualization_server.h"

VisualizationServer::VisualizationServer(const ros::NodeHandle& nh,
                                         const ros::NodeHandle& nh_private) {
  nh_ = nh;
  nh_private_ = nh_private;

  exploration_Path_Vis_Pub =
      nh_.advertise<nav_msgs::Path>("exploration_path", 10);

  raw_Path_Vis_Pub = nh_.advertise<nav_msgs::Path>("raw_path", 10);

  homing_Path_Vis_Pub = nh_.advertise<nav_msgs::Path>("homing_path", 10);

  exploration_Direction_Vis_Pub =
      nh_.advertise<geometry_msgs::PoseStamped>("exploration_direction", 10);

  local_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "local_planning_graph", 10);

  voxel_Gain_Vis_Pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("voxel_gain", 10);

  global_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "global_planning_graph", 10);

  path_Ways_Graph_Vis_Pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("path_ways_graph", 10);

  pose_Graph_Vis_Pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("pose_graph", 10);

  local_Paths_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "local_dijkstra_paths", 10);

  global_Paths_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "global_dijkstra_paths", 10);

  pcl_Pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 10);

  reposition_Path_Vis_Pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("global_repos_path", 10);

  random_Nodes_Vis_Pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("random_nodes", 10);

  dense_graph_Vis_Pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("dense_graph", 10);

  semantic_graph_publisher_ =
      nh_.advertise<stage_planner::Graph>("semantic_graph", 10);
}

std_msgs::ColorRGBA VisualizationServer::getGainColor(double h) {
  std_msgs::ColorRGBA color;
  color.a = 0.8;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}

void VisualizationServer::visualizePathWaysGraph(
    std::shared_ptr<GraphUtils> path_ways_graph) {
  if (path_ways_graph->getNumVertices() == 0) {
    return;
  }
  std::shared_ptr<Graph> g = path_ways_graph->graph_;
  std::unordered_map<int, Vertex*>& v_map = path_ways_graph->vertices_map_;

  visualization_msgs::MarkerArray marker_array;
  std_msgs::ColorRGBA vertex_color;
  std_msgs::ColorRGBA frontier_color;
  vertex_color.r = 125.0 / 255.0;
  vertex_color.g = 42.0 / 255.0;
  vertex_color.b = 104.0 / 255.0;
  vertex_color.a = 1.0;
  frontier_color.r = 250 / 255.0;
  frontier_color.g = 0;
  frontier_color.b = 0;
  frontier_color.a = 1.0;

  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 90.0 / 255.0;
  edge_marker.color.g = 80.0 / 255.0;
  edge_marker.color.b = 100.0;
  edge_marker.color.a = 0.8;
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[src_id]->state[0];
    p1.y = v_map[src_id]->state[1];
    p1.z = v_map[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[tgt_id]->state[0];
    p2.y = v_map[tgt_id]->state[1];
    p2.z = v_map[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);
  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.4;
  vertex_marker.scale.y = 0.4;
  vertex_marker.scale.z = 0.4;
  vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);
    if (v_map[id]->vol_gain.is_frontier) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      vertex_marker.colors.push_back(frontier_color);
      vertex_marker.points.push_back(p1);
    } else {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      vertex_marker.colors.push_back(vertex_color);
      vertex_marker.points.push_back(p1);
    }
  }
  marker_array.markers.push_back(vertex_marker);

  path_Ways_Graph_Vis_Pub_.publish(marker_array);
}

void VisualizationServer::visualizeVoxelGain(
    std::shared_ptr<GraphUtils> graph) {
  if (graph->frontier_vertices_ids_.empty()) {
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker unknown_marker;
  visualization_msgs::Marker free_marker;
  visualization_msgs::Marker occupied_marker;
  for (int id : graph->frontier_vertices_ids_) {
    Vertex* vertex = graph->vertices_map_[id];

    std::vector<Eigen::Vector3d> unknown = vertex->vol_gain.unknown_voxels;
    std::vector<Eigen::Vector3d> free = vertex->vol_gain.free_voxels;
    std::vector<Eigen::Vector3d> occupied = vertex->vol_gain.occupied_voxels;

    unknown_marker.header.stamp = ros::Time::now();
    unknown_marker.header.seq = 0;
    unknown_marker.header.frame_id = world_frame_id;
    unknown_marker.id = 0;
    unknown_marker.ns = "unknown";
    unknown_marker.action = visualization_msgs::Marker::ADD;
    unknown_marker.type = visualization_msgs::Marker::CUBE_LIST;
    unknown_marker.scale.x = 0.1;
    unknown_marker.scale.y = 0.1;
    unknown_marker.scale.z = 0.1;
    unknown_marker.color.r = 0.0 / 255.0;
    unknown_marker.color.g = 255.0 / 255.0;
    unknown_marker.color.b = 0.0 / 255;
    unknown_marker.color.a = 0.8;
    unknown_marker.frame_locked = false;

    free_marker.header.stamp = ros::Time::now();
    free_marker.header.seq = 0;
    free_marker.header.frame_id = world_frame_id;
    free_marker.id = 0;
    free_marker.ns = "free";
    free_marker.action = visualization_msgs::Marker::ADD;
    free_marker.type = visualization_msgs::Marker::CUBE_LIST;
    free_marker.scale.x = 0.1;
    free_marker.scale.y = 0.1;
    free_marker.scale.z = 0.1;
    free_marker.color.r = 0.0 / 255.0;
    free_marker.color.g = 0.0 / 255.0;
    free_marker.color.b = 255.0;
    free_marker.color.a = 0.8;
    free_marker.frame_locked = false;

    occupied_marker.header.stamp = ros::Time::now();
    occupied_marker.header.seq = 0;
    occupied_marker.header.frame_id = world_frame_id;
    occupied_marker.id = 0;
    occupied_marker.ns = "occupied";
    occupied_marker.action = visualization_msgs::Marker::ADD;
    occupied_marker.type = visualization_msgs::Marker::CUBE_LIST;
    occupied_marker.scale.x = 0.1;
    occupied_marker.scale.y = 0.1;
    occupied_marker.scale.z = 0.1;
    occupied_marker.color.r = 255.0 / 255.0;
    occupied_marker.color.g = 0.0 / 255.0;
    occupied_marker.color.b = 0.0;
    occupied_marker.color.a = 0.8;
    occupied_marker.frame_locked = false;

    for (int i = 0; i < unknown.size(); i++) {
      geometry_msgs::Point p1;
      p1.x = unknown[i][0];
      p1.y = unknown[i][1];
      p1.z = unknown[i][2];
      unknown_marker.points.push_back(p1);
    }
    for (int i = 0; i < free.size(); i++) {
      geometry_msgs::Point p1;
      p1.x = free[i][0];
      p1.y = free[i][1];
      p1.z = free[i][2];
      free_marker.points.push_back(p1);
    }
    for (int i = 0; i < occupied.size(); i++) {
      geometry_msgs::Point p1;
      p1.x = occupied[i][0];
      p1.y = occupied[i][1];
      p1.z = occupied[i][2];
      occupied_marker.points.push_back(p1);
    }

    marker_array.markers.push_back(unknown_marker);
    marker_array.markers.push_back(free_marker);
    marker_array.markers.push_back(occupied_marker);
  }

  voxel_Gain_Vis_Pub_.publish(marker_array);
}

void VisualizationServer::visualizeLocalGraph(
    std::shared_ptr<GraphUtils> local_graph) {
  std::shared_ptr<Graph> g = local_graph->graph_;
  std::unordered_map<int, Vertex*>& v_map = local_graph->vertices_map_;

  if (local_graph->getNumVertices() == 0) return;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 0.0 / 255.0;
  edge_marker.color.g = 153.0 / 255.0;
  edge_marker.color.b = 51.0 / 255.0;
  edge_marker.color.a = 0.8;
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[src_id]->state[0];
    p1.y = v_map[src_id]->state[1];
    p1.z = v_map[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[tgt_id]->state[0];
    p2.y = v_map[tgt_id]->state[1];
    p2.z = v_map[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.25;
  vertex_marker.scale.y = 0.25;
  vertex_marker.scale.z = 0.25;
  vertex_marker.frame_locked = false;

  visualization_msgs::Marker frontier_vertex_marker;
  frontier_vertex_marker.header.stamp = ros::Time::now();
  frontier_vertex_marker.header.seq = 0;
  frontier_vertex_marker.header.frame_id = world_frame_id;
  frontier_vertex_marker.id = 0;
  frontier_vertex_marker.ns = "frontier";
  frontier_vertex_marker.action = visualization_msgs::Marker::ADD;
  frontier_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  frontier_vertex_marker.scale.x = 0.4;
  frontier_vertex_marker.scale.y = 0.4;
  frontier_vertex_marker.scale.z = 0.4;
  frontier_vertex_marker.color.r = 255.0 / 255.0;
  frontier_vertex_marker.color.g = 0.0 / 255.0;
  frontier_vertex_marker.color.b = 0.0 / 255.0;
  frontier_vertex_marker.color.a = 1.0;
  frontier_vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  g->getVertexIterator(vi);

  double minGain = std::numeric_limits<double>::max();
  double maxGain = -std::numeric_limits<double>::max();

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);
    if (v_map[id]->vol_gain.gain > maxGain) {
      maxGain = v_map[id]->vol_gain.gain;
    }

    if (v_map[id]->vol_gain.gain < minGain) {
      minGain = v_map[id]->vol_gain.gain;
    }
  }

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];
    double gain = v_map[id]->vol_gain.gain;
    double h =
        (1.0 -
         std::min(std::max((gain - minGain) / (maxGain - minGain), 0.0), 1.0)) *
        0.8;
    vertex_marker.colors.push_back(getGainColor(h));
    vertex_marker.points.push_back(p1);
    if (v_map[id]->vol_gain.is_frontier) {
      // if (v_map[id]->is_leaf_vertex) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];

      frontier_vertex_marker.points.push_back(p1);
    }
  }
  marker_array.markers.push_back(vertex_marker);
  marker_array.markers.push_back(frontier_vertex_marker);
  local_Graph_Vis_Pub_.publish(marker_array);
}

void VisualizationServer::visualizeGlobalGraph(
    std::shared_ptr<GraphUtils> global_graph) {
  if (global_graph->getNumVertices() == 0) return;

  std::shared_ptr<Graph> g = global_graph->graph_;
  std::unordered_map<int, Vertex*>& v_map = global_graph->vertices_map_;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 173.0 / 255.0;
  edge_marker.color.g = 216.0 / 255.0;
  edge_marker.color.b = 230.0 / 255.0;
  edge_marker.color.a = 1.0;
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);

    geometry_msgs::Point p1;
    p1.x = v_map[src_id]->state[0];
    p1.y = v_map[src_id]->state[1];
    p1.z = v_map[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[tgt_id]->state[0];
    p2.y = v_map[tgt_id]->state[1];
    p2.z = v_map[tgt_id]->state[2];
    if (v_map[src_id]->is_semantic) {
      p1.z = v_map[src_id]->state[2] + 10;
    }
    if (v_map[tgt_id]->is_semantic) {
      p2.z = v_map[tgt_id]->state[2] + 10;
    }

    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.color.r = 0.0 / 255.0;
  vertex_marker.color.g = 255.0 / 255.0;
  vertex_marker.color.b = 255.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.frame_locked = false;

  visualization_msgs::Marker frontier_vertex_marker;
  frontier_vertex_marker.header.stamp = ros::Time::now();
  frontier_vertex_marker.header.seq = 0;
  frontier_vertex_marker.header.frame_id = world_frame_id;
  frontier_vertex_marker.id = 0;
  frontier_vertex_marker.ns = "frontier";
  frontier_vertex_marker.action = visualization_msgs::Marker::ADD;
  frontier_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  frontier_vertex_marker.scale.x = 0.4;
  frontier_vertex_marker.scale.y = 0.4;
  frontier_vertex_marker.scale.z = 0.4;
  frontier_vertex_marker.color.r = 255.0 / 255.0;
  frontier_vertex_marker.color.g = 20.0 / 255.0;
  frontier_vertex_marker.color.b = 145.0 / 255.0;
  frontier_vertex_marker.color.a = 1.0;
  frontier_vertex_marker.frame_locked = false;

  visualization_msgs::Marker semantic_vertex;
  semantic_vertex.header.stamp = ros::Time::now();
  semantic_vertex.header.seq = 0;
  semantic_vertex.header.frame_id = world_frame_id;
  semantic_vertex.id = 0;
  semantic_vertex.ns = "semantic";
  semantic_vertex.action = visualization_msgs::Marker::ADD;
  semantic_vertex.type = visualization_msgs::Marker::SPHERE_LIST;
  semantic_vertex.scale.x = 0.8;
  semantic_vertex.scale.y = 0.8;
  semantic_vertex.scale.z = 0.8;
  semantic_vertex.color.r = 0.0 / 255.0;
  semantic_vertex.color.g = 255.0 / 255.0;
  semantic_vertex.color.b = 0.0 / 255.0;
  semantic_vertex.color.a = 1.0;
  semantic_vertex.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);
    if (v_map[id]->vol_gain.is_frontier) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];

      frontier_vertex_marker.points.push_back(p1);
    }

    if (v_map[id]->is_semantic) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2] + 10;

      semantic_vertex.points.push_back(p1);
    }

    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];

    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);
  marker_array.markers.push_back(frontier_vertex_marker);
  marker_array.markers.push_back(semantic_vertex);

  global_Graph_Vis_Pub_.publish(marker_array);
}

void VisualizationServer::visualizeLocalPaths(
    std::vector<std::pair<std::vector<Vertex*>, double>> local_paths) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker edge_marker;
  visualization_msgs::Marker vertex_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 255.0 / 255.0;
  edge_marker.color.g = 0.0 / 255.0;
  edge_marker.color.b = 255.0 / 255.0;
  edge_marker.color.a = 1.0;
  edge_marker.frame_locked = false;

  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.color.r = 255.0 / 255.0;
  vertex_marker.color.g = 0.0 / 255.0;
  vertex_marker.color.b = 255.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.frame_locked = false;

  for (int i = 0; i < local_paths.size(); ++i) {
    for (int it = 0; it < local_paths[i].first.size() - 1; ++it) {
      geometry_msgs::Point p1;
      geometry_msgs::Point p2;
      p1.x = local_paths[i].first[it]->state[0];
      p1.y = local_paths[i].first[it]->state[1];
      p1.z = local_paths[i].first[it]->state[2];
      p2.x = local_paths[i].first[it + 1]->state[0];
      p2.y = local_paths[i].first[it + 1]->state[1];
      p2.z = local_paths[i].first[it + 1]->state[2];

      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
      vertex_marker.points.push_back(p1);
      vertex_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(edge_marker);
  marker_array.markers.push_back(vertex_marker);

  local_Paths_Vis_Pub_.publish(marker_array);
}

void VisualizationServer::visualizeGlobalPaths(
    std::vector<std::pair<std::vector<Vertex*>, double>> global_paths) {
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edge_marker;
  visualization_msgs::Marker vertex_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.1;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 0.0 / 255.0;
  edge_marker.color.b = 20.0 / 255.0;
  edge_marker.color.a = 0.8;
  edge_marker.frame_locked = false;

  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.4;
  vertex_marker.scale.y = 0.4;
  vertex_marker.scale.z = 0.4;
  vertex_marker.color.r = 200.0 / 255.0;
  vertex_marker.color.g = 0 / 255.0;
  vertex_marker.color.b = 20.0 / 255.0;
  vertex_marker.color.a = 0.8;
  vertex_marker.frame_locked = false;

  for (int i = 0; i < global_paths.size(); ++i) {
    for (int it = 0; it < global_paths[i].first.size() - 1; ++it) {
      geometry_msgs::Point p1;
      geometry_msgs::Point p2;
      p1.x = global_paths[i].first[it]->state[0];
      p1.y = global_paths[i].first[it]->state[1];
      p1.z = global_paths[i].first[it]->state[2];
      p2.x = global_paths[i].first[it + 1]->state[0];
      p2.y = global_paths[i].first[it + 1]->state[1];
      p2.z = global_paths[i].first[it + 1]->state[2];

      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
      vertex_marker.points.push_back(p1);
      vertex_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(edge_marker);
  marker_array.markers.push_back(vertex_marker);

  global_Paths_Vis_Pub_.publish(marker_array);
}

void VisualizationServer::visualizeDenseGraph(
    std::shared_ptr<GraphUtils> dense_graph) {
  if (dense_graph->getNumVertices() == 0) return;

  std::shared_ptr<Graph> g = dense_graph->graph_;
  std::unordered_map<int, Vertex*>& v_map = dense_graph->vertices_map_;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 90.0 / 255.0;
  edge_marker.color.g = 80.0 / 255.0;
  edge_marker.color.b = 100.0;
  edge_marker.color.a = 0.8;
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[src_id]->state[0];
    p1.y = v_map[src_id]->state[1];
    p1.z = v_map[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[tgt_id]->state[0];
    p2.y = v_map[tgt_id]->state[1];
    p2.z = v_map[tgt_id]->state[2];
    if (v_map[src_id]->is_semantic) {
      p1.z = v_map[src_id]->state[2] + 5;
    }
    if (v_map[tgt_id]->is_semantic) {
      p2.z = v_map[tgt_id]->state[2] + 5;
    }
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);
  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.frame_locked = false;
  vertex_marker.color.r = 125.0 / 255.0;
  vertex_marker.color.g = 42.0 / 255.0;
  vertex_marker.color.b = 104.0 / 255.0;
  vertex_marker.color.a = 1.0;

  visualization_msgs::Marker frontier_vertex_marker;
  frontier_vertex_marker.header.stamp = ros::Time::now();
  frontier_vertex_marker.header.seq = 0;
  frontier_vertex_marker.header.frame_id = world_frame_id;
  frontier_vertex_marker.id = 0;
  frontier_vertex_marker.ns = "frontier";
  frontier_vertex_marker.action = visualization_msgs::Marker::ADD;
  frontier_vertex_marker.type = visualization_msgs::Marker::CUBE_LIST;
  frontier_vertex_marker.scale.x = 0.3;
  frontier_vertex_marker.scale.y = 0.3;
  frontier_vertex_marker.scale.z = 0.3;
  frontier_vertex_marker.color.r = 0.0 / 255.0;
  frontier_vertex_marker.color.g = 0.0 / 255.0;
  frontier_vertex_marker.color.b = 200.0 / 255.0;
  frontier_vertex_marker.color.a = 0.8;
  frontier_vertex_marker.frame_locked = false;

  visualization_msgs::Marker semantic_vertex;
  semantic_vertex.header.stamp = ros::Time::now();
  semantic_vertex.header.seq = 0;
  semantic_vertex.header.frame_id = world_frame_id;
  semantic_vertex.id = 0;
  semantic_vertex.ns = "semantic";
  semantic_vertex.action = visualization_msgs::Marker::ADD;
  semantic_vertex.type = visualization_msgs::Marker::SPHERE_LIST;
  semantic_vertex.scale.x = 0.5;
  semantic_vertex.scale.y = 0.5;
  semantic_vertex.scale.z = 0.5;
  semantic_vertex.color.r = 0.0 / 255.0;
  semantic_vertex.color.g = 255.0 / 255.0;
  semantic_vertex.color.b = 0.0 / 255.0;
  semantic_vertex.color.a = 1.0;
  semantic_vertex.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);

    if (v_map[id]->vol_gain.is_frontier) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];

      frontier_vertex_marker.points.push_back(p1);
    } else if (v_map[id]->is_semantic) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2] + 5;

      semantic_vertex.points.push_back(p1);
    } else {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];

      vertex_marker.points.push_back(p1);
    }
  }
  marker_array.markers.push_back(vertex_marker);
  marker_array.markers.push_back(frontier_vertex_marker);
  marker_array.markers.push_back(semantic_vertex);
  dense_graph_Vis_Pub_.publish(marker_array);
}

void VisualizationServer::visualizePoseGraph(
    std::shared_ptr<GraphUtils> pose_graph) {
  if (pose_graph->getNumVertices() == 0) return;
  std::shared_ptr<Graph> g = pose_graph->graph_;
  std::unordered_map<int, Vertex*>& v_map = pose_graph->vertices_map_;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 240.0 / 255.0;
  edge_marker.color.g = 230.0 / 255.0;
  edge_marker.color.b = 130.0 / 255.0;
  edge_marker.color.a = 1.0;
  // edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);

    geometry_msgs::Point p1;
    p1.x = v_map[src_id]->state[0];
    p1.y = v_map[src_id]->state[1];
    p1.z = v_map[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[tgt_id]->state[0];
    p2.y = v_map[tgt_id]->state[1];
    p2.z = v_map[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.color.r = 240.0 / 255.0;
  vertex_marker.color.g = 230.0 / 255.0;
  vertex_marker.color.b = 130.0 / 255.0;
  vertex_marker.color.a = 1.0;
  // vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];

    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  pose_Graph_Vis_Pub_.publish(marker_array);
}

void VisualizationServer::visualizeExplorationDirection(
    geometry_msgs::PoseStamped exploration_dir) {
  exploration_dir.header.frame_id = world_frame_id;
  exploration_dir.header.stamp = ros::Time::now();
  exploration_Direction_Vis_Pub.publish(exploration_dir);
}

void VisualizationServer::visualizeHomingPath(nav_msgs::Path homing_path) {
  homing_path.header.frame_id = world_frame_id;
  homing_path.header.stamp = ros::Time::now();
  homing_Path_Vis_Pub.publish(homing_path);
}

void VisualizationServer::visualizeExplorationPath(
    nav_msgs::Path exploration_path) {
  exploration_path.header.frame_id = world_frame_id;
  exploration_path.header.stamp = ros::Time::now();
  exploration_Path_Vis_Pub.publish(exploration_path);
}

void VisualizationServer::visualizeRawPath(nav_msgs::Path raw_path) {
  raw_path.header.frame_id = world_frame_id;
  raw_path.header.stamp = ros::Time::now();
  raw_Path_Vis_Pub.publish(raw_path);
}

void VisualizationServer::publishSemanticGraph(stage_planner::Graph graph) {
  semantic_graph_publisher_.publish(graph);
  ROS_DEBUG("Published semantic graph \n");
}

void VisualizationServer::visualizePclMap(
    const pcl::PointCloud<pcl::PointXYZ>* pcl) {
  if (pcl_Pub_.getNumSubscribers() < 1) return;

  ROS_DEBUG("[PCL Map vis]: PCL size: %d", (int)(pcl->points.size()));
  if (pcl->points.size() <= 0) {
    ROS_WARN("[PCL Map Vis]: Empty PCL");
    return;
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*pcl, output);
  output.header.frame_id = world_frame_id;
  output.header.stamp = ros::Time::now();
  pcl_Pub_.publish(output);
}
