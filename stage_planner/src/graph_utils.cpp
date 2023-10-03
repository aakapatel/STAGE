#include "graph_utils.h"

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
GraphUtils::GraphUtils () {

  kd_tree_ = NULL;
  reset();
}


// GRAPH UTILS

void GraphUtils::reset() {

  // Reset kdtree first.
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);

  // Reset graph.
  graph_.reset(new Graph());

  // Vertex mapping.
  // Clear memory for all vertex pointers
  if (vertices_map_.size() > 0) {
    for (int i = 0; i < vertices_map_.size(); ++i) {
      delete vertices_map_[i];
    }
  }
  vertices_map_.clear();
  edge_map_.clear();

  // Other params.
  subgraph_ind_ = -1;
  id_count_ = -1;

}

bool GraphUtils::edgeExist (int source_id, int target_id) {
  
  bool exist = false;
  std::shared_ptr<Graph> g = graph_;
  std::unordered_map<int, Vertex*>& v_map = vertices_map_;
  if (getNumVertices() == 0) return false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
    if (tgt_id == source_id and src_id == target_id) {
      exist = true;
    } else {
      exist = false;
    }
  }
  return exist;
}

int GraphUtils::getIdCount () {
  return id_count_;
}

Vertex* GraphUtils::getVertexBySubId (int s_id) {
  
  // std::chrono::high_resolution_clock::time_point start_total = std::chrono::high_resolution_clock::now();
  Vertex* v;
  if (graph_->getNumVertices() == 0) {
    return NULL;
  }
  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator> vi;
  graph_->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = graph_->getVertexProperty(it);
    if (s_id == vertices_map_[id]->sub_id) {
      
      v = vertices_map_[id]; 
    }
  }

  // std::chrono::high_resolution_clock::time_point stop_total = std::chrono::high_resolution_clock::now();
  //
  // auto duration_total = std::chrono::duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  //
  // ROS_INFO_STREAM("\033[;34m vertex by sub id time \033[0m "
  //                 << duration_total.count() << " \033[;34m ms \033[0m\n ");

  return v;
}

int GraphUtils::getNumNeighbours (Vertex* v) {

  std::vector<Vertex*> nearby_vertices;
  getNearestVertices(&vertices_map_[v->id]->state, 2, &nearby_vertices);
  return nearby_vertices.size();
}

double GraphUtils::distanceBetweenVertices (Vertex*v, Vertex*u) {
  
  return sqrt(pow(v->state.x() - u->state.x(), 2 ) 
              +pow(v->state.y() - u->state.y(), 2 )
                +pow(v->state.z() - u->state.z(), 2 ));

}

void GraphUtils::insertVertexKdtree (Vertex* v) {

  
  kd_insert3(kd_tree_, v->state.x(), v->state.y(), v->state.z(), v);

}

void GraphUtils::addVertex(Vertex* v) {
  kd_insert3(kd_tree_, v->state.x(), v->state.y(), v->state.z(), v);
  // v->sub_graph_id = subgraph_ind_;
  // v->sub_id = v->id;
  if (v->id == 0)
    graph_->addSourceVertex(0);
  else
    graph_->addVertex(v->id);
  vertices_map_[v->id] = v;
}

void GraphUtils::addEdge(Vertex* v, Vertex* u, double weight) {
  graph_->addEdge(v->id, u->id, weight);
  edge_map_[v->id].push_back(std::make_pair(u->id, weight));
  edge_map_[u->id].push_back(std::make_pair(v->id, weight));
}

void GraphUtils::removeEdge(Vertex* v, Vertex* u) {
  graph_->removeEdge(v->id, u->id);
}

void GraphUtils::clearVertex (int id) {
  graph_->clearVertex(id);
}

void GraphUtils::removeVertex (int id) {
  graph_->removeVertex(id);
}

bool GraphUtils::getNearestVertex(const State* state, Vertex** v_res) {
  if (getNumVertices() <= 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *v_res = (Vertex*)kd_res_item_data(nearest);
  kd_res_free(nearest);
  return true;
}

bool GraphUtils::getNearestVertexInRange(const State* state, double range,
                                           Vertex** v_res) {
  if (getNumVertices() <= 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *v_res = (Vertex*)kd_res_item_data(nearest);
  Eigen::Vector3d dist;
  dist << state->x() - (*v_res)->state.x(), state->y() - (*v_res)->state.y(),
      state->z() - (*v_res)->state.z();
  kd_res_free(nearest);
  if (dist.norm() > range) return false;
  return true;
}

bool GraphUtils::getNearestVertices(const State* state, double range,
                                      std::vector<Vertex*>* v_res) {
  // Notice that this might include the same vertex in the result.
  // if that vertex is added to the tree before.
  // Use the distance 0 or small threshold to filter out.
  kdres* neighbors =
      kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) return false;
  v_res->clear();
  for (int i = 0; i < neighbors_size; ++i) {
    Vertex* new_neighbor = (Vertex*)kd_res_item_data(neighbors);
    v_res->push_back(new_neighbor);
    if (kd_res_next(neighbors) <= 0) break;
  }
  kd_res_free(neighbors);
  return true;
}

bool GraphUtils::findShortestPaths(ShortestPathsReport& rep) {
  // cout << "status before " << rep.status << "\n" << endl;
  return graph_->findDijkstraShortestPaths(0, rep);
  // cout << "status after " << rep.status << "\n" << endl;
}

bool GraphUtils::findShortestPaths(int source_id, ShortestPathsReport& rep) {
  return graph_->findDijkstraShortestPaths(source_id, rep);
}

void GraphUtils::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<Vertex*>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.push_back(vertices_map_[*p]);
  }
}

void GraphUtils::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<Eigen::Vector3d>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.emplace_back(Eigen::Vector3d(vertices_map_[*p]->state.x(),
                                      vertices_map_[*p]->state.y(),
                                      vertices_map_[*p]->state.z()));
  }
}

// void GraphUtils::getShortestPath(int target_id,
//                                    const ShortestPathsReport& rep,
//                                    bool source_to_target_order,
//                                    std::vector<State>& path) {
//   std::vector<int> path_id;
//   getShortestPath(target_id, rep, source_to_target_order, path_id);
//   for (auto p = path_id.begin(); p != path_id.end(); ++p) {
//     path.emplace_back(vertices_map_[*p]->state);
//   }
// }

void GraphUtils::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<int>& path) {
  path.clear();
  if (!rep.status) {
    return;
  }

  if (rep.parent_id_map.size() <= target_id) {
    return;
  }

  if (target_id == rep.source_id) {
    path.push_back(target_id);
    return;
  }

  int parent_id = rep.parent_id_map.at(target_id);
  if (parent_id == target_id) {
    return;
  }

  path.push_back(target_id);  // current vertex id first
  path.push_back(
      parent_id);  // its first parent, the rest is recursively looked up
  while (parent_id != rep.source_id) {
    parent_id = rep.parent_id_map.at(parent_id);
    path.push_back(parent_id);
  }

  // Initially, the path follows target to source order. Reverse if required.
  if (source_to_target_order) {
    std::reverse(path.begin(), path.end());
  }
}

double GraphUtils::getShortestDistance(int target_id,
                                         const ShortestPathsReport& rep) {
  double dist = std::numeric_limits<double>::max();

  if (!rep.status) {
    return dist;
  }
  if (rep.parent_id_map.size() <= target_id) {
    return dist;
  }
  dist = rep.distance_map.at(target_id);
  return dist;
}

int GraphUtils::getParentIDFromShortestPath(int target_id,
                                              const ShortestPathsReport& rep) {
  if (!rep.status) {
    return target_id;
  }

  if (rep.parent_id_map.size() <= target_id) {
    return target_id;
  }

  return rep.parent_id_map.at(target_id);
}



int GraphUtils::generateSubgraphIndex() { return ++subgraph_ind_; }

int GraphUtils::generateVertexID() { 
  return ++id_count_;
}



