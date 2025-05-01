#pragma once

#include <any>

#include "graph/graph.hpp"
#include "common/utils.hpp"
#include "predictive_planner_common/Graph.h"

#include "visualization_msgs/MarkerArray.h"

#define NP "NP"

enum struct VertexStatus {
  kTrue = 0,  // Detected by segmentation module
  kEnhanced = 1,  // Enhanced
  kPotentialExtension = 2,  // Can be extended into a pattern
  kPredicted = 3  // Predicted
};

class SemanticVertex
{
public:
  int id;  // ID of the semantic in the graph
  int local_id;  // ID provided by the detector
  Eigen::Vector6d state; // x,y,z,R,P,Y
  Eigen::Vector3d bbox;  // Dims along local x,y,z. Needs to be rotated by RPY, in state to get the global orientation
  int label;
  std::map<int, int> neighbor_map;  // neighbor vertex id -> edge id
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  int num_detections = 1;
  int associated_compartment_vertex_id = -1;
  bool seen = true;
  bool newly_added = true;
  std::string pattern_name = NP;
  int instance_ind;
  VertexStatus status = VertexStatus::kTrue;
  std::any metadata;
  bool used = false;

  SemanticVertex() {}
  SemanticVertex(SemanticVertex &_v)
  {
    id = _v.id;
    local_id = _v.local_id;
    state = _v.state;
    label = _v.label;
    neighbor_map = _v.neighbor_map;
    // cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*(_v.cloud)));
    num_detections = _v.num_detections;
    associated_compartment_vertex_id = _v.associated_compartment_vertex_id;
    seen = _v.seen;
    newly_added = _v.newly_added;
    pattern_name = _v.pattern_name;
    instance_ind = _v.instance_ind;
    status = _v.status;
    metadata = _v.metadata;
    used = _v.used;
    bbox = _v.bbox;
  }

  std::shared_ptr<SemanticVertex> copy(bool with_id)
  {
    std::shared_ptr<SemanticVertex> new_vertex;
    new_vertex.reset(new SemanticVertex);
    
    if(with_id)
      new_vertex->id = id;
    new_vertex->local_id = local_id;
    new_vertex->state = state;
    new_vertex->label = label;
    new_vertex->neighbor_map = neighbor_map;
    new_vertex->num_detections = num_detections;
    new_vertex->associated_compartment_vertex_id = associated_compartment_vertex_id;
    new_vertex->seen = seen;
    new_vertex->newly_added = newly_added;
    new_vertex->pattern_name = pattern_name;
    new_vertex->instance_ind = instance_ind;
    new_vertex->status = status;
    new_vertex->metadata = metadata;
    new_vertex->used = used;
    new_vertex->bbox = bbox;

    return new_vertex;
  }

  bool copyTo(std::shared_ptr<SemanticVertex> copy_vertex, bool with_id)
  {
    if(copy_vertex == nullptr)
      return false;

    if(with_id)
      copy_vertex->id = id;
    copy_vertex->local_id = local_id;
    copy_vertex->state = state;
    copy_vertex->label = label;
    copy_vertex->neighbor_map = neighbor_map;
    copy_vertex->num_detections = num_detections;
    copy_vertex->associated_compartment_vertex_id = associated_compartment_vertex_id;
    copy_vertex->seen = seen;
    copy_vertex->newly_added = newly_added;
    copy_vertex->pattern_name = pattern_name;
    copy_vertex->instance_ind = instance_ind;
    copy_vertex->status = status;
    copy_vertex->metadata = metadata;
    copy_vertex->used = used;
    copy_vertex->bbox = bbox;

    return true;
  }
};

struct EdgeRelation
{
  EdgeRelation(EdgeRelation &_e)
  {
    id = _e.id;
    label = _e.label;
    weight = _e.label;
    source_vertex.reset(new SemanticVertex(*(_e.source_vertex)));
    target_vertex.reset(new SemanticVertex(*(_e.target_vertex)));
  }
  EdgeRelation() {}
  int id;
  std::shared_ptr<SemanticVertex> source_vertex;
  std::shared_ptr<SemanticVertex> target_vertex;
  int label;
  double weight;

  bool copyTo(std::shared_ptr<EdgeRelation> edge_copy, bool with_id)
  {
    if(edge_copy == nullptr)
      return false;
    
    if(with_id)
      edge_copy->id = id;
    
    edge_copy->source_vertex.reset(new SemanticVertex);
    source_vertex->copyTo(edge_copy->source_vertex, true);
    edge_copy->target_vertex.reset(new SemanticVertex);
    target_vertex->copyTo(edge_copy->target_vertex, true);

    edge_copy->weight = weight;
    edge_copy->label = label;
  }
};


class SSGManager
{
public:
  SSGManager();
  SSGManager(SSGManager &_ssg);
  std::shared_ptr<SemanticVertex> initializeNewVertex();
  int getNextVertexID();
  void addVertex(std::shared_ptr<SemanticVertex> v);
  std::shared_ptr<SemanticVertex> addNewVertex();
  bool removeVertex(std::shared_ptr<SemanticVertex> v);

  void addEdge(std::shared_ptr<EdgeRelation> e);
  bool removeEdge(std::shared_ptr<EdgeRelation> e);
  std::shared_ptr<EdgeRelation> initializeNewEdge();
  // bool removeEdge(std::shared_ptr<SemanticVertex> v1, std::shared_ptr<SemanticVertex> v2);

  // void updateVertex(std::shared_ptr<SemanticVertex> v, std::shared_ptr<SemanticVertex> v_new);

  void insertGraph(std::shared_ptr<SSGManager> in_graph);
  void insertGraph(std::shared_ptr<SSGManager> in_graph, VertexStatus fixed_status);

  bool getNearestVerticesInRange(std::shared_ptr<SemanticVertex> v, std::vector<std::shared_ptr<SemanticVertex>> &nearest_vertices, double range);
  bool getNearestVerticesInRange(Eigen::Vector6d s, std::vector<std::shared_ptr<SemanticVertex>> &nearest_vertices, double range);

  std::shared_ptr<SemanticVertex> getVertex(int id) { return vertices_map_[id]; };
  std::shared_ptr<EdgeRelation> getEdge(int id) { return edge_map_[id]; };

  visualization_msgs::MarkerArray getGraphVis();
  predictive_planner_common::Graph getGraphMsg();
  
  void saveGraph(std::string filepath);
  void loadGraph(std::string filepath);
  void convertMsgToGraph(predictive_planner_common::Graph graph_msg);
  std::shared_ptr<SSGManager> createCopy();

  std::unordered_map<int, std::shared_ptr<SemanticVertex>> vertices_map_;
  std::unordered_map<int, std::shared_ptr<EdgeRelation>> edge_map_;
  std::unordered_map<int, int> ssg_to_subdue_id_map_;
  
  int id_count_ = 0;
  int edge_id_count_ = 0;

  std::shared_ptr<BaseGraph> ssg_;
private:

};