#pragma once

#include "common/communicator.hpp"
#include "graph/ssg_manager.hpp"
#include "bwt_ssg_builder/base_structures.hpp"

#include "planner_manager.hpp"

#include "subdue.h"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

struct PatternInstance
{
  int inst_id;
  std::vector<int> vertex_ids;
  std::vector<int> edge_ids;
};

struct PatternStructure
{
  std::string name;
  std::shared_ptr<SSGManager> definition;
  std::vector<PatternInstance> instances;
  int level;

  PatternStructure() {}

  PatternStructure(PatternStructure &_ps)
  {
    name = _ps.name;
    level = _ps.level;
    instances = _ps.instances;
    definition = _ps.definition->createCopy();
  }

  bool transformWRTVertex(int v_id, std::shared_ptr<SSGManager> base_graph);
  void transformWRTState(const Eigen::Vector6d root_state);

  void attachOnState(const Eigen::Vector6d root_state);
  bool attachOnVertex(int v_id, std::shared_ptr<SSGManager> base_graph);
};

struct SSGLevel
{
  std::shared_ptr<SSGManager> base_graph;
  std::shared_ptr<SSGManager> enhanced_base_graph;
  std::shared_ptr<SSGManager> compressed_graph;
  std::set<std::string> discovered_pattern_names;
  std::set<std::string> enhanced_pattern_names;
  std::map<int, int> vertex_subdue_indx_To_ssg_id;
  std::map<int, int> edge_subdue_indx_To_ssg_id;
};

struct SSGHierarchy
{
  std::vector<SSGLevel> levels;
};

class PredictivePlanning
{
private:
	std::shared_ptr<Communicator> comm_;
	std::shared_ptr<Config> config_;

  ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

  ros::ServiceServer find_patterns_srv_;
  ros::ServiceServer insert_vertices_srv_;
  ros::ServiceServer extend_graph_srv_;
  ros::ServiceServer set_thr_srv_;
  ros::ServiceServer save_graph_srv_;
  ros::ServiceServer load_graph_srv_;
  ros::ServiceServer test_graph_match_srv_;
  
  ros::Publisher compressed_graph_pub_;
  ros::Publisher hierarchy_vis_pub_;
  ros::Publisher ssg_vis_pub_;
  ros::Publisher predicted_subgraph_vis_pub_;

  ros::Timer pattern_detection_timer_;

  Parameters *parameters_;

  std::vector<std::shared_ptr<SSGManager>> compressed_graphs_ssg_;
  std::map<std::string, std::shared_ptr<PatternStructure>> pattern_structs_;  // pattern name -> pattern struct
  std::map<int, int> vertex_subdue_indx_To_ssg_id_;
  std::map<int, int> edge_subdue_indx_To_ssg_id_;
  std::vector<int> entry_classes;
  std::set<std::string> base_labels;

  std::shared_ptr<SSGHierarchy> current_hierarchy_;
  std::map<std::string, int> label_string_to_int_;
  std::map<int, std::string> label_int_to_string_;
  int substruct_label_int_;

public:
  PredictivePlanning(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, std::shared_ptr<Communicator> comm, std::shared_ptr<Config> config);
  void initializeAttributes();
  
  void setSubdueParams();
  void setNumIterations(int num_iters);
  void findPatterns();
  void findPatternsTimerCallback(const ros::TimerEvent& event);
  bool findPatternsCb(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool insertverticesCb(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool extendGraphCb(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool saveSSGCb(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool loadSSGCb(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool testGraphMatchCb(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);

  /// Prediction
  void insertEntryVertices();
  void extendGraphToStructs();

  Graph* ssgToSubdue(std::shared_ptr<SSGManager> ssg, Parameters *parameters, SSGLevel &ssg_level);
  std::shared_ptr<SSGManager> subdueToSSG(Graph* subdue_graph, LabelList *label_list,
                                          std::map<int, int> &vertex_subdue_indx_To_ssg_id, std::map<int, int> &edge_subdue_indx_To_ssg_id);
  std::shared_ptr<SSGManager> convertSubdueCompressedGraph(Graph* subdue_graph, LabelList *label_list, std::shared_ptr<SSGManager> base_graph,
                                          std::map<int, int> &vertex_subdue_indx_To_ssg_id, std::map<int, int> &edge_subdue_indx_To_ssg_id);
  std::shared_ptr<PatternStructure> subdueStructToPatternStruct(Substructure *subdue_stuct, LabelList *label_list,
                                        std::map<int, int> vertex_subdue_indx_To_ssg_id, std::map<int, int> edge_subdue_indx_To_ssg_id);
  
  std::shared_ptr<SSGManager> getDefinitionFromInstance(PatternInstance inst, std::shared_ptr<SSGManager> base_graph);
  int getIntLabel(std::string string_label);
  std::shared_ptr<SSGManager> getPredictedSubgraph(); 
  
  void offsetGraphVis(visualization_msgs::MarkerArray &graph_vis, Eigen::Vector3d offset);
  void visualizeHierarchy(SSGHierarchy h, bool enhanced);
};