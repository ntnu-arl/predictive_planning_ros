#include "predictive_planning.hpp"

bool PatternStructure::transformWRTVertex(int v_id, std::shared_ptr<SSGManager> base_graph)
{
  if (base_graph->getVertex(v_id) == NULL)
  {
    ROS_ERROR("Vertex %d is not in the graph", v_id);
    return false;
  }

  Eigen::Vector6d root_state = base_graph->getVertex(v_id)->state;
  transformWRTState(root_state);

  return true;
}

void PatternStructure::transformWRTState(const Eigen::Vector6d root_state)
{
  geometry_msgs::Pose root_pose;
  geometry_msgs::TransformStamped root_tfs, root_tfs_inv;
  convert(root_state, root_pose);
  convert(root_pose, root_tfs.transform);
  tf::Transform root_tf_tf;
  tf::transformMsgToTF(root_tfs.transform, root_tf_tf);
  tf::transformTFToMsg(root_tf_tf.inverse(), root_tfs_inv.transform);

  for (auto v_it : definition->vertices_map_)
  {
    transformState(v_it.second->state, v_it.second->state, root_tfs_inv);
  }
}

void PatternStructure::attachOnState(const Eigen::Vector6d root_state)
{
  geometry_msgs::Pose root_pose;
  geometry_msgs::TransformStamped root_tfs;
  convert(root_state, root_pose);
  convert(root_pose, root_tfs.transform);

  for (auto v_it : definition->vertices_map_)
  {
    transformState(v_it.second->state, v_it.second->state, root_tfs);
  }
}

bool PatternStructure::attachOnVertex(int v_id, std::shared_ptr<SSGManager> base_graph)
{
  if (base_graph->getVertex(v_id) == NULL)
  {
    ROS_ERROR("Vertex %d is not in the graph", v_id);
    return false;
  }

  Eigen::Vector6d root_state = base_graph->getVertex(v_id)->state;
  attachOnState(root_state);

  return true;
}

PredictivePlanning::PredictivePlanning(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, std::shared_ptr<Communicator> comm, std::shared_ptr<Config> config)
    : nh_(nh), nh_private_(nh_private), comm_(comm), config_(config)
{
  find_patterns_srv_ = nh_.advertiseService(
      "find_patterns", &PredictivePlanning::findPatternsCb, this);
  insert_vertices_srv_ = nh_.advertiseService(
      "insert_vertices", &PredictivePlanning::insertverticesCb, this);
  extend_graph_srv_ = nh_.advertiseService(
      "extend_graph", &PredictivePlanning::extendGraphCb, this);
  save_graph_srv_ = nh_.advertiseService(
      "save_ssg", &PredictivePlanning::saveSSGCb, this);
  load_graph_srv_ = nh_.advertiseService(
      "load_ssg", &PredictivePlanning::loadSSGCb, this);
  test_graph_match_srv_ = nh_.advertiseService(
      "test_graph_match", &PredictivePlanning::testGraphMatchCb, this);

  compressed_graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("compressed_graphs", 1);
  hierarchy_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("hierarchy_vis", 1);
  predicted_subgraph_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("predicted_subgraph_vis", 1);
  if (!config_->system_params.build_bwt_ssg)
    ssg_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("ssg_vis", 1);
  // pattern_detection_timer_ =
  //         nh_.createTimer(ros::Duration(30.0),
  //                         &PredictivePlanning::findPatternsTimerCallback, this);
  initializeAttributes();

  substruct_label_int_ = config_->ssg_params.substruct_label_start;
}

void PredictivePlanning::initializeAttributes()
{
  entry_classes.push_back(L_MANHOLE);

  label_string_to_int_[std::to_string(L_ANODE)] = L_ANODE;
  label_string_to_int_[std::to_string(L_STIFF)] = L_STIFF;
  label_string_to_int_[std::to_string(L_BT)] = L_BT;
  label_string_to_int_[std::to_string(L_LONG)] = L_LONG;
  label_string_to_int_[std::to_string(L_WALL)] = L_WALL;
  label_string_to_int_[std::to_string(L_MANHOLE)] = L_MANHOLE;
  label_string_to_int_[std::to_string(L_COMPARTMENT)] = L_COMPARTMENT;
}

void PredictivePlanning::setSubdueParams()
{
  parameters_ = initializeNewParameters();

  parameters_->predefinedSubs = config_->subdue_params.predefinedSubs;
  parameters_->outputToFile = config_->subdue_params.outputToFile;
  parameters_->directed = config_->subdue_params.directed;
  parameters_->valueBased = config_->subdue_params.valueBased;
  parameters_->allowInstanceOverlap = config_->subdue_params.allowInstanceOverlap;
  parameters_->recursion = config_->subdue_params.recursion;
  parameters_->variables = config_->subdue_params.variables;
  parameters_->relations = config_->subdue_params.relations;
  if (parameters_->relations)
    parameters_->variables = TRUE;
  parameters_->incremental = config_->subdue_params.incremental;
  parameters_->compress = config_->subdue_params.compress;
  parameters_->prune = config_->subdue_params.prune;

  parameters_->beamWidth = (ULONG)config_->subdue_params.beamWidth;
  parameters_->limit = (ULONG)config_->subdue_params.limit;
  parameters_->maxVertices = (ULONG)config_->subdue_params.maxVertices;
  parameters_->minVertices = (ULONG)config_->subdue_params.minVertices;
  parameters_->numBestSubs = (ULONG)config_->subdue_params.numBestSubs;
  parameters_->outputLevel = (ULONG)config_->subdue_params.outputLevel;
  parameters_->evalMethod = (ULONG)config_->subdue_params.evalMethod;
  parameters_->iterations = (ULONG)config_->subdue_params.iterations;
  parameters_->threshold = config_->subdue_params.threshold;
  parameters_->use_pose_cost = config_->subdue_params.use_pose_cost;
  parameters_->normalize_value_with_cost = config_->subdue_params.normalize_value_with_cost;

  std::copy(config_->subdue_params.outputFileName.begin(),
            config_->subdue_params.outputFileName.end(), parameters_->outFileName);
  std::copy(config_->subdue_params.psInputFileName.begin(),
            config_->subdue_params.psInputFileName.end(), parameters_->psInputFileName);

  parameters_ = processParameters(parameters_);
}

Graph *PredictivePlanning::ssgToSubdue(std::shared_ptr<SSGManager> ssg, Parameters *parameters, SSGLevel &ssg_level)
{
  Graph *graph;
  Vertex *newVertexList;
  ULONG *posEgsVertexIndices = NULL;
  ULONG numPosEgs = 0;
  // LabelList *labelList;
  // labelList = AllocateLabelList();
  // std::cout << "Label list allocated" << std::endl;

  numPosEgs++;
  posEgsVertexIndices = AddVertexIndex(posEgsVertexIndices, numPosEgs, 0);

  graph = AllocateGraph(0, 0);
  // std::cout << "Graph allocated" << std::endl;

  /// Add vertices
  for (auto v_it : ssg->vertices_map_)
  {
    /// Increase the vertex list
    ULONG numVertices = graph->numVertices;
    ULONG vertexListSize = graph->vertexListSize;
    vertexListSize += LIST_SIZE_INC;
    newVertexList = (Vertex *)realloc(graph->vertices, (sizeof(Vertex) * (vertexListSize)));
    if (newVertexList == NULL)
      OutOfMemoryError("vertex list");
    graph->vertices = newVertexList;
    graph->vertexListSize = vertexListSize;
    // std::cout << "Vertex list re-allocated" << std::endl;

    /// Set the label
    std::string label_str = std::to_string(v_it.second->label);
    Label label;
    label.labelType = STRING_LABEL;
    // std::copy(label_str.begin(),
    //         label_str.end(), label.labelValue.stringLabel);
    label.labelValue.stringLabel = label_str.data();
    label.level = 0;
    // if(v_it.second->label == L_LONG)
    //   label.imp = 1;
    // else
    //   label.imp = 0;
    label.imp = 1;
    ULONG label_index = StoreLabel(&label, parameters->labelList);
    // std::cout << "Label set" << std::endl;

    /// store information in vertex
    graph->vertices[numVertices].label = label_index;
    graph->vertices[numVertices].numEdges = 0;
    graph->vertices[numVertices].edges = NULL;
    graph->vertices[numVertices].map = VERTEX_UNMAPPED;
    graph->vertices[numVertices].used = FALSE;
    graph->vertices[numVertices].degree = v_it.second->neighbor_map.size();
    // std::cout << "Vertex properties set" << std::endl;
    graph->vertices[numVertices].position = gsl_vector_alloc(3);
    gsl_vector_set(graph->vertices[numVertices].position, 0, v_it.second->state.x());
    gsl_vector_set(graph->vertices[numVertices].position, 1, v_it.second->state.y());
    gsl_vector_set(graph->vertices[numVertices].position, 2, v_it.second->state.z());
    graph->vertices[numVertices].orientation = gsl_vector_alloc(3);
    gsl_vector_set(graph->vertices[numVertices].orientation, 0, v_it.second->state(3));
    gsl_vector_set(graph->vertices[numVertices].orientation, 1, v_it.second->state(4));
    gsl_vector_set(graph->vertices[numVertices].orientation, 2, v_it.second->state(5));
    // std::cout << "Vertex position set" << std::endl;
    graph->vertices[numVertices].in_pattern = FALSE;
    ssg->ssg_to_subdue_id_map_[v_it.first] = numVertices;
    ssg_level.vertex_subdue_indx_To_ssg_id[numVertices] = v_it.first;
    ++graph->numVertices;
    // std::cout << "Vertex fully set" << std::endl;
    // std::cout << "Vertex: ID: " << id << " label: " << label_str << " position: ";
    // printVector(graph->vertices[numVertices].position, 3);
    // std::cout << "\n";
  }
  // std::cout << "Vertices set" << std::endl;

  /// Add edges
  ULONG sourceVertexIndex;
  ULONG targetVertexIndex;
  for (auto e_it : ssg->edge_map_)
  {
    sourceVertexIndex = ssg->ssg_to_subdue_id_map_[e_it.second->source_vertex->id];
    targetVertexIndex = ssg->ssg_to_subdue_id_map_[e_it.second->target_vertex->id];

    Label label;
    label.labelType = STRING_LABEL;
    std::string label_str = std::to_string(e_it.second->label);
    // std::copy(label_str.begin(),
    //         label_str.end(), label.labelValue.stringLabel);
    label.labelValue.stringLabel = label_str.data();
    ULONG label_index = StoreLabel(&label, parameters->labelList);
    // printf("Edge label: %s\n", parameters->labelList->labels[label_index].labelValue);

    Edge *newEdgeList;
    ULONG edgeListSize = graph->edgeListSize;

    // make sure there is enough room for another edge in the graph
    if (edgeListSize == graph->numEdges)
    {
      edgeListSize += LIST_SIZE_INC;
      newEdgeList = (Edge *)realloc(graph->edges, (sizeof(Edge) * (edgeListSize)));
      if (newEdgeList == NULL)
        OutOfMemoryError("AddEdge:newEdgeList");
      graph->edges = newEdgeList;
      graph->edgeListSize = edgeListSize;
    }

    // add edge to graph
    graph->edges[graph->numEdges].vertex1 = sourceVertexIndex;
    graph->edges[graph->numEdges].vertex2 = targetVertexIndex;
    graph->edges[graph->numEdges].label = label_index;
    graph->edges[graph->numEdges].directed = FALSE;
    graph->edges[graph->numEdges].used = FALSE;
    graph->edges[graph->numEdges].spansIncrement = FALSE;
    graph->edges[graph->numEdges].validPath = TRUE;

    ssg_level.edge_subdue_indx_To_ssg_id[graph->numEdges] = e_it.first;

    AddEdgeToVertices(graph, graph->numEdges);

    ++graph->numEdges;
  }
  // std::cout << "Edges set" << std::endl;

  parameters->numPosEgs = numPosEgs;
  parameters->posEgsVertexIndices = posEgsVertexIndices;

  return graph;
}

std::shared_ptr<SSGManager> PredictivePlanning::subdueToSSG(Graph *subdue_graph, LabelList *label_list,
                                                            std::map<int, int> &vertex_subdue_indx_To_ssg_id, std::map<int, int> &edge_subdue_indx_To_ssg_id)
{
  // std::cout << "Inside subdueToSSG" << std::endl;
  std::shared_ptr<SSGManager> ssg;
  // std::cout << "Graph initialized" << std::endl;
  ssg.reset(new SSGManager());
  // std::cout << "Graph set" << std::endl;
  // if(ssg == NULL) std::cout << "ssg null" << std::endl;
  // if(subdue_graph == NULL) std::cout << "subdue_graph null" << std::endl;
  // if(label_list == NULL) std::cout << "label_list null" << std::endl;

  for (ULONG v = 0; v < subdue_graph->numVertices; ++v)
  {
    std::shared_ptr<SemanticVertex> new_vertex = ssg->initializeNewVertex();
    // std::cout << "0" << std::endl;
    // auto result = std::find_if(
    //             ith_level.base_graph->edge_map_.begin(),
    //             ith_level.base_graph->edge_map_.end(),
    //             [e_id](const auto& mo) {return mo.first == e_id; });
    // std::string val = label_list->labels[subdue_graph->vertices[v].label].labelValue.stringLabel;
    // auto result = std::find_if
    //               (base_labels.begin(), base_labels.end(),
    //               [val] (const std::string &s) {return (s.compare(val) != 0); });
    // // if(label_list->labels[subdue_graph->vertices[v].label].level > 0)
    // if(result == base_labels.end())
    // {
    //   new_vertex->pattern_name = label_list->labels[subdue_graph->vertices[v].label].labelValue.stringLabel;
    // }
    // std::cout << "1" << std::endl;
    int int_label = getIntLabel(label_list->labels[subdue_graph->vertices[v].label].labelValue.stringLabel);
    if (int_label >= config_->ssg_params.substruct_label_start)
    {
      new_vertex->pattern_name = label_list->labels[subdue_graph->vertices[v].label].labelValue.stringLabel;
    }
    // std::cout << "Got int label" << std::endl;
    new_vertex->label = int_label;
    new_vertex->state << gsl_vector_get(subdue_graph->vertices[v].position, 0),
        gsl_vector_get(subdue_graph->vertices[v].position, 1),
        gsl_vector_get(subdue_graph->vertices[v].position, 2),
        gsl_vector_get(subdue_graph->vertices[v].orientation, 0),
        gsl_vector_get(subdue_graph->vertices[v].orientation, 1),
        gsl_vector_get(subdue_graph->vertices[v].orientation, 2);
    // new_vertex->state << gsl_vector_get(subdue_graph->vertices[v].position, 0),
    //                      gsl_vector_get(subdue_graph->vertices[v].position, 1),
    //                      gsl_vector_get(subdue_graph->vertices[v].position, 2),
    //                      0.0,
    //                      0.0,
    //                      0.0;
    ssg->addVertex(new_vertex);
    vertex_subdue_indx_To_ssg_id[v] = new_vertex->id;
    // std::cout << "new vertex added: " << new_vertex->id << std::endl;
  }
  // std::cout << "Vertices added" << std::endl;

  for (ULONG e = 0; e < subdue_graph->numEdges; ++e)
  {
    std::shared_ptr<EdgeRelation> new_edge = ssg->initializeNewEdge();
    new_edge->label = 0;
    // std::cout << e << ":";
    // std::cout << subdue_graph->edges[e].vertex1 << "<->" << subdue_graph->edges[e].vertex2 << std::endl;
    new_edge->source_vertex = ssg->getVertex(subdue_graph->edges[e].vertex1);
    // std::cout << "source vertex found " << subdue_graph->edges[e].vertex1 << std::endl;
    new_edge->target_vertex = ssg->getVertex(subdue_graph->edges[e].vertex2);
    // std::cout << "target vertex found " << subdue_graph->edges[e].vertex2 << std::endl;
    // std::cout << "Adding edge between " << new_edge->source_vertex->id << "<->" << new_edge->target_vertex->id << std::endl;
    new_edge->weight = 1.0;
    ssg->addEdge(new_edge);
    edge_subdue_indx_To_ssg_id[e] = new_edge->id;
    // std::cout << "new edge added" << std::endl;
  }
  // std::cout << "Edges added" << std::endl;

  return ssg;
}

std::shared_ptr<SSGManager> PredictivePlanning::convertSubdueCompressedGraph(Graph* subdue_graph, LabelList *label_list, std::shared_ptr<SSGManager> base_graph,
                                          std::map<int, int> &vertex_subdue_indx_To_ssg_id, std::map<int, int> &edge_subdue_indx_To_ssg_id)
{
  std::shared_ptr<SSGManager> ssg;

  ssg.reset(new SSGManager());
  // std::cout << "Graph set" << std::endl;
  // if(ssg == NULL) std::cout << "ssg null" << std::endl;
  // if(subdue_graph == NULL) std::cout << "subdue_graph null" << std::endl;
  // if(label_list == NULL) std::cout << "label_list null" << std::endl;

  std::map<int, int> og_vertex_subdue_indx_To_ssg_id = vertex_subdue_indx_To_ssg_id;
  std::map<int, int> og_edge_subdue_indx_To_ssg_id = edge_subdue_indx_To_ssg_id;

  vertex_subdue_indx_To_ssg_id.clear();
  edge_subdue_indx_To_ssg_id.clear();

  for (ULONG v = 0; v < subdue_graph->numVertices; ++v)
  {
    std::shared_ptr<SemanticVertex> new_vertex = ssg->initializeNewVertex();
    // std::cout << "0" << std::endl;
    // auto result = std::find_if(
    //             ith_level.base_graph->edge_map_.begin(),
    //             ith_level.base_graph->edge_map_.end(),
    //             [e_id](const auto& mo) {return mo.first == e_id; });
    // std::string val = label_list->labels[subdue_graph->vertices[v].label].labelValue.stringLabel;
    // auto result = std::find_if
    //               (base_labels.begin(), base_labels.end(),
    //               [val] (const std::string &s) {return (s.compare(val) != 0); });
    // // if(label_list->labels[subdue_graph->vertices[v].label].level > 0)
    // if(result == base_labels.end())
    // {
    //   new_vertex->pattern_name = label_list->labels[subdue_graph->vertices[v].label].labelValue.stringLabel;
    // }
    // std::cout << "1" << std::endl;
    int int_label = getIntLabel(label_list->labels[subdue_graph->vertices[v].label].labelValue.stringLabel);
    if (int_label >= config_->ssg_params.substruct_label_start)
    {
      new_vertex->pattern_name = label_list->labels[subdue_graph->vertices[v].label].labelValue.stringLabel;
    }
    // std::cout << "Got int label" << std::endl;
    new_vertex->label = int_label;
    new_vertex->state << gsl_vector_get(subdue_graph->vertices[v].position, 0),
        gsl_vector_get(subdue_graph->vertices[v].position, 1),
        gsl_vector_get(subdue_graph->vertices[v].position, 2),
        gsl_vector_get(subdue_graph->vertices[v].orientation, 0),
        gsl_vector_get(subdue_graph->vertices[v].orientation, 1),
        gsl_vector_get(subdue_graph->vertices[v].orientation, 2);
    // new_vertex->state << gsl_vector_get(subdue_graph->vertices[v].position, 0),
    //                      gsl_vector_get(subdue_graph->vertices[v].position, 1),
    //                      gsl_vector_get(subdue_graph->vertices[v].position, 2),
    //                      0.0,
    //                      0.0,
    //                      0.0;
    new_vertex->bbox = base_graph->getVertex(og_vertex_subdue_indx_To_ssg_id[subdue_graph->vertices[v].map])->bbox;
    // std::cout << "vertex " << new_vertex->id << " label: " << new_vertex->label 
    //           << " | Maped: subdue: " << subdue_graph->vertices[v].map << " ssg: " <<  og_vertex_subdue_indx_To_ssg_id[subdue_graph->vertices[v].map]
    //           << " label: " << base_graph->getVertex(og_vertex_subdue_indx_To_ssg_id[subdue_graph->vertices[v].map])->label
    //           <<  " bbox: " << new_vertex->bbox.transpose() << std::endl;
    ssg->addVertex(new_vertex);
    vertex_subdue_indx_To_ssg_id[v] = new_vertex->id;
    // std::cout << "new vertex added: " << new_vertex->id << std::endl;
  }
  // std::cout << "Vertices added" << std::endl;

  for (ULONG e = 0; e < subdue_graph->numEdges; ++e)
  {
    std::shared_ptr<EdgeRelation> new_edge = ssg->initializeNewEdge();
    new_edge->label = 0;
    // std::cout << e << ":";
    // std::cout << subdue_graph->edges[e].vertex1 << "<->" << subdue_graph->edges[e].vertex2 << std::endl;
    new_edge->source_vertex = ssg->getVertex(subdue_graph->edges[e].vertex1);
    // std::cout << "source vertex found " << subdue_graph->edges[e].vertex1 << std::endl;
    new_edge->target_vertex = ssg->getVertex(subdue_graph->edges[e].vertex2);
    // std::cout << "target vertex found " << subdue_graph->edges[e].vertex2 << std::endl;
    // std::cout << "Adding edge between " << new_edge->source_vertex->id << "<->" << new_edge->target_vertex->id << std::endl;
    new_edge->weight = 1.0;
    ssg->addEdge(new_edge);
    edge_subdue_indx_To_ssg_id[e] = new_edge->id;
    // std::cout << "new edge added" << std::endl;
  }
  // std::cout << "Edges added" << std::endl;

  return ssg;
}

std::shared_ptr<PatternStructure> PredictivePlanning::subdueStructToPatternStruct
  (Substructure *subdue_stuct, LabelList *label_list, std::map<int, int> vertex_subdue_indx_To_ssg_id, std::map<int, int> edge_subdue_indx_To_ssg_id)
{
  std::shared_ptr<PatternStructure> pattern_struct;
  pattern_struct.reset(new PatternStructure);
  // std::cout << "Label list: [";
  // for(int i=0; i<label_list->numLabels; ++i)
  // {
  //   std::cout << label_list->labels[i].labelValue.stringLabel << ", ";
  // }
  // std::cout << "]" << std::endl;
  PrintGraph(subdue_stuct->definition, label_list);

  // std::cout << "Substruct definition converted" << std::endl;
  InstanceListNode *inst_node = subdue_stuct->instances->head;

  int i = 0;
  while (inst_node != NULL)
  {
    // std::cout << "Instance no " << i << std::endl;
    PatternInstance p_inst;
    p_inst.inst_id = i;
    for (int v_ind = 0; v_ind < inst_node->instance->numVertices; ++v_ind)
    {
      p_inst.vertex_ids.push_back(vertex_subdue_indx_To_ssg_id[inst_node->instance->vertices[v_ind]]);
    }
    // std::cout << "Vertices converted" << std::endl;

    for (int e_ind = 0; e_ind < inst_node->instance->numEdges; ++e_ind)
    {
      p_inst.edge_ids.push_back(edge_subdue_indx_To_ssg_id[inst_node->instance->edges[e_ind]]);
    }
    // std::cout << "Edges converted" << std::endl;
    // p_inst.vertex_ids = std::vector<int>(inst_node->instance->vertices,
    //       inst_node->instance->vertices + sizeof(inst_node->instance->vertices)/sizeof(inst_node->instance->vertices[0]));
    // p_inst.edge_ids = std::vector<int>(inst_node->instance->edges,
    //       inst_node->instance->edges + sizeof(inst_node->instance->edges)/sizeof(inst_node->instance->edges[0]));
    pattern_struct->instances.push_back(p_inst);
    // std::cout << "Added instance" << std::endl;
    inst_node = inst_node->next;
    // std::cout << "Updated iterator" << std::endl;
    ++i;
  }

  std::map<int, int> temp_vertex_subdue_indx_To_ssg_id;
  std::map<int, int> temp_edge_subdue_indx_To_ssg_id;
  pattern_struct->definition = subdueToSSG(subdue_stuct->definition, label_list, temp_vertex_subdue_indx_To_ssg_id, temp_edge_subdue_indx_To_ssg_id);
  // std::cout << "All instances converted" << std::endl;
  pattern_struct->level = subdue_stuct->level;
  // std::cout << "Added level" << std::endl;
  pattern_struct->name = std::string(subdue_stuct->label_str);
  // std::cout << "Name of the level " << pattern_struct->name << " / " << subdue_stuct->label_str << std::endl;

  // std::cout << "Substruct converted" << std::endl;
  return pattern_struct;
}

bool PredictivePlanning::testGraphMatchCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  setSubdueParams();
  // PostProcessParameters(parameters_);

  int g_correct_v = 6, g_correct_e = 5;
  int g_missing_v = 5, g_missing_e = 4;
  int g_wrong_v = 6, g_wrong_e = 5;
  int g_extra_v = 7, g_extra_e = 6;
  int g_correct_copy_v = 6, g_correct_copy_e = 5;

  Graph *g_correct = AllocateGraph(6, 5);
  Graph *g_missing = AllocateGraph(5, 4);
  Graph *g_wrong = AllocateGraph(6, 5);
  Graph *g_extra = AllocateGraph(g_extra_v, g_extra_e);
  Graph *g_correct_copy = AllocateGraph(6, 5);

  std::string label_str;

  std::cout << "Graphs allocated" << std::endl;

  /// g_correct ///
  {
    Label label;
    label.labelType = STRING_LABEL;
    label_str = std::to_string(L_WALL).data();
    label.labelValue.stringLabel = label_str.data();
    ULONG label_index = StoreLabel(&label, parameters_->labelList);
    g_correct->vertices[0].label = label_index;
    g_correct->vertices[0].map = VERTEX_UNMAPPED;
    g_correct->vertices[0].used = FALSE;
    g_correct->vertices[0].numEdges = 0;
    g_correct->vertices[0].edges = NULL;
    g_correct->vertices[0].position = gsl_vector_alloc(3);
    // -4.335 1.45221 1.53744
    gsl_vector_set(g_correct->vertices[0].position, 0, -0.102303);
    gsl_vector_set(g_correct->vertices[0].position, 1, 1.70546);
    gsl_vector_set(g_correct->vertices[0].position, 2, 2.55189);
    g_correct->vertices[0].orientation = gsl_vector_alloc(3);
    gsl_vector_set(g_correct->vertices[0].orientation, 0, 1.0);
    gsl_vector_set(g_correct->vertices[0].orientation, 1, 0.0);
    gsl_vector_set(g_correct->vertices[0].orientation, 2, 0.0);
    g_correct->vertices[0].in_pattern = FALSE;
    std::vector<Eigen::Vector3d> positions;
    positions.push_back(Eigen::Vector3d(-0.0832978, 1.46434, 4.22118));
    positions.push_back(Eigen::Vector3d(-0.123576, 1.4504, 3.33051));
    positions.push_back(Eigen::Vector3d(-0.147016, 1.44998, 0.648832));
    positions.push_back(Eigen::Vector3d(-0.182566, 1.4548, 1.53786));
    positions.push_back(Eigen::Vector3d(-0.127534, 1.46245, 2.42439));
    for (int i = 1; i < g_correct_v; ++i)
    {
      Label label_l;
      label_l.labelType = STRING_LABEL;
      label_str = std::to_string(L_LONG).data();
      label_l.labelValue.stringLabel = label_str.data();
      ULONG label_index_v = StoreLabel(&label_l, parameters_->labelList);
      g_correct->vertices[i].label = label_index_v;
      g_correct->vertices[i].map = VERTEX_UNMAPPED;
      g_correct->vertices[i].used = FALSE;
      g_correct->vertices[i].numEdges = 0;
      g_correct->vertices[i].edges = NULL;
      g_correct->vertices[i].position = gsl_vector_alloc(3);
      // -4.335 1.45221 1.53744
      gsl_vector_set(g_correct->vertices[i].position, 0, positions[i-1].x());
      gsl_vector_set(g_correct->vertices[i].position, 1, positions[i-1].y());
      gsl_vector_set(g_correct->vertices[i].position, 2, positions[i-1].z());
      g_correct->vertices[i].orientation = gsl_vector_alloc(3);
      gsl_vector_set(g_correct->vertices[i].orientation, 0, 1.0);
      gsl_vector_set(g_correct->vertices[i].orientation, 1, 0.0);
      gsl_vector_set(g_correct->vertices[i].orientation, 2, 0.0);
      g_correct->vertices[i].in_pattern = FALSE;
    }
    for (int i = 0; i < g_correct_e; ++i)
    {
      Label e_label;
      e_label.labelType = STRING_LABEL;
      e_label.labelValue.stringLabel = "e";
      ULONG label_index_e = StoreLabel(&e_label, parameters_->labelList);
      // printf("Edge label: %s\n", parameters_->labelList->labels[label_index_e].labelValue);

      // add edge to graph
      g_correct->edges[i].vertex1 = 0;
      g_correct->edges[i].vertex2 = i+1;
      g_correct->edges[i].label = label_index_e;
      g_correct->edges[i].directed = FALSE;
      g_correct->edges[i].used = FALSE;
      g_correct->edges[i].spansIncrement = FALSE;
      g_correct->edges[i].validPath = TRUE;

      AddEdgeToVertices(g_correct, i);
    }
  }

  /// g_missing ///
  {
    Label label;
    label.labelType = STRING_LABEL;
    label_str = std::to_string(L_WALL).data();
    label.labelValue.stringLabel = label_str.data();
    ULONG label_index = StoreLabel(&label, parameters_->labelList);
    g_missing->vertices[0].label = label_index;
    g_missing->vertices[0].map = VERTEX_UNMAPPED;
    g_missing->vertices[0].used = FALSE;
    g_missing->vertices[0].numEdges = 0;
    g_missing->vertices[0].edges = NULL;
    g_missing->vertices[0].position = gsl_vector_alloc(3);
    // -4.335 1.45221 1.53744
    gsl_vector_set(g_missing->vertices[0].position, 0, -8.45429);
    gsl_vector_set(g_missing->vertices[0].position, 1, 1.70675);
    gsl_vector_set(g_missing->vertices[0].position, 2, 2.65857);
    g_missing->vertices[0].orientation = gsl_vector_alloc(3);
    gsl_vector_set(g_missing->vertices[0].orientation, 0, 1.0);
    gsl_vector_set(g_missing->vertices[0].orientation, 1, 0.0);
    gsl_vector_set(g_missing->vertices[0].orientation, 2, 0.0);
    g_missing->vertices[0].in_pattern = FALSE;
    std::vector<Eigen::Vector3d> positions;
    positions.push_back(Eigen::Vector3d(-8.33671, 1.40465, 2.42078));
    positions.push_back(Eigen::Vector3d(-8.45757, 1.45184, 0.649316));
    positions.push_back(Eigen::Vector3d(-8.46382, 1.43967, 4.21905));
    positions.push_back(Eigen::Vector3d(-8.45848, 1.45167, 1.5367));
    for (int i = 1; i < g_missing_v; ++i)
    {
      Label label_l;
      label_l.labelType = STRING_LABEL;
      label_str = std::to_string(L_LONG).data();
      label_l.labelValue.stringLabel = label_str.data();
      ULONG label_index_v = StoreLabel(&label_l, parameters_->labelList);
      g_missing->vertices[i].label = label_index_v;
      g_missing->vertices[i].map = VERTEX_UNMAPPED;
      g_missing->vertices[i].used = FALSE;
      g_missing->vertices[i].numEdges = 0;
      g_missing->vertices[i].edges = NULL;
      g_missing->vertices[i].position = gsl_vector_alloc(3);
      // -4.335 1.45221 1.53744
      gsl_vector_set(g_missing->vertices[i].position, 0, positions[i-1].x());
      gsl_vector_set(g_missing->vertices[i].position, 1, positions[i-1].y());
      gsl_vector_set(g_missing->vertices[i].position, 2, positions[i-1].z());
      g_missing->vertices[i].orientation = gsl_vector_alloc(3);
      gsl_vector_set(g_missing->vertices[i].orientation, 0, 1.0);
      gsl_vector_set(g_missing->vertices[i].orientation, 1, 0.0);
      gsl_vector_set(g_missing->vertices[i].orientation, 2, 0.0);
      g_missing->vertices[i].in_pattern = FALSE;
    }
    for (int i = 0; i < g_missing_e; ++i)
    {
      Label e_label;
      e_label.labelType = STRING_LABEL;
      e_label.labelValue.stringLabel = "e";
      ULONG label_index_e = StoreLabel(&e_label, parameters_->labelList);
      // printf("Edge label: %s\n", parameters_->labelList->labels[label_index_e].labelValue);

      // add edge to graph
      g_missing->edges[i].vertex1 = 0;
      g_missing->edges[i].vertex2 = i+1;
      g_missing->edges[i].label = label_index_e;
      g_missing->edges[i].directed = FALSE;
      g_missing->edges[i].used = FALSE;
      g_missing->edges[i].spansIncrement = FALSE;
      g_missing->edges[i].validPath = TRUE;

      AddEdgeToVertices(g_missing, i);
    }
  }

  /// g_wrong ///
  {
    Label label;
    label.labelType = STRING_LABEL;
    label_str = std::to_string(L_WALL).data();
    label.labelValue.stringLabel = label_str.data();
    ULONG label_index = StoreLabel(&label, parameters_->labelList);
    g_wrong->vertices[0].label = label_index;
    g_wrong->vertices[0].map = VERTEX_UNMAPPED;
    g_wrong->vertices[0].used = FALSE;
    g_wrong->vertices[0].numEdges = 0;
    g_wrong->vertices[0].edges = NULL;
    g_wrong->vertices[0].position = gsl_vector_alloc(3);
    // -4.335 1.45221 1.53744
    gsl_vector_set(g_wrong->vertices[0].position, 0, -8.45429);
    gsl_vector_set(g_wrong->vertices[0].position, 1, 1.70675);
    gsl_vector_set(g_wrong->vertices[0].position, 2, 2.65857);
    g_wrong->vertices[0].orientation = gsl_vector_alloc(3);
    gsl_vector_set(g_wrong->vertices[0].orientation, 0, 1.0);
    gsl_vector_set(g_wrong->vertices[0].orientation, 1, 0.0);
    gsl_vector_set(g_wrong->vertices[0].orientation, 2, 0.0);
    g_wrong->vertices[0].in_pattern = FALSE;

    Label label_2;
    label_2.labelType = STRING_LABEL;
    label_str = std::to_string(L_COMPARTMENT).data();
    label_2.labelValue.stringLabel = label_str.data();
    ULONG label_index_2 = StoreLabel(&label_2, parameters_->labelList);
    g_wrong->vertices[1].label = label_index_2;
    g_wrong->vertices[1].map = VERTEX_UNMAPPED;
    g_wrong->vertices[1].used = FALSE;
    g_wrong->vertices[1].numEdges = 0;
    g_wrong->vertices[1].edges = NULL;
    g_wrong->vertices[1].position = gsl_vector_alloc(3);
    // -4.335 1.45221 1.53744
    gsl_vector_set(g_wrong->vertices[1].position, 0, -8.47697);
    gsl_vector_set(g_wrong->vertices[1].position, 1, -1.03655);
    gsl_vector_set(g_wrong->vertices[1].position, 2, 2.67545);
    g_wrong->vertices[1].orientation = gsl_vector_alloc(3);
    gsl_vector_set(g_wrong->vertices[1].orientation, 0, 1.0);
    gsl_vector_set(g_wrong->vertices[1].orientation, 1, 0.0);
    gsl_vector_set(g_wrong->vertices[1].orientation, 2, 0.0);
    g_wrong->vertices[1].in_pattern = FALSE;
    
    std::vector<Eigen::Vector3d> positions;
    positions.push_back(Eigen::Vector3d(-8.33671, 1.40465, 2.42078));
    positions.push_back(Eigen::Vector3d(-8.45757, 1.45184, 0.649316));
    positions.push_back(Eigen::Vector3d(-8.46382, 1.43967, 4.21905));
    positions.push_back(Eigen::Vector3d(-8.45848, 1.45167, 1.5367));
    for (int i = 2; i < g_wrong_v; ++i)
    {
      Label label_l;
      label_l.labelType = STRING_LABEL;
      label_str = std::to_string(L_LONG).data();
      label_l.labelValue.stringLabel = label_str.data();
      ULONG label_index_v = StoreLabel(&label_l, parameters_->labelList);
      g_wrong->vertices[i].label = label_index_v;
      g_wrong->vertices[i].map = VERTEX_UNMAPPED;
      g_wrong->vertices[i].used = FALSE;
      g_wrong->vertices[i].numEdges = 0;
      g_wrong->vertices[i].edges = NULL;
      g_wrong->vertices[i].position = gsl_vector_alloc(3);
      // -4.335 1.45221 1.53744
      gsl_vector_set(g_wrong->vertices[i].position, 0, positions[i-2].x());
      gsl_vector_set(g_wrong->vertices[i].position, 1, positions[i-2].y());
      gsl_vector_set(g_wrong->vertices[i].position, 2, positions[i-2].z());
      g_wrong->vertices[i].orientation = gsl_vector_alloc(3);
      gsl_vector_set(g_wrong->vertices[i].orientation, 0, 1.0);
      gsl_vector_set(g_wrong->vertices[i].orientation, 1, 0.0);
      gsl_vector_set(g_wrong->vertices[i].orientation, 2, 0.0);
      g_wrong->vertices[i].in_pattern = FALSE;
    }

    for (int i = 0; i < g_wrong_e; ++i)
    {
      Label e_label;
      e_label.labelType = STRING_LABEL;
      e_label.labelValue.stringLabel = "e";
      ULONG label_index_e = StoreLabel(&e_label, parameters_->labelList);
      // printf("Edge label: %s\n", parameters_->labelList->labels[label_index_e].labelValue);

      // add edge to graph
      g_wrong->edges[i].vertex1 = 0;
      g_wrong->edges[i].vertex2 = i+1;
      g_wrong->edges[i].label = label_index_e;
      g_wrong->edges[i].directed = FALSE;
      g_wrong->edges[i].used = FALSE;
      g_wrong->edges[i].spansIncrement = FALSE;
      g_wrong->edges[i].validPath = TRUE;

      AddEdgeToVertices(g_wrong, i);
    }
  }

  /// g_extra ///
  {
    Label label;
    label.labelType = STRING_LABEL;
    label_str = std::to_string(L_WALL).data();
    label.labelValue.stringLabel = label_str.data();
    ULONG label_index = StoreLabel(&label, parameters_->labelList);
    g_extra->vertices[0].label = label_index;
    g_extra->vertices[0].map = VERTEX_UNMAPPED;
    g_extra->vertices[0].used = FALSE;
    g_extra->vertices[0].numEdges = 0;
    g_extra->vertices[0].edges = NULL;
    g_extra->vertices[0].position = gsl_vector_alloc(3);
    // -4.335 1.45221 1.53744
    gsl_vector_set(g_extra->vertices[0].position, 0, -8.45429);
    gsl_vector_set(g_extra->vertices[0].position, 1, 1.70675);
    gsl_vector_set(g_extra->vertices[0].position, 2, 2.65857);
    g_extra->vertices[0].orientation = gsl_vector_alloc(3);
    gsl_vector_set(g_extra->vertices[0].orientation, 0, 1.0);
    gsl_vector_set(g_extra->vertices[0].orientation, 1, 0.0);
    gsl_vector_set(g_extra->vertices[0].orientation, 2, 0.0);
    g_extra->vertices[0].in_pattern = FALSE;

    Label label_2;
    label_2.labelType = STRING_LABEL;
    label_str = std::to_string(L_COMPARTMENT).data();
    label_2.labelValue.stringLabel = label_str.data();
    ULONG label_index_2 = StoreLabel(&label_2, parameters_->labelList);
    g_extra->vertices[1].label = label_index_2;
    g_extra->vertices[1].map = VERTEX_UNMAPPED;
    g_extra->vertices[1].used = FALSE;
    g_extra->vertices[1].numEdges = 0;
    g_extra->vertices[1].edges = NULL;
    g_extra->vertices[1].position = gsl_vector_alloc(3);
    // -4.335 1.45221 1.53744
    gsl_vector_set(g_extra->vertices[1].position, 0, -8.47697);
    gsl_vector_set(g_extra->vertices[1].position, 1, -1.03655);
    gsl_vector_set(g_extra->vertices[1].position, 2, 2.67545);
    g_extra->vertices[1].orientation = gsl_vector_alloc(3);
    gsl_vector_set(g_extra->vertices[1].orientation, 0, 1.0);
    gsl_vector_set(g_extra->vertices[1].orientation, 1, 0.0);
    gsl_vector_set(g_extra->vertices[1].orientation, 2, 0.0);
    g_extra->vertices[1].in_pattern = FALSE;

    // std::cout << "W, C set" << std::endl;
    
    std::vector<Eigen::Vector3d> positions;
    positions.push_back(Eigen::Vector3d(-8.33671, 1.40465, 2.42078));
    positions.push_back(Eigen::Vector3d(-8.45757, 1.45184, 0.649316));
    positions.push_back(Eigen::Vector3d(-8.46382, 1.43967, 4.21905));
    positions.push_back(Eigen::Vector3d(-8.45848, 1.45167, 1.5367));
    positions.push_back(Eigen::Vector3d(-8.39768, 1.40391, 3.33604));
    for (int i = 2; i < g_extra_v; ++i)
    {
      Label label_l;
      label_l.labelType = STRING_LABEL;
      label_str = std::to_string(L_LONG).data();
      label_l.labelValue.stringLabel = label_str.data();
      ULONG label_index_v = StoreLabel(&label_l, parameters_->labelList);
      g_extra->vertices[i].label = label_index_v;
      g_extra->vertices[i].map = VERTEX_UNMAPPED;
      g_extra->vertices[i].used = FALSE;
      g_extra->vertices[i].numEdges = 0;
      g_extra->vertices[i].edges = NULL;
      g_extra->vertices[i].position = gsl_vector_alloc(3);
      // -4.335 1.45221 1.53744
      gsl_vector_set(g_extra->vertices[i].position, 0, positions[i-2].x());
      gsl_vector_set(g_extra->vertices[i].position, 1, positions[i-2].y());
      gsl_vector_set(g_extra->vertices[i].position, 2, positions[i-2].z());
      g_extra->vertices[i].orientation = gsl_vector_alloc(3);
      gsl_vector_set(g_extra->vertices[i].orientation, 0, 1.0);
      gsl_vector_set(g_extra->vertices[i].orientation, 1, 0.0);
      gsl_vector_set(g_extra->vertices[i].orientation, 2, 0.0);
      g_extra->vertices[i].in_pattern = FALSE;
    }

    // std::cout << "vertices set" << std::endl;

    for (int i = 0; i < g_extra_e; ++i)
    {
      Label e_label;
      e_label.labelType = STRING_LABEL;
      e_label.labelValue.stringLabel = "e";
      ULONG label_index_e = StoreLabel(&e_label, parameters_->labelList);
      // printf("Edge label: %s\n", parameters_->labelList->labels[label_index_e].labelValue);

      // add edge to graph
      g_extra->edges[i].vertex1 = 0;
      g_extra->edges[i].vertex2 = i+1;
      g_extra->edges[i].label = label_index_e;
      g_extra->edges[i].directed = FALSE;
      g_extra->edges[i].used = FALSE;
      g_extra->edges[i].spansIncrement = FALSE;
      g_extra->edges[i].validPath = TRUE;
      // std::cout << "edge properties set" << std::endl;

      AddEdgeToVertices(g_extra, i);
    }
  }

  std::cout << "g_extra set" << std::endl;

  /// g_correct_copy ///
  {
    Label label;
    label.labelType = STRING_LABEL;
    label_str = std::to_string(L_WALL).data();
    label.labelValue.stringLabel = label_str.data();
    ULONG label_index = StoreLabel(&label, parameters_->labelList);
    g_correct_copy->vertices[0].label = label_index;
    g_correct_copy->vertices[0].map = VERTEX_UNMAPPED;
    g_correct_copy->vertices[0].used = FALSE;
    g_correct_copy->vertices[0].numEdges = 0;
    g_correct_copy->vertices[0].edges = NULL;
    g_correct_copy->vertices[0].position = gsl_vector_alloc(3);
    // -4.335 1.45221 1.53744
    gsl_vector_set(g_correct_copy->vertices[0].position, 0, -8.45429);
    gsl_vector_set(g_correct_copy->vertices[0].position, 1, 1.70675);
    gsl_vector_set(g_correct_copy->vertices[0].position, 2, 2.65857);
    g_correct_copy->vertices[0].orientation = gsl_vector_alloc(3);
    gsl_vector_set(g_correct_copy->vertices[0].orientation, 0, 1.0);
    gsl_vector_set(g_correct_copy->vertices[0].orientation, 1, 0.0);
    gsl_vector_set(g_correct_copy->vertices[0].orientation, 2, 0.0);
    g_correct_copy->vertices[0].in_pattern = FALSE;
    std::vector<Eigen::Vector3d> positions;
    positions.push_back(Eigen::Vector3d(-8.33671, 1.40465, 2.42078));
    positions.push_back(Eigen::Vector3d(-8.45757, 1.45184, 0.649316));
    positions.push_back(Eigen::Vector3d(-8.46382, 1.43967, 4.21905));
    positions.push_back(Eigen::Vector3d(-8.45848, 1.45167, 1.5367));
    positions.push_back(Eigen::Vector3d(-8.39768, 1.40391, 3.33604));
    for (int i = 1; i < g_correct_copy_v; ++i)
    {
      Label label_l;
      label_l.labelType = STRING_LABEL;
      label_str = std::to_string(L_LONG).data();
      label_l.labelValue.stringLabel = label_str.data();
      ULONG label_index_v = StoreLabel(&label_l, parameters_->labelList);
      g_correct_copy->vertices[i].label = label_index_v;
      g_correct_copy->vertices[i].map = VERTEX_UNMAPPED;
      g_correct_copy->vertices[i].used = FALSE;
      g_correct_copy->vertices[i].numEdges = 0;
      g_correct_copy->vertices[i].edges = NULL;
      g_correct_copy->vertices[i].position = gsl_vector_alloc(3);
      // -4.335 1.45221 1.53744
      gsl_vector_set(g_correct_copy->vertices[i].position, 0, positions[i-1].x());
      gsl_vector_set(g_correct_copy->vertices[i].position, 1, positions[i-1].y());
      gsl_vector_set(g_correct_copy->vertices[i].position, 2, positions[i-1].z());
      g_correct_copy->vertices[i].orientation = gsl_vector_alloc(3);
      gsl_vector_set(g_correct_copy->vertices[i].orientation, 0, 1.0);
      gsl_vector_set(g_correct_copy->vertices[i].orientation, 1, 0.0);
      gsl_vector_set(g_correct_copy->vertices[i].orientation, 2, 0.0);
      g_correct_copy->vertices[i].in_pattern = FALSE;
    }
    for (int i = 0; i < g_correct_copy_e; ++i)
    {
      Label e_label;
      e_label.labelType = STRING_LABEL;
      e_label.labelValue.stringLabel = "e";
      ULONG label_index_e = StoreLabel(&e_label, parameters_->labelList);
      // printf("Edge label: %s\n", parameters_->labelList->labels[label_index_e].labelValue);

      // add edge to graph
      g_correct_copy->edges[i].vertex1 = 0;
      g_correct_copy->edges[i].vertex2 = i+1;
      g_correct_copy->edges[i].label = label_index_e;
      g_correct_copy->edges[i].directed = FALSE;
      g_correct_copy->edges[i].used = FALSE;
      g_correct_copy->edges[i].spansIncrement = FALSE;
      g_correct_copy->edges[i].validPath = TRUE;

      AddEdgeToVertices(g_correct_copy, i);
    }
  }

  std::cout << "g_correct set" << std::endl;

  PrintGraph(g_correct, parameters_->labelList);
  PrintGraph(g_missing, parameters_->labelList);
  PrintGraph(g_extra, parameters_->labelList);
  PrintGraph(g_wrong, parameters_->labelList);
  PrintGraph(g_correct_copy, parameters_->labelList);

  std::map<std::string, double> match_costs;
  std::map<std::string, bool> matches;
  std::map<std::string, VertexMap*> vertex_maps;
  vertex_maps["missing"] = (VertexMap *) malloc(sizeof(VertexMap) * g_extra_v);
  vertex_maps["wrong"] = (VertexMap *) malloc(sizeof(VertexMap) * g_extra_v);
  vertex_maps["extra"] = (VertexMap *) malloc(sizeof(VertexMap) * g_extra_v);
  vertex_maps["copy"] = (VertexMap *) malloc(sizeof(VertexMap) * g_extra_v);
  // match_costs.reserve(4);
  // matches.reserve(4);

  double thresholdLimit = parameters_->threshold * (g_correct->numVertices + g_correct->numEdges);

  matches["missing"] = GraphMatch(g_correct, g_missing, parameters_->labelList, thresholdLimit, &match_costs["missing"], vertex_maps["missing"], parameters_->use_pose_cost);
  matches["wrong"] = GraphMatch(g_correct, g_wrong, parameters_->labelList, thresholdLimit, &match_costs["wrong"], vertex_maps["wrong"], parameters_->use_pose_cost);
  matches["extra"] = GraphMatch(g_correct, g_extra, parameters_->labelList, thresholdLimit, &match_costs["extra"], vertex_maps["extra"], parameters_->use_pose_cost);
  matches["copy"] = GraphMatch(g_correct, g_correct_copy, parameters_->labelList, thresholdLimit, &match_costs["copy"], vertex_maps["copy"], parameters_->use_pose_cost);

  std::cout << "Results:" << std::endl;
  for(auto it : match_costs)
  {
    std::cout << it.first << ": " << it.second << std::endl;
    for(int i=0; i<g_correct_v; ++i)
    {
      std::cout << vertex_maps[it.first][i].v1 << " : " << vertex_maps[it.first][i].v2 << std::endl;
    }
  }

  res.success = true;
  return true;
}

void PredictivePlanning::findPatterns()
{
  current_hierarchy_.reset(new SSGHierarchy());
  pattern_structs_.clear();
  setSubdueParams();
  SSGLevel zeroth_level;
  parameters_->posGraph = ssgToSubdue(comm_->ssg_manager(), parameters_, zeroth_level);

  // std::cout << "Graph converted" << std::endl;
  if (parameters_->evalMethod == EVAL_MDL)
  {
    parameters_->posGraphDL = MDL(parameters_->posGraph,
                                  parameters_->labelList->numLabels, parameters_);
    if (parameters_->negGraph != NULL)
    {
      parameters_->negGraphDL =
          MDL(parameters_->negGraph, parameters_->labelList->numLabels,
              parameters_);
    }
  }
  for (auto v_it : comm_->ssg_manager()->vertices_map_)
  {
    base_labels.insert(std::to_string(v_it.second->label));
  }
  // parameters_->posGraph = g;
  PostProcessParameters(parameters_);
  Graph *compressed_graphs[parameters_->iterations+1];
  LabelList *all_label_lists[parameters_->iterations];
  Substructure *discovered_subs[parameters_->iterations];
  LabelList *og_label_list;
  og_label_list = AllocateLabelList();
  for (int il = 0; il < parameters_->labelList->numLabels; ++il)
  {
    Label new_label = parameters_->labelList->labels[il];
    StoreLabel(&new_label, og_label_list);
  }
  runSubdue(parameters_, compressed_graphs, all_label_lists, discovered_subs);
  // for(int v=0; v<compressed_graphs[parameters_->iterations]->numVertices; ++v)
  // {
  //   std::cout << v << ": " << compressed_graphs[parameters_->iterations]->vertices[v].label;
  //   std::cout << " - "  << compressed_graphs[parameters_->iterations]->vertices[v].map << ": " 
  //             << compressed_graphs[0]->vertices[compressed_graphs[parameters_->iterations]->vertices[v].map].label << std::endl;
  // }
  // std::cout << "Subdue run" << std::endl;
  // std::cout << "OG Label list: [";
  // for(int i=0; i<og_label_list->numLabels; ++i)
  // {
  //   std::cout << og_label_list->labels[i].labelValue.stringLabel << ", ";
  // }
  // std::cout << "]" << std::endl;
  // PrintGraph(compressed_graphs[0], all_label_lists[0]);
  // std::cout << "Graph printed" << std::endl;

  // std::cout << "SSG : Subdue maps:" << std::endl;
  // std::cout << "  Vertices:" << std::endl;
  // for(auto v_it : zeroth_level.vertex_subdue_indx_To_ssg_id)
  // {
  //   std::cout << v_it.first << " : " << v_it.second << std::endl;
  // }
  // std::cout << "  Edges:" << std::endl;
  // for(auto e_it : zeroth_level.edge_subdue_indx_To_ssg_id)
  // {
  //   std::cout << e_it.first << " : " << e_it.second << std::endl;
  // }

  // std::cout << "SSG - All neighbor maps:" << std::endl;
  // for(auto v_it : comm_->ssg_manager()->vertices_map_)
  // {
  //   std::cout << v_it.first << ":" << std::endl;
  //   for(auto n_it : v_it.second->neighbor_map)
  //   {
  //     std::cout << "  (v)" << n_it.first << " : (e)" << n_it.second << std::endl;
  //   }
  // }

  // for (auto v_it : comm_->ssg_manager()->vertices_map_)
  // {
  //   for (auto n_it : v_it.second->neighbor_map)
  //   {
  //     if (comm_->ssg_manager()->getVertex(n_it.first)->label == L_MANHOLE)
  //     {
  //       std::cout << "Manhole neighbor " << n_it.first << std::endl;
  //     }
  //   }
  // }

  visualization_msgs::MarkerArray compressed_graphs_vis;

  std::map<int, int> vertex_subdue_indx_To_ssg_id, edge_subdue_indx_To_ssg_id;
  vertex_subdue_indx_To_ssg_id = zeroth_level.vertex_subdue_indx_To_ssg_id;
  edge_subdue_indx_To_ssg_id = zeroth_level.edge_subdue_indx_To_ssg_id;

  // std::cout << "OG SSG edges:" << std::endl;
  // for (auto e_it : comm_->ssg_manager()->edge_map_)
  // {
  //   std::cout << e_it.first << std::endl;
  // }

  for (int i = 0; i < parameters_->iterations; ++i)
  {
    std::cout << i << "th compression:" << std::endl;
    // PrintGraph(compressed_graphs[i], all_label_lists[i]);
    SSGLevel ith_level;
    // Convert pattern structs
    std::shared_ptr<PatternStructure> ith_pattern_struct;
    if (i == 0)
    {
      ith_pattern_struct = subdueStructToPatternStruct(discovered_subs[i], og_label_list, vertex_subdue_indx_To_ssg_id, edge_subdue_indx_To_ssg_id);
      // std::cout << "i=0 Substruct converted" << std::endl;
      ith_level.base_graph.reset(new SSGManager(*comm_->ssg_manager()));
      
      // std::cout << "base graph set" << std::endl;
    }
    else
    {
      ith_pattern_struct = subdueStructToPatternStruct(discovered_subs[i], all_label_lists[i - 1], vertex_subdue_indx_To_ssg_id, edge_subdue_indx_To_ssg_id);
      // std::cout << "i>0 Substruct converted" << std::endl;
      ith_level.base_graph = current_hierarchy_->levels.back().compressed_graph;
      // std::cout << "base graph set" << std::endl;
    }

    // std::cout << "Bboxes: " << std::endl;
    // for(auto v_it : ith_level.base_graph->vertices_map_)
    // {
    //   std::cout << v_it.second->bbox.transpose() << std::endl;
    // }
    
    /********/
    /// TODO: This should ideally go in the subdueStructToPatternStruct function
    std::map<int, int> base_ssg_to_def_map;
    ith_pattern_struct->definition.reset(new SSGManager);
    for(int v_id : ith_pattern_struct->instances[0].vertex_ids)
    {
      std::shared_ptr<SemanticVertex> new_vertex = ith_pattern_struct->definition->initializeNewVertex();
      ith_level.base_graph->getVertex(v_id)->copyTo(new_vertex, false);
      base_ssg_to_def_map[v_id] = new_vertex->id;
      ith_pattern_struct->definition->addVertex(new_vertex);
    }
    for(int e_id : ith_pattern_struct->instances[0].edge_ids)
    {
      std::shared_ptr<EdgeRelation> new_edge = ith_pattern_struct->definition->initializeNewEdge();
      std::shared_ptr<EdgeRelation> og_edge = ith_level.base_graph->getEdge(e_id);
      new_edge->label = og_edge->label;
      new_edge->weight = og_edge->weight;
      new_edge->source_vertex = ith_pattern_struct->definition->getVertex(base_ssg_to_def_map[og_edge->source_vertex->id]);
      new_edge->target_vertex = ith_pattern_struct->definition->getVertex(base_ssg_to_def_map[og_edge->target_vertex->id]);
      ith_pattern_struct->definition->addEdge(new_edge);
    }
    /********/
    // std::cout << "Struct Bboxes: " << std::endl;
    // for(auto v_it : ith_pattern_struct->definition->vertices_map_)
    // {
    //   std::cout << v_it.second->bbox.transpose() << std::endl;
    // }

    // std::cout << "Found structure " << ith_pattern_struct->name << " Num instances: " << ith_pattern_struct->instances.size() << std::endl;
    pattern_structs_[ith_pattern_struct->name] = ith_pattern_struct;

    // Assign instance ids
    for (int inst_id = 0; inst_id < ith_pattern_struct->instances.size(); ++inst_id)
    {
      for (int v_id : ith_pattern_struct->instances[inst_id].vertex_ids)
      {
        ith_level.base_graph->getVertex(v_id)->instance_ind = inst_id;
        ith_level.base_graph->getVertex(v_id)->pattern_name = ith_pattern_struct->name;
      }
    }

    // Conver compressed graph
    // std::shared_ptr<SSGManager> ith_compression_graph =
    //     subdueToSSG(compressed_graphs[i], all_label_lists[i], vertex_subdue_indx_To_ssg_id, edge_subdue_indx_To_ssg_id);
    std::shared_ptr<SSGManager> ith_compression_graph =
        convertSubdueCompressedGraph(compressed_graphs[i], all_label_lists[i], ith_level.base_graph, vertex_subdue_indx_To_ssg_id, edge_subdue_indx_To_ssg_id);
    // compressed_graphs_ssg_.push_back(ith_compression_graph);
    ith_level.compressed_graph = ith_compression_graph;
    ith_level.discovered_pattern_names.insert(ith_pattern_struct->name);
    // Update names lists
    for (auto v_it : ith_compression_graph->vertices_map_)
    {
      if (v_it.second->pattern_name != NP)
      {
        ith_level.discovered_pattern_names.insert(v_it.second->pattern_name);
      }
    }

    //   std::cout << "Structures discovered so far: ";
    // for (auto it : pattern_structs_)
    //   std::cout << it.first << ", ";
    // std::cout << std::endl;

    // std::cout << "Structures discovered in this level: ";
    // for (auto it : ith_level.discovered_pattern_names)
    //   std::cout << it << ", ";
    // std::cout << std::endl;

    for (std::string pn : ith_level.discovered_pattern_names)
    {
      // std::cout << "Pattern " << pn << std::endl;
      for (auto inst : pattern_structs_[pn]->instances)
      {
        // std::cout << "Instance: " << inst.inst_id << std::endl;
        for (int e_id : inst.edge_ids)
        {
          // std::cout << e_id << std::endl;
          auto result = std::find_if(
              ith_level.base_graph->edge_map_.begin(),
              ith_level.base_graph->edge_map_.end(),
              [e_id](const auto &mo)
              { return mo.first == e_id; });
          // if (result == ith_level.base_graph->edge_map_.end())
          // {
          //   std::cout << "Edge " << e_id << " not in the graph";
          // }
        }
      }
    }

    // //
    // for(std::string ps : ith_level.discovered_pattern_names)
    // {
    //   std::shared_ptr<PatternStructure> p_struct = pattern_structs_[ps];
    //   for(auto inst : p_struct->instances)
    //   {
    //     for(auto v_id : inst.vertex_ids)
    //     {
    //       ith_level.base_graph->getVertex(v_id)->pattern_name = p_struct->name;
    //     }
    //   }
    // }

    current_hierarchy_->levels.push_back(ith_level);

    visualization_msgs::MarkerArray ith_compressed_graphs_vis = ith_compression_graph->getGraphVis();
    Eigen::Vector3d offset(0.0, 0.0, 9.0 * (i + 1));
    // std::cout << offset.transpose() << std::endl;
    offsetGraphVis(ith_compressed_graphs_vis, offset);
    smartInsert(compressed_graphs_vis, ith_compressed_graphs_vis);
  }

  std::cout << "Converted to ssg" << std::endl;

  // std::cout << "Got vis" << std::endl;
  // std::cout << "Offseted vis" << std::endl;
  compressed_graph_pub_.publish(compressed_graphs_vis);
  // std::cout << "OG SSG: v:" << comm_->ssg_manager()->id_count_ << " e: " << comm_->ssg_manager()->edge_id_count_
  //           << " | H SSG: v:" << current_hierarchy_->levels[0].base_graph->id_count_ << " e: " << current_hierarchy_->levels[0].base_graph->edge_id_count_ << std::endl;
  visualizeHierarchy(*current_hierarchy_, config_->prediction_params.visualize_enhance_hierarchy);

  insertEntryVertices();
  std::cout << "Entry vertices inserted" << std::endl;
  visualizeHierarchy(*current_hierarchy_, config_->prediction_params.visualize_enhance_hierarchy);
  // for(auto level : current_hierarchy_->levels)
  // {
  //   for(auto v_it : level.base_graph->vertices_map_)
  //   {
  //     for(auto n_it : v_it.second->neighbor_map)
  //     {
  //       if(level.base_graph->getVertex(n_it.first)->label == L_MANHOLE)
  //       {
  //         std::cout << "Vertex " << v_it.first << ": Manhole neighbor " << n_it.first << std::endl;
  //       }
  //     }
  //   }
  // }
}

bool PredictivePlanning::insertverticesCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  insertEntryVertices();
  visualizeHierarchy(*current_hierarchy_, true);

  res.success = true;
  return true;
}

bool PredictivePlanning::findPatternsCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  // config_->loadAllParams(ros::this_node::getName());
  config_->subdue_params.loadParams(ros::this_node::getName() + "/SubsdueParams");
  std::cout << "Subdue thr: " << config_->subdue_params.threshold << std::endl;
  std::cout << "Subdue limit: " << config_->subdue_params.limit << std::endl;
  findPatterns();

  res.success = true;
  return true;
}

bool PredictivePlanning::extendGraphCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  // config_->loadAllParams(ros::this_node::getName());
  // std::cout << "Subdue thr: " << config_->subdue_params.threshold << std::endl;
  extendGraphToStructs();

  res.success = true;
  return true;
}

bool PredictivePlanning::saveSSGCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  comm_->ssg_manager()->saveGraph(config_->ssg_params.path_to_save);
  res.success = true;
  return true;
}

bool PredictivePlanning::loadSSGCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  comm_->ssg_manager()->loadGraph(config_->ssg_params.path_to_load);
  for (auto v_it : comm_->ssg_manager()->vertices_map_)
  {
    std::cout << v_it.first << " (" << v_it.second->label << "): ";
    for (auto n_it : v_it.second->neighbor_map)
    {
      std::cout << n_it.first << " ";
    }
    std::cout << std::endl;
    // TODO: Fix this hack (should be fixed in the detector)
    v_it.second->state(3) = 0.0;
    v_it.second->state(4) = 0.0;
  }

  ///// Skip longitudinals /////
  if(config_->ssg_params.num_longs_to_skip > 0 && config_->ssg_params.num_pats_to_modify > 0)
  {
    std::cout << "Need to skip " << config_->ssg_params.num_longs_to_skip << " in " << config_->ssg_params.num_pats_to_modify << " patterns" << std::endl;
    std::shared_ptr<SSGManager> pruned_ssg_manager;
    pruned_ssg_manager.reset(new SSGManager);
    std::map<int, int> pruned_vertex_map;
    std::vector<int> skipped_vertices;
    int pat_counter = 0;
    int id_count = 0;
    for (auto v_it : comm_->ssg_manager()->vertices_map_)
    {
      if(v_it.second->label != L_WALL && v_it.second->label != L_LONG)
      {
        std::shared_ptr<SemanticVertex> new_vertex;
        new_vertex.reset(new SemanticVertex(*(v_it.second)));
        new_vertex->id = id_count;
        pruned_ssg_manager->addVertex(new_vertex);
        pruned_vertex_map[v_it.first] = id_count;
        ++id_count;
      }
      else
      {
        std::cout << "Wall or long" << std::endl;
        if(v_it.second->label == L_WALL)
        {
          std::cout << "Wall " << v_it.first << std::endl;
          std::cout << "Num neighbors b: " << v_it.second->neighbor_map.size() << std::endl;
          std::shared_ptr<SemanticVertex> new_vertex;
          new_vertex.reset(new SemanticVertex(*(v_it.second)));
          new_vertex->id = id_count;
          pruned_ssg_manager->addVertex(new_vertex);
          pruned_vertex_map[v_it.first] = id_count;
          ++id_count;
          int skip_count = 0;
          bool long_found = false;
          std::cout << "Num neighbors a: " << v_it.second->neighbor_map.size() << std::endl;
          for(auto n_it : v_it.second->neighbor_map)
          {
            std::cout << "Neighbor: " << n_it.first << " label: " << comm_->ssg_manager()->getVertex(n_it.first)->label << std::endl;
            if(comm_->ssg_manager()->getVertex(n_it.first)->label != L_LONG)
              continue;
            else
            {
              std::cout << "Neighbor long" << std::endl;
              long_found = true;
              if(pat_counter < config_->ssg_params.num_pats_to_modify)
              {
                std::cout << "Pats remaining to be skipped" << std::endl;
                if(skip_count < config_->ssg_params.num_longs_to_skip)
                {
                  ++skip_count;
                  std::cout << "Skip counter : " << skip_count << std::endl;
                  skipped_vertices.push_back(n_it.first);
                  continue;
                }
                else
                {
                  std::shared_ptr<SemanticVertex> new_vertex;
                  new_vertex.reset(new SemanticVertex(*(comm_->ssg_manager()->getVertex(n_it.first))));
                  new_vertex->id = id_count;
                  pruned_ssg_manager->addVertex(new_vertex);
                  pruned_vertex_map[n_it.first] = id_count;
                  ++id_count;
                }
              }
              else
              {
                std::cout << "Pats NOT remaining to be skipped" << std::endl;
                std::shared_ptr<SemanticVertex> new_vertex;
                new_vertex.reset(new SemanticVertex(*(comm_->ssg_manager()->getVertex(n_it.first))));
                new_vertex->id = id_count;
                pruned_ssg_manager->addVertex(new_vertex);
                pruned_vertex_map[n_it.first] = id_count;
                ++id_count;
              }
            }
          }
          if(long_found)
            ++pat_counter;
          std::cout << "Pat counter : " << pat_counter << std::endl;
        }
      }
    }

    int e_id_counter = 0;
    for(auto e_it : comm_->ssg_manager()->edge_map_)
    {
      std::cout << "Checking edge " << e_it.first << std::endl;
      auto it1 = std::find(skipped_vertices.begin(), skipped_vertices.end(), e_it.second->source_vertex->id);
      auto it2 = std::find(skipped_vertices.begin(), skipped_vertices.end(), e_it.second->target_vertex->id);
      if(it1 != skipped_vertices.end() || it2 != skipped_vertices.end())  // vertex is skipped. Skip the edge
      {
        std::cout << "Vertex " << e_it.second->source_vertex->id << " or " << e_it.second->target_vertex->id << " skipped" << std::endl;
        continue;
      }

      std::shared_ptr<EdgeRelation> new_edge;
      new_edge.reset(new EdgeRelation(*(e_it.second)));
      new_edge->id = e_id_counter;
      new_edge->source_vertex.reset(new SemanticVertex(*(pruned_ssg_manager->getVertex(pruned_vertex_map[e_it.second->source_vertex->id]))));
      new_edge->target_vertex.reset(new SemanticVertex(*(pruned_ssg_manager->getVertex(pruned_vertex_map[e_it.second->target_vertex->id]))));
      pruned_ssg_manager->addEdge(new_edge);
      ++e_id_counter;
    }

    std::cout << "Before: " << comm_->ssg_manager()->vertices_map_.size() << " pruned: " << pruned_ssg_manager->vertices_map_.size();
    // comm_->ssg_manager().reset(new SSGManager(*pruned_ssg_manager));
    comm_->ssg_manager()->ssg_.reset(new BaseGraph);
    comm_->ssg_manager()->id_count_ = pruned_ssg_manager->vertices_map_.size();
    comm_->ssg_manager()->edge_id_count_ = pruned_ssg_manager->edge_map_.size();
    comm_->ssg_manager()->vertices_map_.clear();
    for(auto v_it : pruned_ssg_manager->vertices_map_)
    {
      std::shared_ptr<SemanticVertex> new_vertex;
      new_vertex.reset(new SemanticVertex(*(v_it.second)));
      comm_->ssg_manager()->addVertex(new_vertex);
    }
    // std::cout << "v set" << std::endl;

    comm_->ssg_manager()->edge_map_.clear();
    for(auto e_it : pruned_ssg_manager->edge_map_)
    {
      std::shared_ptr<EdgeRelation> new_edge;
      new_edge.reset(new EdgeRelation(*(e_it.second)));
      comm_->ssg_manager()->addEdge(new_edge);
    }
    std::cout << " After: " << comm_->ssg_manager()->vertices_map_.size() << std::endl;
  }

  // std::shared_ptr<SemanticVertex> attached_vertex = comm_->ssg_manager()->getVertex(102);
  
  // std::shared_ptr<SemanticVertex> new_vertex;
  // new_vertex.reset(new SemanticVertex());
  // new_vertex->id = comm_->ssg_manager()->vertices_map_.size();
  // new_vertex->label = L_MANHOLE;
  // new_vertex->state(0) = attached_vertex->state(0) + 5.0;
  // new_vertex->state(1) = attached_vertex->state(1);
  // new_vertex->state(2) = attached_vertex->state(2);
  // new_vertex->state(5) = 0.0;
  // comm_->ssg_manager()->addVertex(new_vertex);

  // std::shared_ptr<EdgeRelation> new_edge;
  // new_edge.reset(new EdgeRelation());
  // new_edge->id = comm_->ssg_manager()->edge_map_.size();
  // new_edge->label = 1;
  // new_edge->weight = 1.0;
  // new_edge->source_vertex = new_vertex;
  // new_edge->target_vertex = attached_vertex;
  // comm_->ssg_manager()->addEdge(new_edge);
  //////////////////////////////
  
  // for(auto e_it : comm_->ssg_manager()->edge_map_)
  // {
  //   std::cout << e_it.first << " = " << e_it.second->id << std::endl;
  // }
  ssg_vis_pub_.publish(comm_->ssg_manager()->getGraphVis());
  // for(int i=0; i<10; ++i)
  // {
  //   ros::Duration(0.05).sleep();
  //   ros::spinOnce();
  // }
  res.success = true;
  return true;
}

void PredictivePlanning::findPatternsTimerCallback(const ros::TimerEvent &event)
{
  if (comm_->ssg_manager()->vertices_map_.empty())
    return;
  findPatterns();
}

void PredictivePlanning::offsetGraphVis(visualization_msgs::MarkerArray &graph_vis, Eigen::Vector3d offset)
{
  for (auto &m : graph_vis.markers)
  {
    // for(auto &p : m.points)
    // {
    //   p.x += offset.x();
    //   p.y += offset.y();
    //   p.z += offset.z();
    // }
    m.pose.position.x += offset.x();
    m.pose.position.y += offset.y();
    m.pose.position.z += offset.z();
  }
}

void PredictivePlanning::insertEntryVertices()
{
  for (int lvl_num = 0; lvl_num < current_hierarchy_->levels.size(); ++lvl_num)
  {
    SSGLevel &level = current_hierarchy_->levels[lvl_num];
    // std::cout << "All edges" << std::endl;
    // for(auto e : level.base_graph->edge_map_)
    // {
    //   std::cout << e.first << std::endl;
    // }
    // std::cout << "Found level" << std::endl;
    std::shared_ptr<SSGManager> enhanced_base_graph;
    // enhanced_base_graph.reset(new SSGManager(*level.base_graph));
    enhanced_base_graph = level.base_graph->createCopy();
    // level.enhanced_base_graph.reset(new SSGManager(*level.base_graph));
    level.enhanced_base_graph = level.base_graph->createCopy();

    for (std::string substruct_name : level.discovered_pattern_names)
    {
      std::shared_ptr<PatternStructure> enhanced_struct;
      enhanced_struct.reset(new PatternStructure(*pattern_structs_[substruct_name]));
      bool struct_enhanced = false;
      for (int inst_i = 0; inst_i < pattern_structs_[substruct_name]->instances.size(); ++inst_i)
      {
        PatternInstance &inst = pattern_structs_[substruct_name]->instances[inst_i];
        // for(int v_id : inst.vertex_ids)
        
        for (int vi = 0; vi < inst.vertex_ids.size(); ++vi)
        {
          int v_id = inst.vertex_ids[vi];
          for (auto n_it : level.enhanced_base_graph->getVertex(v_id)->neighbor_map)
          {
            if (level.enhanced_base_graph->getVertex(n_it.first)->label == L_MANHOLE)
            {
              auto it_v = std::find(inst.vertex_ids.begin(), inst.vertex_ids.end(), n_it.first);
              if (it_v == inst.vertex_ids.end()) // The neighbor is manhole and not part of this instance
              {
                // Add vertex
                std::shared_ptr<SemanticVertex> new_entry_vertex = enhanced_base_graph->initializeNewVertex();
                // new_entry_vertex->state = level.enhanced_base_graph->getVertex(v_id)->state;
                new_entry_vertex->state = level.enhanced_base_graph->getVertex(n_it.first)->state;
                new_entry_vertex->state(0) += 0.1; // FOR VIS ONLY
                new_entry_vertex->pattern_name = enhanced_struct->name;
                new_entry_vertex->instance_ind = inst_i;
                new_entry_vertex->label = L_MANHOLE;
                // new_entry_vertex->enhanced = true;
                new_entry_vertex->status = VertexStatus::kEnhanced;
                enhanced_base_graph->addVertex(new_entry_vertex);
                // Update edges
                std::shared_ptr<EdgeRelation> new_edge = enhanced_base_graph->initializeNewEdge();
                new_edge->source_vertex = enhanced_base_graph->getVertex(n_it.first);
                new_edge->target_vertex = new_entry_vertex;
                enhanced_base_graph->addEdge(new_edge);

                enhanced_base_graph->getEdge(n_it.second)->source_vertex = new_entry_vertex;
                enhanced_base_graph->getEdge(n_it.second)->target_vertex = enhanced_base_graph->getVertex(v_id);
                enhanced_base_graph->getVertex(n_it.first)->neighbor_map.erase(v_id);
                // enhanced_base_graph->getVertex(n_it.first)->neighbor_map[new_entry_vertex->id] = new_edge->id;
                enhanced_base_graph->getVertex(v_id)->neighbor_map.erase(n_it.first);
                enhanced_base_graph->getVertex(v_id)->neighbor_map[new_entry_vertex->id] = n_it.second;
                new_entry_vertex->neighbor_map[v_id] = n_it.second;

                enhanced_struct->instances[inst_i].vertex_ids.push_back(new_entry_vertex->id);
                enhanced_struct->instances[inst_i].edge_ids.push_back(n_it.second);

                struct_enhanced = true;

                // if(inst_i == 0)
                // {
                //   std::shared_ptr<SemanticVertex> new_vertex_en = enhanced_struct->definition->initializeNewVertex();
                //   new_vertex_en->label = new_entry_vertex->label;
                //   new_vertex_en->state = new_entry_vertex->state;
                //   new_vertex_en->pattern_name = new_entry_vertex->pattern_name;
                //   enhanced_struct->definition->addVertex(new_vertex_en);
                //   // std::shared_ptr<EdgeRelation> new_edge_en = enhanced_struct->definition->initializeNewEdge();
                //   // new_edge_en->source_vertex =
                // }
              }
            }
          }
        }
      }
      // if(enhanced_base_graph->vertices_map_.size() > level.base_graph->vertices_map_.size())
      if (struct_enhanced)
      {
        std::string enhanced_name = substruct_name + "_en";
        pattern_structs_[enhanced_name] = enhanced_struct;
        level.enhanced_pattern_names.insert(enhanced_name);
        int biggest_instance_ind;
        int biggest_inst_size = 0;
        for (int ii = 0; ii < enhanced_struct->instances.size(); ++ii)
        {
          int size = (enhanced_struct->instances[ii].edge_ids.size() +
                      enhanced_struct->instances[ii].vertex_ids.size());
          if (size > biggest_inst_size)
          {
            biggest_inst_size = size;
            biggest_instance_ind = ii;
          }
        }
        enhanced_struct->definition = getDefinitionFromInstance(enhanced_struct->instances[biggest_instance_ind], enhanced_base_graph);
      }
    }
    // level.enhanced_base_graph = enhanced_base_graph;
    level.enhanced_base_graph = enhanced_base_graph->createCopy();

    for (auto v_it : level.enhanced_base_graph->vertices_map_)
    {
      // auto it = std::find_if(v_it.second->neighbor_map.begin(), v_it.second->neighbor_map.end(),
      //                        [level] (std::pair<int, int> &np)
      //                        {
      //                         return level.enhanced_base_graph->getVertex(np.first)->label == L_MANHOLE;
      //                        });
      if (level.enhanced_base_graph->getVertex(v_it.first)->label != L_MANHOLE)
        continue;

      bool manhole_neighbor_exists = false;
      for (auto it : v_it.second->neighbor_map)
      {
        if (level.enhanced_base_graph->getVertex(it.first)->label == L_MANHOLE)
        {
          manhole_neighbor_exists = true;
          break;
        }
      }
      if (!manhole_neighbor_exists) // Not connected to any other manhole vertex
      {
        std::shared_ptr<SemanticVertex> new_entry_vertex = enhanced_base_graph->initializeNewVertex();
        // new_entry_vertex->state = level.enhanced_base_graph->getVertex(v_id)->state;
        new_entry_vertex->state = v_it.second->state;
        new_entry_vertex->state(0) -= 0.2; // FOR VIS ONLY
        new_entry_vertex->pattern_name = NP;
        new_entry_vertex->label = L_MANHOLE;
        // new_entry_vertex->enhanced = true;
        new_entry_vertex->status = VertexStatus::kEnhanced;
        enhanced_base_graph->addVertex(new_entry_vertex);
        // std::cout << "New vertex added: " << new_entry_vertex->id << std::endl;
        // std::cout << "enhanced graph vertices: " << enhanced_base_graph->vertices_map_.size()
        //           << ", lvl enhanced graph vertices: " << level.enhanced_base_graph->vertices_map_.size() << std::endl;

        // Update edges
        std::shared_ptr<EdgeRelation> new_edge = enhanced_base_graph->initializeNewEdge();
        new_edge->source_vertex = enhanced_base_graph->getVertex(v_it.first);
        new_edge->target_vertex = new_entry_vertex;
        enhanced_base_graph->addEdge(new_edge);
        // std::cout << "New Edge added: " << new_edge->id << std::endl;

        if (v_it.second->neighbor_map.size() > 1)
        {
          enhanced_base_graph->getEdge((v_it.second->neighbor_map.begin())->second)->source_vertex = new_entry_vertex;
          enhanced_base_graph->getEdge((v_it.second->neighbor_map.begin())->second)->target_vertex = enhanced_base_graph->getVertex((v_it.second->neighbor_map.begin())->first);
          // std::cout << "Old edge updated" << std::endl;
          enhanced_base_graph->getVertex((v_it.second->neighbor_map.begin())->first)->neighbor_map.erase(v_it.first);
          enhanced_base_graph->getVertex((v_it.second->neighbor_map.begin())->first)->neighbor_map[new_entry_vertex->id] = (v_it.second->neighbor_map.begin())->second;
          enhanced_base_graph->getVertex(v_it.first)->neighbor_map.erase((v_it.second->neighbor_map.begin())->first);
          new_entry_vertex->neighbor_map[v_it.first] = (v_it.second->neighbor_map.begin())->second;
          // std::cout << "Neighbor maps updated" << std::endl;
        }
      }
    }
    level.enhanced_base_graph = enhanced_base_graph->createCopy();
    std::cout << "Adding enhanced base graph" << std::endl;
  }
}

void PredictivePlanning::extendGraphToStructs()
{
  std::vector<std::string> patterns_with_entry;
  for (auto it : pattern_structs_)
  {
    for (auto v_it : it.second->definition->vertices_map_)
    {
      if (v_it.second->label == L_MANHOLE)
      {
        patterns_with_entry.push_back(it.second->name);
        break;
      }
    }
  }

  for (int i = 0; i < current_hierarchy_->levels.size(); ++i)
  {
    SSGLevel &level = current_hierarchy_->levels[i];
    if(level.enhanced_base_graph == nullptr)
    {
      continue;
    }
    std::shared_ptr<SSGManager> extended_base_graph = level.enhanced_base_graph->createCopy();
    std::shared_ptr<PatternStructure> copied_struct;
    if (level.enhanced_pattern_names.empty())
    {
      continue;
    }

    std::shared_ptr<SSGManager> graph_to_insert;
    int best_overlap_global = -1;
    std::map<std::string, std::pair<int, std::shared_ptr<SSGManager>>> overlap_map; // pattern_name : <overlap, graph def>

    for (std::string pattern_name : level.enhanced_pattern_names)
    {
      copied_struct.reset(new PatternStructure(*pattern_structs_[pattern_name]));

      // for(auto v_it : extended_base_graph->vertices_map_)  // This is the correct way, the other is temp for testing
      int best_overlap_local = -1;
      std::shared_ptr<SSGManager> graph_to_insert_local;
      for (auto v_it : level.enhanced_base_graph->vertices_map_)
      {
        if (v_it.second->label == L_MANHOLE && v_it.second->pattern_name == NP && v_it.second->neighbor_map.size() < 2)
        {
          if((v_it.second->state.head(3) - comm_->planner_manager()->getCurrentRobotState().head(3)).norm() > 9.5)
            continue;
          bool at_least_one_added = false;
          // v_it.second->status = VertexStatus::kPotentialExtension;
          for (auto sv_it : copied_struct->definition->vertices_map_)
          {
            if (sv_it.second->label == L_MANHOLE)
            {
              copied_struct->transformWRTVertex(sv_it.first, copied_struct->definition);
              copied_struct->attachOnState(v_it.second->state);
              // extended_base_graph->insertGraph(copied_struct->definition, VertexStatus::kPredicted);
              int overlap = 0;
              for (auto sv_it2 : copied_struct->definition->vertices_map_) // Put this in a function
              {
                if(sv_it2.second->label == L_COMPARTMENT &&
                   (sv_it2.second->state.z() > config_->ssg_params.max_long_height || 
                    sv_it2.second->state.z() < config_->ssg_params.min_long_height))
                {
                  overlap = 0;
                  break;
                }
                std::vector<std::shared_ptr<SemanticVertex>> nearest_vertices;
                level.enhanced_base_graph->getNearestVerticesInRange(sv_it2.second->state, nearest_vertices, 0.4);
                bool positive_found = false, negative_found = false;
                for (auto v : nearest_vertices)
                {
                  if(v->pattern_name == NP)
                  {
                    if (v->label == sv_it2.second->label)
                    {
                      // ++overlap;
                      positive_found = true;
                    }
                  }
                  else
                  {
                    if(v->label == sv_it2.second->label)
                    {
                      if(v->label == L_WALL || v->label == L_MANHOLE)
                      {
                        continue;
                      }
                      else
                      {
                        negative_found = true;
                      }
                    }
                    else
                    {
                      negative_found = true;
                    }
                    // if not wall or manhole: overlap --
                    // if wall: if not opposite normals: overlap --
                  }

                  if(positive_found)
                    ++overlap;
                  else if(negative_found)
                    --overlap;
                  // if (v->label == sv_it2.second->label && v_it.second->id != v->id && sv_it2.second->pattern_name == NP)
                  // {
                  //   std::cout << "Overlaps with " << v->id << " " << v->label << std::endl;
                  //   ++overlap;
                  //   break;
                  // }
                  // else
                  // {
                  //   if(v_it.second->id != v->id)
                  //   {
                  //     // if()
                  //     --overlap;  // If there is another vertex nearby reduce the overlap count
                  //   }
                  // }
                }
              }
              if (overlap > best_overlap_global)
              {
                best_overlap_global = overlap;
                graph_to_insert = copied_struct->definition->createCopy();
              }
              if (overlap > best_overlap_local)
              {
                best_overlap_local = overlap;
                graph_to_insert_local = copied_struct->definition->createCopy();
              }
              // at_least_one_added = true;
            }
          }
          // if(at_least_one_added) break;
        }
      }
      overlap_map[pattern_name] = std::make_pair(best_overlap_local, graph_to_insert_local);
    }
    extended_base_graph->insertGraph(graph_to_insert, VertexStatus::kPredicted);
    level.enhanced_base_graph = extended_base_graph->createCopy(); // TEMP: TODO: There should be another field.
  }

  std::cout << "Graph extended" << std::endl;
  visualizeHierarchy(*current_hierarchy_, true);
}

void PredictivePlanning::visualizeHierarchy(SSGHierarchy h, bool enhanced)
{
  visualization_msgs::MarkerArray combined_graph_vis;
  for (int i = 0; i < h.levels.size(); ++i)
  {
    SSGLevel level = h.levels[i];
    // Base graph
    visualization_msgs::MarkerArray graph_vis;
    if (enhanced)
      graph_vis = level.enhanced_base_graph->getGraphVis();
    else
      graph_vis = level.base_graph->getGraphVis();
    Eigen::Vector3d offset(0.0, 0.0, 9.0 * i);
    if (enhanced)
      offset(1) = 9.0;
    offsetGraphVis(graph_vis, offset);
    smartInsert(combined_graph_vis, graph_vis);
    // Substructs
    std::set<std::string> pattern_names;
    if (enhanced)
    {
      if (!level.enhanced_pattern_names.empty())
        pattern_names = level.enhanced_pattern_names;
      else
        pattern_names = level.discovered_pattern_names;
    }
    else
      pattern_names = level.discovered_pattern_names;
    int j = 0;
    std::cout << "Num patterns: " << pattern_names.size() << std::endl;
    if (!pattern_names.empty())
    {
      for (auto name : pattern_names)
      {
        visualization_msgs::MarkerArray struct_vis;
        struct_vis = pattern_structs_[name]->definition->getGraphVis();
        Eigen::Vector3d offset(0.0, -20.0 * (j + 1), 20.0 * i);
        offsetGraphVis(struct_vis, offset);
        smartInsert(combined_graph_vis, struct_vis);
        ++j;
      }
    }
  }

  hierarchy_vis_pub_.publish(combined_graph_vis);
}

int PredictivePlanning::getIntLabel(std::string string_label)
{
  auto it = label_string_to_int_.find(string_label);
  if (it == label_string_to_int_.end())
  {
    substruct_label_int_ += 30;
    int new_label_int = substruct_label_int_;
    label_string_to_int_[string_label] = new_label_int;
    label_int_to_string_[substruct_label_int_] = string_label;
    return new_label_int;
  }
  else
  {
    return label_string_to_int_[string_label];
  }
}

std::shared_ptr<SSGManager> PredictivePlanning::getDefinitionFromInstance(PatternInstance inst, std::shared_ptr<SSGManager> base_graph)
{
  std::shared_ptr<SSGManager> new_definition;
  new_definition.reset(new SSGManager());

  std::map<int, int> inst_to_def_vids;
  for (int v_id : inst.vertex_ids)
  {
    std::shared_ptr<SemanticVertex> new_vertex = new_definition->initializeNewVertex();
    new_vertex->state = base_graph->getVertex(v_id)->state;
    new_vertex->label = base_graph->getVertex(v_id)->label;

    new_definition->addVertex(new_vertex);

    inst_to_def_vids[v_id] = new_vertex->id;
  }

  // for (auto it : inst_to_def_vids)
  // {
  //   std::cout << it.first << " : " << it.second << std::endl;
  // }

  // std::cout << "Num edges in instance: " << inst.edge_ids.size() << std::endl;
  // std::cout << "Edge map:" << std::endl;
  // for (auto e_it : base_graph->edge_map_)
  // {
  //   std::cout << e_it.first << std::endl;
  // }
  // std::cout << "Edge ids:" << std::endl;
  // for (int i = 0; i < inst.edge_ids.size(); ++i)
  // {
  //   std::cout << " " << inst.edge_ids[i] << std::endl; // << ": " << base_graph->getEdge(inst.edge_ids[i])->id
  // }
  // std::cout << "Edges:" << std::endl;
  // for (int i = 0; i < inst.edge_ids.size(); ++i)
  // {
  //   std::cout << " " << inst.edge_ids[i]
  //             << ": " << base_graph->getEdge(inst.edge_ids[i])->source_vertex->id
  //             << " <-> " << base_graph->getEdge(inst.edge_ids[i])->target_vertex->id << std::endl;
  // }
  // for(int e_id : inst.edge_ids)
  for (int i = 0; i < inst.edge_ids.size(); ++i)
  {
    // std::cout << "Edge: " << i << ": ";
    int e_id = inst.edge_ids[i];
    // std::cout << e_id;
    // std::cout << " " << base_graph->getEdge(e_id)->source_vertex->id << " : " << base_graph->getEdge(e_id)->target_vertex->id << std::endl;
    std::shared_ptr<EdgeRelation> new_edge = new_definition->initializeNewEdge();
    new_edge->source_vertex =
        new_definition->getVertex(inst_to_def_vids[base_graph->getEdge(e_id)->source_vertex->id]);
    new_edge->target_vertex =
        new_definition->getVertex(inst_to_def_vids[base_graph->getEdge(e_id)->target_vertex->id]);
    new_edge->label = base_graph->getEdge(e_id)->label;
    new_edge->weight = base_graph->getEdge(e_id)->weight;
    // std::cout << "edge updated" << std::endl;
    new_definition->addEdge(new_edge);
    // std::cout << "edge added" << std::endl;
  }

  // for(int i=0; i<inst.edge_ids.size(); ++i)
  // {
  //   std::cout << "Edge: ";
  //   std::cout << i << ": ";
  //   ros::Duration(0.01).sleep();
  //   int e_id = inst.edge_ids[i];
  //   std::cout << e_id;
  //   std::cout << " -> " << base_graph->getEdge(e_id)->source_vertex->id << " : " << base_graph->getEdge(e_id)->target_vertex->id << std::endl;
  //   std::cout << "Definition ids: " << std::endl;
  //   std::cout << inst_to_def_vids[base_graph->getEdge(e_id)->source_vertex->id]
  //             << " : " << inst_to_def_vids[base_graph->getEdge(e_id)->target_vertex->id] << std::endl;
  //   std::cout << "Label: " << base_graph->getEdge(e_id)->label << std::endl;
  //   std::cout << "Weight: " << base_graph->getEdge(e_id)->weight << std::endl;
  // }
  // std::cout << "Definition constructed" << std::endl;

  return new_definition;
}

std::shared_ptr<SSGManager> PredictivePlanning::getPredictedSubgraph()
{
  std::shared_ptr<SSGManager> extended_graph;
  extended_graph.reset(new SSGManager);

  int extension_level = 0;
  bool extension_found = false;
  for(int li=0; li<current_hierarchy_->levels.size(); ++li)
  {
    for(auto v_it : current_hierarchy_->levels[li].enhanced_base_graph->vertices_map_)
    {
      if(v_it.second->status == VertexStatus::kPredicted && !v_it.second->used)
      {
        // std::cout << "Found predicted vertex: " << v_it.second->state.transpose() << std::endl;
        extension_found = true;
        v_it.second->used = true;
        std::shared_ptr<SemanticVertex> new_vertex = extended_graph->initializeNewVertex();
        v_it.second->copyTo(new_vertex, false);
        extended_graph->addVertex(new_vertex);
        for(auto n_it : v_it.second->neighbor_map)
        {
          std::shared_ptr<SemanticVertex> nv 
            = current_hierarchy_->levels[li].enhanced_base_graph->getVertex(n_it.first);
          if(!nv->used && nv->status == VertexStatus::kPredicted)
          {
            nv->used = true;
            std::shared_ptr<SemanticVertex> new_nb_vertex = extended_graph->initializeNewVertex();
            nv->copyTo(new_nb_vertex, false);
            extended_graph->addVertex(new_nb_vertex);

            std::shared_ptr<EdgeRelation> ne = current_hierarchy_->levels[li].enhanced_base_graph->getEdge(n_it.second);

            std::shared_ptr<EdgeRelation> new_edge = extended_graph->initializeNewEdge();
            new_edge->source_vertex = new_vertex;
            new_edge->target_vertex = nv;
            new_edge->label = ne->label;
            new_edge->weight = ne->weight;
            extended_graph->addEdge(new_edge);
          }
        }
      }
    }

    if(extension_found)
    {
      extension_level = li;
      break;
    }
  }

  // std::cout << "Extended graph extracted. Extension found: " << extension_found << std::endl;

  if(extension_level > 0 && extension_found)  // Need to insert the substructs
  {
    // std::cout << "Need to add structs" << std::endl;
    SSGLevel &level = current_hierarchy_->levels[extension_level];
    std::shared_ptr<SSGManager> extended_graph_copy;
    extended_graph_copy = extended_graph->createCopy();
    for(auto v_it : extended_graph->vertices_map_)
    {
      if(v_it.second->label >= config_->ssg_params.substruct_label_start)
      {
        // std::cout << "Vertex " << v_it.first << " needs decompression | " << label_int_to_string_[v_it.second->label] << std::endl;
        auto it = pattern_structs_.find(label_int_to_string_[v_it.second->label]);
        if(it == pattern_structs_.end()) 
        {
          ROS_ERROR("This pattern does not exist in pattern_structs_");
          continue;
        }
        std::shared_ptr<PatternStructure> pattern_struct;
        pattern_struct.reset(new PatternStructure(*pattern_structs_[label_int_to_string_[v_it.second->label]]));
        Eigen::Vector6d pattern_state;
        Eigen::Vector3d mean_pos;
        mean_pos << 0.0, 0.0, 0.0;
        // pattern_state << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        bool comp_found = false, wall_found = false;
        int comp_ind, wall_ind;
        for(auto sv_it : pattern_struct->definition->vertices_map_)
        {
          if(sv_it.second->label == L_COMPARTMENT)
          {
            comp_found = true;
            comp_ind = sv_it.first;
          }
          if(sv_it.second->label == L_WALL)
          {
            wall_found = true;
            wall_ind = sv_it.first;
          }
          mean_pos += sv_it.second->state.head(3);
        }
        mean_pos /= pattern_struct->definition->vertices_map_.size();
        
        if(comp_found)
        {
          pattern_state = pattern_struct->definition->vertices_map_[comp_ind]->state;
        }
        else if(wall_found)
        {
          pattern_state = pattern_struct->definition->vertices_map_[wall_ind]->state;
        }
        else
        {
          pattern_state = pattern_struct->definition->vertices_map_.begin()->second->state;
        }
        pattern_state.head(3) = mean_pos;
        // pattern_state /= pattern_struct->definition->vertices_map_.size();
        // std::cout << "1" << std::endl;
        pattern_struct->transformWRTState(pattern_state);
        // std::cout << "2" << std::endl;
        pattern_struct->attachOnState(v_it.second->state);
        // std::cout << "3" << std::endl;
        extended_graph_copy->insertGraph(pattern_struct->definition, VertexStatus::kPredicted);
        // std::cout << "4" << std::endl;
      }
    }
    extended_graph = extended_graph_copy;
  }

  std::cout << "Num vertices extended predicted graph: "
            << extended_graph->vertices_map_.size() << std::endl;
  for(auto v_it : extended_graph->vertices_map_)
  {
    std::cout << v_it.first << " label: " << v_it.second->label 
    << " pos: " << v_it.second->state.head(3).transpose()
    << " ori: " << v_it.second->state.tail(3).transpose() << std::endl;
  }

  visualization_msgs::MarkerArray extended_graph_vis = extended_graph->getGraphVis();
  // offsetGraphVis(extended_graph_vis, Eigen::Vector3d(0.0, 5.0, 0.0));
  predicted_subgraph_vis_pub_.publish(extended_graph_vis);

  return extended_graph;

}