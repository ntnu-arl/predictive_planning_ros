#include "graph/ssg_manager.hpp"

SSGManager::SSGManager()
{
  ssg_.reset(new BaseGraph);
  id_count_ = 0;
  edge_id_count_ = 0;
}

SSGManager::SSGManager(SSGManager &_ssg)
{
  ssg_.reset(new BaseGraph(*(_ssg.ssg_)));
  // std::cout << "0" << std::endl;
  id_count_ = _ssg.id_count_;
  edge_id_count_ = _ssg.edge_id_count_;

  ssg_to_subdue_id_map_ = _ssg.ssg_to_subdue_id_map_;

  vertices_map_.clear();
  for(auto v_it : _ssg.vertices_map_)
  {
    std::shared_ptr<SemanticVertex> new_vertex;
    new_vertex.reset(new SemanticVertex(*(v_it.second)));
    vertices_map_[v_it.first] = new_vertex;
  }
  // std::cout << "v set" << std::endl;

  edge_map_.clear();
  for(auto e_it : _ssg.edge_map_)
  {
    std::shared_ptr<EdgeRelation> new_edge;
    new_edge.reset(new EdgeRelation(*(e_it.second)));
    edge_map_[e_it.first] = new_edge;
  }
  // std::cout << "e set" << std::endl;
}

std::shared_ptr<SemanticVertex> SSGManager::initializeNewVertex()
{
  std::shared_ptr<SemanticVertex> new_vertex(new SemanticVertex);
  new_vertex->id = id_count_++;

  return new_vertex;
}

int SSGManager::getNextVertexID()
{
  return ++id_count_;
}

std::shared_ptr<SemanticVertex> SSGManager::addNewVertex()
{
  std::shared_ptr<SemanticVertex> new_vertex(new SemanticVertex);
  new_vertex->id = ++id_count_;
  addVertex(new_vertex);

  return new_vertex;
}

void SSGManager::addVertex(std::shared_ptr<SemanticVertex> v)
{
  /* Add to KD tree */
  ssg_->addVertex(v->id);
  vertices_map_[v->id] = v;
}

bool SSGManager::removeVertex(std::shared_ptr<SemanticVertex> v)
{
  /* Remove from KD tree */
  if(vertices_map_[v->id] == NULL)
    return false;

  ssg_->removeVertex(v->id);
  vertices_map_[v->id] = NULL;
  return true;
}

std::shared_ptr<EdgeRelation> SSGManager::initializeNewEdge()
{
  std::shared_ptr<EdgeRelation> new_edge(new EdgeRelation);
  new_edge->id = edge_id_count_++;

  return new_edge;
}

void SSGManager::addEdge(std::shared_ptr<EdgeRelation> e)
{
  ssg_->addEdge(e->source_vertex->id, e->target_vertex->id, e->weight);
  edge_map_[e->id] = e;

  e->source_vertex->neighbor_map[e->target_vertex->id] = e->id;
  e->target_vertex->neighbor_map[e->source_vertex->id] = e->id;
  // e->source_vertex->neighbors.push_back(std::make_pair(e->id, e->target_vertex->id));
  // e->target_vertex->neighbors.push_back(std::make_pair(e->id, e->source_vertex->id));
}

bool SSGManager::removeEdge(std::shared_ptr<EdgeRelation> e)
{
  if(edge_map_[e->id] == NULL)
    return false;
  
  int e_id = e->id;

  //// Removing from neighbor list of end vertices
  e->source_vertex->neighbor_map.erase(e->target_vertex->id);
  e->target_vertex->neighbor_map.erase(e->source_vertex->id);
  // std::vector<std::pair<int, int>>::iterator source_it 
  //   = std::find_if(e->source_vertex->neighbors.begin(), 
  //                  e->source_vertex->neighbors.end(),
  //                  [&e_id](const std::pair<int, int> &p) {
  //                   return p.first == e_id;
  //                  });
  // if(source_it == e->source_vertex->neighbors.end())
  //   return false;
  // e->source_vertex->neighbors.erase(source_it);

  // std::vector<std::pair<int, int>>::iterator target_it 
  //   = std::find_if(e->target_vertex->neighbors.begin(), 
  //                  e->target_vertex->neighbors.end(),
  //                  [&e_id](const std::pair<int, int> &p) {
  //                   return p.first == e_id;
  //                  });
  // if(target_it == e->target_vertex->neighbors.end())
  //   return false;
  // e->target_vertex->neighbors.erase(target_it);

  ssg_->removeEdge(e->source_vertex->id, e->target_vertex->id);
  // edge_map_[e->id] = NULL;
  edge_map_.erase(e->id);

  return true;
}

bool SSGManager::getNearestVerticesInRange(std::shared_ptr<SemanticVertex> v, std::vector<std::shared_ptr<SemanticVertex>> &nearest_vertices, double range)
{
  nearest_vertices.clear();
  std::vector<std::pair<std::shared_ptr<SemanticVertex>, double>> nearest_vertices_dist;
  for(auto v_it : vertices_map_)
  {
    if(v_it.second->id != v->id)
    {
      double dist = (v->state.head(3) - v_it.second->state.head(3)).norm();
      if(dist <= range)
      {
        // nearest_vertices.push_back(v_it.second);
        nearest_vertices_dist.push_back(std::make_pair(v_it.second, dist));
      }
    }
  }

  std::sort(nearest_vertices_dist.begin(), nearest_vertices_dist.end(), 
            [](const std::pair<std::shared_ptr<SemanticVertex>, double> &a, std::pair<std::shared_ptr<SemanticVertex>, double> &b)
            {
              return a.second < b.second;
            });

  for(auto it : nearest_vertices_dist)
  {
    nearest_vertices.push_back(it.first);
  }
  return true;
}

bool SSGManager::getNearestVerticesInRange(Eigen::Vector6d s, std::vector<std::shared_ptr<SemanticVertex>> &nearest_vertices, double range)
{
  nearest_vertices.clear();
  std::vector<std::pair<std::shared_ptr<SemanticVertex>, double>> nearest_vertices_dist;
  for(auto v_it : vertices_map_)
  {
    double dist = (s.head(3) - v_it.second->state.head(3)).norm();
    if(dist <= range)
    {
      // nearest_vertices.push_back(v_it.second);
      nearest_vertices_dist.push_back(std::make_pair(v_it.second, dist));
    }
  }

  std::sort(nearest_vertices_dist.begin(), nearest_vertices_dist.end(), 
            [](const std::pair<std::shared_ptr<SemanticVertex>, double> &a, std::pair<std::shared_ptr<SemanticVertex>, double> &b)
            {
              return a.second < b.second;
            });

  for(auto it : nearest_vertices_dist)
  {
    nearest_vertices.push_back(it.first);
  }
  return true;
}

visualization_msgs::MarkerArray SSGManager::getGraphVis()
{
  visualization_msgs::MarkerArray marker_array;

  // Plot all edges
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = "world";
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(0.0);
  edge_marker.frame_locked = false;

  for (auto e_it : edge_map_) {
    geometry_msgs::Point p1;
    p1.x = e_it.second->source_vertex->state[0];
    p1.y = e_it.second->source_vertex->state[1];
    p1.z = e_it.second->source_vertex->state[2];
    geometry_msgs::Point p2;
    p2.x = e_it.second->target_vertex->state[0];
    p2.y = e_it.second->target_vertex->state[1];
    p2.z = e_it.second->target_vertex->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  if(!edge_marker.points.empty())
    marker_array.markers.push_back(edge_marker);

  // Plot regular vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = "world";
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.9;
  vertex_marker.scale.y = 0.9;
  vertex_marker.scale.z = 0.9;
  // vertex_marker.color.r = 125.0 / 255.0;
  // vertex_marker.color.g = 42.0 / 255.0;
  // vertex_marker.color.b = 104.0 / 255.0;
  // vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(0.0);
  vertex_marker.frame_locked = false;

  int marker_id = 0;
  for (auto v_it : vertices_map_) {
    if(v_it.second->status != VertexStatus::kTrue) continue;

    geometry_msgs::Point p1;
    p1.x = v_it.second->state[0];
    p1.y = v_it.second->state[1];
    p1.z = v_it.second->state[2];
    std_msgs::ColorRGBA rgb_msg;
    Eigen::Vector3i rgb = getColor(v_it.second->label);
    rgb_msg.r = (double)rgb[0] / 255.0;
    rgb_msg.g = (double)rgb[1] / 255.0;
    rgb_msg.b = (double)rgb[2] / 255.0;
    rgb_msg.a = 1.0;
    vertex_marker.colors.push_back(rgb_msg);
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  // potential_extension vertices
  visualization_msgs::Marker potential_extension_vertex_marker;
  potential_extension_vertex_marker.header.stamp = ros::Time::now();
  potential_extension_vertex_marker.header.seq = 0;
  potential_extension_vertex_marker.header.frame_id = "world";
  potential_extension_vertex_marker.id = 0;
  potential_extension_vertex_marker.ns = "potential_extension_vertices";
  potential_extension_vertex_marker.action = visualization_msgs::Marker::ADD;
  potential_extension_vertex_marker.type = visualization_msgs::Marker::CUBE_LIST;
  potential_extension_vertex_marker.scale.x = 0.3;
  potential_extension_vertex_marker.scale.y = 0.3;
  potential_extension_vertex_marker.scale.z = 0.3;
  potential_extension_vertex_marker.lifetime = ros::Duration(0.0);
  potential_extension_vertex_marker.frame_locked = false;
  for (auto v_it : vertices_map_) {
    if(v_it.second->status != VertexStatus::kPotentialExtension) continue;
    geometry_msgs::Point p1;
    p1.x = v_it.second->state[0];
    p1.y = v_it.second->state[1];
    p1.z = v_it.second->state[2];
    std_msgs::ColorRGBA rgb_msg;
    Eigen::Vector3i rgb = getColor(v_it.second->label);
    rgb_msg.r = (double)rgb[0] / 255.0;
    rgb_msg.g = (double)rgb[1] / 255.0;
    rgb_msg.b = (double)rgb[2] / 255.0;
    rgb_msg.a = 0.5;
    potential_extension_vertex_marker.colors.push_back(rgb_msg);
    potential_extension_vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(potential_extension_vertex_marker);

  // Enhanced vertices
  visualization_msgs::Marker enhanced_vertex_marker;
  enhanced_vertex_marker.header.stamp = ros::Time::now();
  enhanced_vertex_marker.header.seq = 0;
  enhanced_vertex_marker.header.frame_id = "world";
  enhanced_vertex_marker.id = 0;
  enhanced_vertex_marker.ns = "enhanced_vertices";
  enhanced_vertex_marker.action = visualization_msgs::Marker::ADD;
  enhanced_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  enhanced_vertex_marker.scale.x = 0.6;
  enhanced_vertex_marker.scale.y = 0.1;
  enhanced_vertex_marker.scale.z = 0.3;
  enhanced_vertex_marker.lifetime = ros::Duration(0.0);
  enhanced_vertex_marker.frame_locked = false;
  for (auto v_it : vertices_map_) {
    if(v_it.second->status != VertexStatus::kEnhanced) continue;
    geometry_msgs::Point p1;
    p1.x = v_it.second->state[0];
    p1.y = v_it.second->state[1];
    p1.z = v_it.second->state[2];
    std_msgs::ColorRGBA rgb_msg;
    Eigen::Vector3i rgb = getColor(v_it.second->label);
    rgb_msg.r = (double)rgb[0] / 255.0;
    rgb_msg.g = (double)rgb[1] / 255.0;
    rgb_msg.b = (double)rgb[2] / 255.0;
    rgb_msg.a = 1.0;
    enhanced_vertex_marker.colors.push_back(rgb_msg);
    enhanced_vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(enhanced_vertex_marker);

  // Predicted vertices
  visualization_msgs::Marker predicted_vertex_marker;
  predicted_vertex_marker.header.stamp = ros::Time::now();
  predicted_vertex_marker.header.seq = 0;
  predicted_vertex_marker.header.frame_id = "world";
  predicted_vertex_marker.id = 0;
  predicted_vertex_marker.ns = "predicted_vertices";
  predicted_vertex_marker.action = visualization_msgs::Marker::ADD;
  predicted_vertex_marker.type = visualization_msgs::Marker::CUBE_LIST;
  predicted_vertex_marker.scale.x = 0.3;
  predicted_vertex_marker.scale.y = 0.3;
  predicted_vertex_marker.scale.z = 0.3;
  predicted_vertex_marker.lifetime = ros::Duration(0.0);
  predicted_vertex_marker.frame_locked = false;
  for (auto v_it : vertices_map_) {
    if(v_it.second->status != VertexStatus::kPredicted) continue;
    geometry_msgs::Point p1;
    p1.x = v_it.second->state[0];
    p1.y = v_it.second->state[1];
    p1.z = v_it.second->state[2];
    std_msgs::ColorRGBA rgb_msg;
    Eigen::Vector3i rgb = getColor(v_it.second->label);
    rgb_msg.r = (double)rgb[0] / 255.0;
    rgb_msg.g = (double)rgb[1] / 255.0;
    rgb_msg.b = (double)rgb[2] / 255.0;
    rgb_msg.a = 0.5;
    predicted_vertex_marker.colors.push_back(rgb_msg);
    predicted_vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(predicted_vertex_marker);

  for(auto v_it : vertices_map_)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.header.frame_id = "world";
    marker.ns = "headings";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.3;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    std_msgs::ColorRGBA rgb_msg;
    Eigen::Vector3i rgb = getColor(v_it.second->label);
    marker.color.r = (float)rgb[0] / 255.0;
    marker.color.g = (float)rgb[1] / 255.0;
    marker.color.b = (float)rgb[2] / 255.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = false;
    convert(v_it.second->state, marker.pose);
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);
  }

  for(auto v_it : vertices_map_)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.header.frame_id = "world";
    marker.ns = "ids";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 0.15;  // text height
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = false;
    marker.pose.position.x = v_it.second->state[0];
    marker.pose.position.y = v_it.second->state[1];
    marker.pose.position.z = v_it.second->state[2] + 0.1;
    // Show vertex gains.
    std::string pattern_str;
    (v_it.second->pattern_name != NP) ? pattern_str = v_it.second->pattern_name + "." + std::to_string(v_it.second->instance_ind) : pattern_str = "";
    std::string text_display =
        std::to_string(v_it.second->id) + "," 
        + std::to_string(v_it.second->label) + "," 
        + std::to_string(v_it.second->associated_compartment_vertex_id) + ","
        + pattern_str;

    marker.text = text_display;
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

predictive_planner_common::Graph SSGManager::getGraphMsg()
{
  predictive_planner_common::Graph graph_msg;
  for(auto v_it : vertices_map_)
  {
    predictive_planner_common::Vertex v_msg;
    v_msg.id = v_it.first;
    v_msg.label = v_it.second->label;
    v_msg.pattern_name.data = v_it.second->pattern_name;
    tf::Quaternion quat;
    quat.setEuler(v_it.second->state(3), v_it.second->state(4), v_it.second->state(5));
    tf::Vector3 origin(v_it.second->state(0), v_it.second->state(1), v_it.second->state(2));
    tf::Pose poseTF(quat, origin);
    tf::poseTFToMsg(poseTF, v_msg.pose);
    graph_msg.vertices.push_back(v_msg);
  }

  for(auto e_it : edge_map_)
  {
    predictive_planner_common::Edge e_msg;
    e_msg.id = e_it.second->id;
    e_msg.label = e_it.second->label;
    e_msg.source_id = e_it.second->source_vertex->id;
    e_msg.target_id = e_it.second->target_vertex->id;
    e_msg.weight = e_it.second->weight;
    graph_msg.edges.push_back(e_msg);
  }

  return graph_msg;
}

void SSGManager::saveGraph(std::string filename)
{
  predictive_planner_common::Graph graph_msg = this->getGraphMsg();
  // Serialize the data, then write to a file
  uint32_t serial_size = ros::serialization::serializationLength(graph_msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, graph_msg);

  // Write to a file
  std::ofstream wrt_file(filename, std::ios::out | std::ios::binary);
  wrt_file.write((char*)buffer.get(), serial_size);
  wrt_file.close();
}

void SSGManager::loadGraph(std::string filename)
{
  // Fill buffer with a serialized UInt32
  std::ifstream read_file(filename, std::ifstream::binary);
  // Get the total number of bytes:
  // http://www.cplusplus.com/reference/fstream/ifstream/rdbuf/
  std::filebuf* pbuf = read_file.rdbuf();
  std::size_t size = pbuf->pubseekoff(0, read_file.end, read_file.in);
  pbuf->pubseekpos(0, read_file.in);
  // Read the whole file to a buffer
  boost::shared_array<uint8_t> buffer(new uint8_t[size]);
  // char* buffer1=new char[size];
  pbuf->sgetn((char*)buffer.get(), size);
  read_file.close();

  // Deserialize data into msg
  predictive_planner_common::Graph graph_msg;
  ros::serialization::IStream stream_in(buffer.get(), size);
  ros::serialization::deserialize(stream_in, graph_msg);

  ssg_.reset(new BaseGraph);
  id_count_ = 0;
  std::cout << "SSG reset" << std::endl;

  convertMsgToGraph(graph_msg);
}

void SSGManager::convertMsgToGraph(predictive_planner_common::Graph graph_msg)
{
  int v_id_count = 0;
  for(predictive_planner_common::Vertex v_msg : graph_msg.vertices)
  {
    // if(v_msg.id == 109) 
    // {
    //   ROS_WARN("Skipping 109");
    //   continue;
    // }
    std::shared_ptr<SemanticVertex> new_vertex;
    new_vertex.reset(new SemanticVertex());
    new_vertex->id = v_msg.id;
    new_vertex->label = v_msg.label;
    new_vertex->pattern_name = v_msg.pattern_name.data;
    new_vertex->state(0) = v_msg.pose.position.x;
    new_vertex->state(1) = v_msg.pose.position.y;
    new_vertex->state(2) = v_msg.pose.position.z;
    new_vertex->state(5) = tf::getYaw(v_msg.pose.orientation);
    addVertex(new_vertex);
    ++v_id_count;
  }
  id_count_ = graph_msg.vertices.size();
  // id_count_ = v_id_count;
  std::cout << "Vertices loaded" << std::endl;

  int e_i_count = 0;
  for(predictive_planner_common::Edge e_msg : graph_msg.edges)
  {
    // if(e_msg.target_id == 109 || e_msg.source_id == 109) continue;
    std::shared_ptr<EdgeRelation> new_edge;
    new_edge.reset(new EdgeRelation());
    new_edge->id = e_msg.id;
    new_edge->label = e_msg.label;
    new_edge->weight = e_msg.weight;
    if(e_msg.id == 11) std::cout << e_msg.source_id << " " << e_msg.target_id << std::endl;
    new_edge->source_vertex = getVertex(e_msg.source_id);
    new_edge->target_vertex = getVertex(e_msg.target_id);
    getVertex(e_msg.source_id)->neighbor_map[e_msg.target_id] = e_msg.id;
    getVertex(e_msg.target_id)->neighbor_map[e_msg.source_id] = e_msg.id;
    addEdge(new_edge);
    ++e_i_count;
  }
  edge_id_count_ = graph_msg.edges.size();
  // edge_id_count_ = e_i_count;
  std::cout << "Edges loaded" << std::endl;
}

std::shared_ptr<SSGManager> SSGManager::createCopy()
{
  std::shared_ptr<SSGManager> ssg_copy;
  ssg_copy = std::make_shared<SSGManager>();

  ssg_copy->id_count_ = id_count_;
  ssg_copy->edge_id_count_ = edge_id_count_;

  ssg_copy->ssg_to_subdue_id_map_ = ssg_to_subdue_id_map_;

  ssg_copy->vertices_map_.clear();
  for(auto v_it : vertices_map_)
  {
    std::shared_ptr<SemanticVertex> new_vertex;
    new_vertex.reset(new SemanticVertex(*(v_it.second)));
    ssg_copy->vertices_map_[v_it.first] = new_vertex;
  }

  ssg_copy->edge_map_.clear();
  for(auto e_it : edge_map_)
  {
    std::shared_ptr<EdgeRelation> new_edge;
    new_edge.reset(new EdgeRelation(*(e_it.second)));
    ssg_copy->edge_map_[e_it.first] = new_edge;
  }

  return ssg_copy;
}

void SSGManager::insertGraph(std::shared_ptr<SSGManager> in_graph)
{
  std::map<int, int> vid_in_graph_to_this;
  for(auto v_it : in_graph->vertices_map_)
  {
    std::shared_ptr<SemanticVertex> new_vertex = initializeNewVertex();
    // new_vertex->local_id = v_it.second->local_id;
    // new_vertex->state = v_it.second->state;
    // new_vertex->label = v_it.second->label;
    // new_vertex->pattern_name = v_it.second->pattern_name;
    // new_vertex->instance_ind = v_it.second->instance_ind;
    // new_vertex->status = v_it.second->status;
    v_it.second->copyTo(new_vertex, false);
    vid_in_graph_to_this[v_it.first] = new_vertex->id;
    addVertex(new_vertex);
  }

  for(auto e_it : in_graph->edge_map_)
  {
    std::shared_ptr<EdgeRelation> new_edge = initializeNewEdge();
    new_edge->source_vertex = getVertex(vid_in_graph_to_this[e_it.second->source_vertex->id]);
    new_edge->target_vertex = getVertex(vid_in_graph_to_this[e_it.second->target_vertex->id]);
    new_edge->label = e_it.second->label;
    new_edge->weight = e_it.second->weight;
    addEdge(new_edge);
  }
}

void SSGManager::insertGraph(std::shared_ptr<SSGManager> in_graph, VertexStatus fixed_status)
{
  std::map<int, int> vid_in_graph_to_this;
  for(auto v_it : in_graph->vertices_map_)
  {
    std::shared_ptr<SemanticVertex> new_vertex = initializeNewVertex();
    // new_vertex->local_id = v_it.second->local_id;
    // new_vertex->state = v_it.second->state;
    // new_vertex->label = v_it.second->label;
    // new_vertex->pattern_name = v_it.second->pattern_name;
    // new_vertex->instance_ind = v_it.second->instance_ind;
    v_it.second->copyTo(new_vertex, false);
    new_vertex->status = fixed_status;
    vid_in_graph_to_this[v_it.first] = new_vertex->id;
    new_vertex->state.x() += 0.15;
    addVertex(new_vertex);
  }

  for(auto e_it : in_graph->edge_map_)
  {
    std::shared_ptr<EdgeRelation> new_edge = initializeNewEdge();
    new_edge->source_vertex = getVertex(vid_in_graph_to_this[e_it.second->source_vertex->id]);
    new_edge->target_vertex = getVertex(vid_in_graph_to_this[e_it.second->target_vertex->id]);
    new_edge->label = e_it.second->label;
    new_edge->weight = e_it.second->weight;
    addEdge(new_edge);
  }
}