#pragma once

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/Float32MultiArray.h>

#include "common/utils.hpp"
#include "common/communicator.hpp"
#include "graph/ssg_manager.hpp"
#include "gbplanner/gbplanner.h"
#include "predictive_planning.hpp"
#include "bwt_ssg_builder/bwt_ssg_manager.hpp"

enum PlannerState
{
  LOCAL_EXPLORATION=0,
  INSPECTION,
  MH_PHASE_1,
  MH_CHECK,
  MH_PHASE_2,
  PATTERN_DET,
  VERIFICATION,
  ASSISTED_EXPLORATION,
  ASSISTED_INSPECTION,
  HOMING_P1,
  HOMING_CHECK,
  HOMING_P2,
  IDLE
};

enum VerificationState
{
  VERIFICATION_PATH=0,
  INSPECTION_PATH,
  VERIFICATION_WAIT,
  NONE
};

class PlannerManager
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  std::shared_ptr<Communicator> comm_;
  std::shared_ptr<Config> config_;

  ros::ServiceServer planner_srv_;
  ros::ServiceServer manhole_homing_srv_;
  ros::ServiceServer blind_homing_srv_;
  ros::ServiceServer get_prediction_srv_;

  ros::Subscriber odom_sub_;

  ros::Publisher predicted_inspection_path_pub_;
  ros::Publisher per_comp_insp_time_pub_;

  PlannerState planner_state_;
  VerificationState verification_state_;
  
  std::vector<StateVec> verification_viewpoints_;
  std::shared_ptr<SSGManager> predicted_subgraph_;

  StateVec current_robot_state_;
  std::vector<StateVec> predicted_inspection_viewpoints_;

  int current_compartment_id_, previous_compartment_id_;
  int compartment_counter_ = 0;
  int local_exploration_steps_ = 0, assisted_exploration_steps_ = 0;
  int local_exploration_fails_ = 0, assisted_exploration_fails_ = 0;
  int opp_assist_exploration_steps_ = 0;
  bool skip_opp_viewpoint_ = false;
  bool full_overlap = false;
  bool first_inspection_ = false;
  std::vector<geometry_msgs::Pose> full_overlap_path_;
  std::vector<int> homing_manhole_traversal_order_;
  std::vector<int> traversed_manholes_;
  int mh_under_execution_;

  bool start_insp_timer_ = true;
  double insp_timer_start_;

public:
  PlannerManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, std::shared_ptr<Communicator> comm, std::shared_ptr<Config> config);
  bool plannerServiceCallback( planner_msgs::planner_srv::Request& req, planner_msgs::planner_srv::Response& res);
  bool mhHomingServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool getPredictionCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void odomCb(const nav_msgs::Odometry &odom);

  void convertLongs(const geometry_msgs::PoseArray &longs_array, std::vector<Longitudinal> &longs_vec);
  std::vector<Longitudinal> updateLongsUsingPreviousCompartment(std::vector<Longitudinal> current_longs, std::vector<Longitudinal> prev_longs);
  int getMHID();
  std::vector<Longitudinal> getLongsForSubgraph(std::shared_ptr<SSGManager> subgraph);

  StateVec getCurrentRobotState() { return current_robot_state_; }
};