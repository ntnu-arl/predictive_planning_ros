#pragma once

#include <ros/ros.h>

#include "common/config_utils.hpp"

enum Verbosity { SILENT = 0, PLANNER_STATUS = 1, ERROR = 2, WARN = 3, INFO = 4, DEBUG = 5 };

#define param_verbosity Verbosity::ERROR

#define ROSPARAM_ERROR(param_name)                                         \
  ({                                                                       \
    if (param_verbosity >= Verbosity::ERROR) {                     \
      std::cout << "\033[31m"                                              \
                << "[ERROR][File: " << __FILE__ << "] [Line: " << __LINE__ \
                << "]"                                                     \
                << "\nParam is not set: " << param_name << "\033[0m\n"     \
                << std::endl;                                              \
    }                                                                      \
  })

#define ROSPARAM_WARN(param_name, default_val)                            \
  ({                                                                      \
    if (param_verbosity >= Verbosity::WARN) {                  \
      std::cout << "\033[33m"                                             \
                << "[WARN][File: " << __FILE__ << "] [Line: " << __LINE__ \
                << "]"                                                    \
                << "\nParam is not set: " << param_name                   \
                << ". Default value is: " << default_val << "\033[0m\n"   \
                << std::endl;                                             \
    }                                                                     \
  })

#define ROSPARAM_INFO(msg)                                                \
  ({                                                                      \
    if (param_verbosity >= Verbosity::INFO) {                     \
      std::cout << "\033[32m"                                             \
                << "[INFO][File: " << __FILE__ << "] [Line: " << __LINE__ \
                << "]"                                                    \
                << "\n"                                                   \
                << msg << "\033[0m\n"                                     \
                << std::endl;                                             \
    }                                                                     \
  })

bool SubdueParams::loadParams(std::string ns)
{
  std::string param_name;
  
  param_name = ns + "/beamWidth";
  if (!ros::param::get(param_name, beamWidth) || beamWidth <= 0)
  {
    beamWidth = 4;
    ROSPARAM_WARN(param_name, beamWidth);
  }

  param_name = ns + "/compress";
  if (!ros::param::get(param_name, compress))
  {
    compress = false;
    ROSPARAM_WARN(param_name, compress);
  }

  param_name = ns + "/evalMethod";
  if (!ros::param::get(param_name, evalMethod) || (evalMethod < 1) || (evalMethod > 3))
  {
    evalMethod = 1;
    ROSPARAM_WARN(param_name, evalMethod);
  }

  param_name = ns + "/incremental";
  if (!ros::param::get(param_name, incremental))
  {
    incremental = false;
    ROSPARAM_WARN(param_name, incremental);
  }

  param_name = ns + "/iterations";
  if (!ros::param::get(param_name, iterations))
  {
    iterations = 1;
    ROSPARAM_WARN(param_name, iterations);
  }

  param_name = ns + "/limit";
  if (!ros::param::get(param_name, limit) || limit <= 0)
  {
    limit = 1;
    ROSPARAM_WARN(param_name, limit);
  }

  param_name = ns + "/maxVertices";
  if (!ros::param::get(param_name, maxVertices) || maxVertices <= 0)
  {
    maxVertices = 0;  /// i.e. infinity
    ROSPARAM_WARN(param_name, maxVertices);
  }

  param_name = ns + "/minVertices";
  if (!ros::param::get(param_name, minVertices) || minVertices <= 0)
  {
    minVertices = 1;
    ROSPARAM_WARN(param_name, minVertices);
  }

  param_name = ns + "/numBestSubs";
  if (!ros::param::get(param_name, numBestSubs) || numBestSubs <= 0)
  {
    numBestSubs = 3;
    ROSPARAM_WARN(param_name, numBestSubs);
  }

  param_name = ns + "/outputToFile";
  if (!ros::param::get(param_name, outputToFile))
  {
    outputToFile = false;
    ROSPARAM_WARN(param_name, outputToFile);
  }

  param_name = ns + "/outputLevel";
  if (!ros::param::get(param_name, outputLevel) || (outputLevel < 1) || (outputLevel > 5))
  {
    outputLevel = 2;
    ROSPARAM_WARN(param_name, outputLevel);
  }

  param_name = ns + "/allowInstanceOverlap";
  if (!ros::param::get(param_name, allowInstanceOverlap))
  {
    allowInstanceOverlap = false;
    ROSPARAM_WARN(param_name, allowInstanceOverlap);
  }

  param_name = ns + "/prune";
  if (!ros::param::get(param_name, prune))
  {
    prune = false;
    ROSPARAM_WARN(param_name, prune);
  }

  param_name = ns + "/predefinedSubs";
  if (!ros::param::get(param_name, predefinedSubs))
  {
    predefinedSubs = false;
    ROSPARAM_WARN(param_name, predefinedSubs);
  }

  param_name = ns + "/recursion";
  if (!ros::param::get(param_name, recursion))
  {
    recursion = false;
    ROSPARAM_WARN(param_name, recursion);
  }

  param_name = ns + "/threshold";
  if (!ros::param::get(param_name, threshold) || (threshold < 0.0) || (threshold > 1.0))
  {
    threshold = 0.0;
    ROSPARAM_WARN(param_name, threshold);
  }

  param_name = ns + "/directed";
  if (!ros::param::get(param_name, directed))
  {
    directed = false;  // Different from the default subdue params
    ROSPARAM_WARN(param_name, directed);
  }

  param_name = ns + "/valueBased";
  if (!ros::param::get(param_name, valueBased))
  {
    valueBased = false;  // Different from the default subdue params
    ROSPARAM_WARN(param_name, directed);
  }

  param_name = ns + "/variables";
  if (!ros::param::get(param_name, variables))
  {
    variables = false;  // Different from the default subdue params
    ROSPARAM_WARN(param_name, variables);
  }

  param_name = ns + "/relations";
  if (!ros::param::get(param_name, relations))
  {
    relations = false;
    ROSPARAM_WARN(param_name, relations);
  }
  else
  {
    if(relations) variables = true;
  }

  param_name = ns + "/use_pose_cost";
  if (!ros::param::get(param_name, use_pose_cost))
  {
    use_pose_cost = true;  // Different from the default subdue params
    ROSPARAM_WARN(param_name, use_pose_cost);
  }

  param_name = ns + "/normalize_value_with_cost";
  if (!ros::param::get(param_name, normalize_value_with_cost))
  {
    normalize_value_with_cost = true; 
    ROSPARAM_WARN(param_name, normalize_value_with_cost);
  }

  param_name = ns + "/outputFileName";
  if (!ros::param::get(param_name, outputFileName))
  {
    if(outputToFile)
    {
      ROSPARAM_ERROR(param_name);
      return false;
    }
  }

  param_name = ns + "/psInputFileName";
  if (!ros::param::get(param_name, psInputFileName))
  {
    if(predefinedSubs)
    {
      ROSPARAM_ERROR(param_name);
      return false;
    }
  }

  return true;
}

bool SSGParams::loadParams(std::string ns)
{
  std::string param_name;
  param_name = ns + "/path_to_save";
  if (!ros::param::get(param_name, path_to_save))
  {
    path_to_save = "ssg.msg";
  }

  param_name = ns + "/path_to_load";
  if (!ros::param::get(param_name, path_to_load))
  {
    path_to_load = path_to_save;
  }

  param_name = ns + "/wall_distance_thr";
  if (!ros::param::get(param_name, wall_distance_thr))
  {
    wall_distance_thr = 2.0;
  }

  param_name = ns + "/min_compartment_detections";
  if (!ros::param::get(param_name, min_compartment_detections))
  {
    min_compartment_detections = 6.0;
  }

  param_name = ns + "/min_long_length";
  if (!ros::param::get(param_name, min_long_length))
  {
    min_long_length = 1.0;
  }

  param_name = ns + "/lidar_tf_lookup_delay";
  if (!ros::param::get(param_name, lidar_tf_lookup_delay))
  {
    lidar_tf_lookup_delay = 0.3;
  }

  param_name = ns + "/min_wall_points";
  if (!ros::param::get(param_name, min_wall_points))
  {
    min_wall_points = 300;
  }

  param_name = ns + "/compartment_center_thr";
  if (!ros::param::get(param_name, compartment_center_thr))
  {
    compartment_center_thr = 1.0;
  }

  param_name = ns + "/compartment_distance_thr";
  if (!ros::param::get(param_name, compartment_distance_thr))
  {
    compartment_distance_thr = 5.0;
  }
  
  param_name = ns + "/sor_mean_k";
  if (!ros::param::get(param_name, sor_mean_k))
  {
    sor_mean_k = 50.0;
  }
  
  param_name = ns + "/sor_std_dev_thr";
  if (!ros::param::get(param_name, sor_std_dev_thr))
  {
    sor_std_dev_thr = 1.0;
  }

  param_name = ns + "/min_points_long";
  if (!ros::param::get(param_name, min_points_long))
  {
    min_points_long = 50;
  }

  param_name = ns + "/min_long_detections";
  if (!ros::param::get(param_name, min_long_detections))
  {
    min_long_detections = 6;
  }

  param_name = ns + "/num_lines_to_extract";
  if (!ros::param::get(param_name, num_lines_to_extract))
  {
    num_lines_to_extract = 6;
  }

  param_name = ns + "/min_long_height";
  if (!ros::param::get(param_name, min_long_height))
  {
    min_long_height = 1.0;
  }

  param_name = ns + "/max_long_height";
  if (!ros::param::get(param_name, max_long_height))
  {
    max_long_height = 4.8;
  }

  param_name = ns + "/detection_fov";
  if (!ros::param::get(param_name, detection_fov))
  {
    detection_fov = 360;
  }

  param_name = ns + "/long_distance_thr";
  if (!ros::param::get(param_name, long_distance_thr))
  {
    long_distance_thr = 0.2;
  }

  param_name = ns + "/min_dist_bet_longs";
  if (!ros::param::get(param_name, min_dist_bet_longs))
  {
    min_dist_bet_longs = 0.7;
  }

  param_name = ns + "/furthest_wall_to_consider";
  if (!ros::param::get(param_name, furthest_wall_to_consider))
  {
    furthest_wall_to_consider = 5.5;
  }

  param_name = ns + "/min_long_detections_for_overlap";
  if (!ros::param::get(param_name, min_long_detections_for_overlap))
  {
    min_long_detections_for_overlap = 3;
  }

  param_name = ns + "/wall_to_long_thr";
  if (!ros::param::get(param_name, wall_to_long_thr))
  {
    wall_to_long_thr = 0.4;
  }

  param_name = ns + "/min_num_longs";
  if (!ros::param::get(param_name, min_num_longs))
  {
    min_num_longs = 2;
  }

  param_name = ns + "/long_dir_ang_thr";
  if (!ros::param::get(param_name, long_dir_ang_thr))
  {
    long_dir_ang_thr = 5.0*M_PI/180.0;
  }
  
  param_name = ns + "/wall_to_long_thr_min";
  if (!ros::param::get(param_name, wall_to_long_thr_min))
  {
    wall_to_long_thr_min = 0.07;
  }

  param_name = ns + "/long_only_on_specific_walls";
  if (!ros::param::get(param_name, long_only_on_specific_walls))
  {
    long_only_on_specific_walls = false;
  }

  param_name = ns + "/long_width_thr";
  if (!ros::param::get(param_name, long_width_thr))
  {
    long_width_thr = 0.2;
  }
  
  param_name = ns + "/manual_inlier_classification";
  if (!ros::param::get(param_name, manual_inlier_classification))
  {
    manual_inlier_classification = false;
  }

  param_name = ns + "/num_longs_to_skip";
  if (!ros::param::get(param_name, num_longs_to_skip))
  {
    num_longs_to_skip = 0;
  }

  param_name = ns + "/num_pats_to_modify";
  if (!ros::param::get(param_name, num_pats_to_modify))
  {
    num_pats_to_modify = 0;
  }

  param_name = ns + "/external_mh_detections";
  if (!ros::param::get(param_name, external_mh_detections))
  {
    external_mh_detections = 0;
  }

  param_name = ns + "/wall_normal_thr";
  if (!ros::param::get(param_name, wall_normal_thr))
  {
    wall_normal_thr = 15*M_PI/180.0;
  }

  param_name = ns + "/wall_normal_ang_thr";
  if (!ros::param::get(param_name, wall_normal_ang_thr))
  {
    wall_normal_ang_thr = 15*M_PI/180.0;
  }
  
  return true;
}

bool SystemParams::loadParams(std::string ns)
{
  std::string param_name;
  param_name = ns + "/build_bwt_ssg";
  if (!ros::param::get(param_name, build_bwt_ssg))
  {
    build_bwt_ssg = true;
  }

  param_name = ns + "/num_compartments_to_inspect";
  if (!ros::param::get(param_name, num_compartments_to_inspect))
  {
    num_compartments_to_inspect = 5;
  }

  param_name = ns + "/max_mh_height";
  if (!ros::param::get(param_name, max_mh_height))
  {
    max_mh_height = 5.0;
  }

  param_name = ns + "/vp_reach_thr";
  if (!ros::param::get(param_name, vp_reach_thr))
  {
    vp_reach_thr = 1.0;
  }

  param_name = ns + "/use_opportunistic_inspection";
  if (!ros::param::get(param_name, use_opportunistic_inspection))
  {
    use_opportunistic_inspection = true;
  }

  std::vector<double> param_val;
  param_name = ns + "/inspection_robot_box_size";
  if ((!ros::param::get(param_name, param_val)) || (param_val.size() != 3)) {
    ROSPARAM_ERROR(param_name);
    return false;
  }
  inspection_robot_box_size << param_val[0], param_val[1], param_val[2];

  return true;
}

bool PredictionParams::loadParams(std::string ns)
{
  std::string param_name;
  param_name = ns + "/visualize_enhance_hierarchy";
  if (!ros::param::get(param_name, visualize_enhance_hierarchy))
  {
    visualize_enhance_hierarchy = true;
  }

  param_name = ns + "/use_predictive_planning";
  if (!ros::param::get(param_name, use_predictive_planning))
  {
    use_predictive_planning = true;
  }

  param_name = ns + "/init_sem_weight";
  if (!ros::param::get(param_name, init_sem_weight))
  {
    init_sem_weight = 1.5;
    kSem = 0.5;  // TEMP
  }

  param_name = ns + "/kSemPriorWeight";
  if (!ros::param::get(param_name, kSemPriorWeight))
  {
    kSemPriorWeight = init_sem_weight;
  }

  param_name = ns + "/min_exp_weight";
  if (!ros::param::get(param_name, min_exp_weight))
  {
    min_exp_weight = 0.1;
  }

  param_name = ns + "/max_local_exploration_steps";
  if (!ros::param::get(param_name, max_local_exploration_steps))
  {
    max_local_exploration_steps = 4;
  }

  param_name = ns + "/max_assisted_exploration_steps";
  if (!ros::param::get(param_name, max_assisted_exploration_steps))
  {
    max_assisted_exploration_steps = 4;
  }

  param_name = ns + "/max_local_exploration_fails";
  if (!ros::param::get(param_name, max_local_exploration_fails))
  {
    max_local_exploration_fails = 3;
  }

  param_name = ns + "/max_assisted_exploration_fails";
  if (!ros::param::get(param_name, max_assisted_exploration_fails))
  {
    max_assisted_exploration_fails = 3;
  }

  param_name = ns + "/viewpoint_ang_offset";
  if (!ros::param::get(param_name, viewpoint_ang_offset))
  {
    viewpoint_ang_offset = 15*M_PI/180;
  }

  param_name = ns + "/view_center";
  if (!ros::param::get(param_name, view_center))
  {
    view_center = false;
  }

  param_name = ns + "/overlap_thr";
  if (!ros::param::get(param_name, overlap_thr))
  {
    overlap_thr = 0.8;
  }

  return true;
}
