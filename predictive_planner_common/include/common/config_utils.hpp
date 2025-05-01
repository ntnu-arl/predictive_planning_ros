#pragma once
#include <string>
#include <math.h>
#include <eigen3/Eigen/Dense>

struct SSGParams
{
	bool external_mh_detections = false;

  int min_wall_detections = 6;
  double wall_distance_thr = 0.2;
	double along_wall_dist_thr = 2.0;
  double wall_normal_thr = 15*M_PI/180.0;
  double wall_normal_ang_thr = 15*M_PI/180.0;
  double wall_to_long_thr = 0.4;
	double wall_to_long_thr_min = 0.07;
	int min_wall_points = 300;
	double furthest_wall_to_consider = 5.5;
	
	int min_compartment_detections = 6;
  double compartment_center_thr = 2.0;
	double compartment_distance_thr = 5.0;
	
	int min_long_detections = 6;
	int min_long_detections_for_overlap = 3;
	double long_distance_thr = 0.2;
	double min_dist_bet_longs = 0.7;
	double long_direction_thr = 10*M_PI/180.0;
	double long_dir_ang_thr = 5.0*M_PI/180.0;
	double long_width_thr = 0.2;
	double min_long_length = 1.0;
	int min_points_long = 50;
	int num_lines_to_extract = 6;
	double min_long_height = 1.0;
	double max_long_height = 4.8;
	int min_num_longs = 2;
	bool long_only_on_specific_walls = false;
	bool manual_inlier_classification = false;

	std::string path_to_save;
	std::string path_to_load;

	double lidar_tf_lookup_delay = 0.3;
	double detection_fov = 360;  // Degrees

	double sor_mean_k = 50;
	double sor_std_dev_thr = 1.0;

	int substruct_label_start = 9000;

	int num_longs_to_skip = 0;
	int num_pats_to_modify = 0;

	bool loadParams(std::string ns);
};

struct PredictionParams
{
	bool visualize_enhance_hierarchy = true;
	bool use_predictive_planning = true;

	// Weights for assisted exploration
  double kSem;
	double kSemPriorWeight;
  double init_sem_weight;
  double min_exp_weight;

	int max_local_exploration_steps;
	int max_assisted_exploration_steps;
	int max_local_exploration_fails;
	int max_assisted_exploration_fails;

	double viewpoint_ang_offset;
	bool view_center;
	double overlap_thr;
	
	bool loadParams(std::string ns);
};

struct SystemParams
{
	bool build_bwt_ssg;
	int num_compartments_to_inspect;
	double max_mh_height;
	double vp_reach_thr;
	bool use_opportunistic_inspection;
	Eigen::Vector3d inspection_robot_box_size;

	bool loadParams(std::string ns);
};

struct SubdueParams
{

	bool predefinedSubs; // TRUE is predefined substructures given ///
	bool outputToFile; // TRUE if file given for machine-readable output ///
	bool directed;     // If TRUE, 'e' edges treated as directed ///
	bool valueBased;   // If TRUE, then queues are trimmed to contain ///
												//   all substructures with the top beamWidth
												//   values; otherwise, queues are trimmed to
												//   contain only the top beamWidth substructures.
	bool prune;        // If TRUE, then expanded substructures with lower ///
												//   value than their parents are discarded.
	bool allowInstanceOverlap; // Default is FALSE; if TRUE, then instances ///
																// may overlap, but compression costlier
	bool recursion;    // If TRUE, recursive graph grammar subs allowed ///
	bool variables;    // If TRUE, variable vertices allowed ///
	bool relations;    // If TRUE, relations between vertices allowed ///
	bool incremental;  // If TRUE, data is processed incrementally ///
	bool compress;     // If TRUE, write compressed graph to file ///
	int iterations;     // Number of SUBDUE iterations; if more than 1, then ///
												// graph compressed with best sub between iterations
	int beamWidth;      // Limit on size of substructure queue (> 0) ///
	int limit;          // Limit on number of substructures expanded (> 0) ///
	int maxVertices;    // Maximum vertices in discovered substructures ///
	int minVertices;    // Minimum vertices in discovered substructures ///
	int numBestSubs;    // Limit on number of best substructures ///
												//   returned (> 0)
	int outputLevel;    // More screen (stdout) output as value increases ///
	int evalMethod;     // One of EVAL_MDL (default), EVAL_SIZE or ///
												//   EVAL_SETCOVER
	double threshold;     // Percentage of size by which an instance can differ ///
												// from the substructure definition according to
												// graph match transformation costs
	bool use_pose_cost;
	bool normalize_value_with_cost;

	std::string outputFileName;
	std::string psInputFileName;
	bool loadParams(std::string ns);
};

class Config
{
private:
	// std::shared_ptr<spdlog::logger> logger_;
	// std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> logger_console_sink_;

public:
	std::string ns;

	SSGParams ssg_params;
	SubdueParams subdue_params;
	SystemParams system_params;
	PredictionParams prediction_params;

	Config();

	bool loadAllParams(std::string ns);
};
