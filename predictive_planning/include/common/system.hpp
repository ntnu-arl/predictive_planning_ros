#pragma once

#include "ros/ros.h"

#include "common/communicator.hpp"
// #include "bwt_ssg_builder/bwt_ssg_builder_sim.hpp"
#include "bwt_ssg_builder/bwt_ssg_manager.hpp"
#include "graph/ssg_manager.hpp"
#include "predictive_planning.hpp"
#include "gbplanner/gbplanner.h"
#include "planner_manager.hpp"

class System
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	std::shared_ptr<Communicator> comm_;
	std::shared_ptr<Config> config_;
public:
	System(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

	void buildSystem();

	void shutdownRoutine();
};