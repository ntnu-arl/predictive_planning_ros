#include "common/system.hpp"

System::System(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    :nh_(nh), nh_private_(nh_private)
{
  buildSystem();
}

void System::buildSystem()
{
	config_ = std::make_shared<Config>();
	config_->loadAllParams(ros::this_node::getName());

	comm_ = std::make_shared<Communicator>(config_);

	comm_->setupSSGManager(std::make_shared<SSGManager>());

	if(config_->system_params.build_bwt_ssg) comm_->setupBWTSSGFrontEnd(std::make_shared<BWTSSGFrontEnd>(nh_, nh_private_, comm_, config_));

	comm_->setupPredictivePlanning(std::make_shared<PredictivePlanning>(nh_, nh_private_, comm_, config_));
	
	comm_->setupGbplanner(std::make_shared<Gbplanner>(nh_, nh_private_, comm_));
	comm_->gbplanner()->setConfig(config_);

	comm_->setupPlannerManager(std::make_shared<PlannerManager>(nh_, nh_private_, comm_, config_));
}

void System::shutdownRoutine()
{

}
