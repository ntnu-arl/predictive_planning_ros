#include "common/communicator.hpp"

Communicator::Communicator(std::shared_ptr<Config> config) 
	:config_(std::move(config))
{
}

void Communicator::setupSSGManager(std::shared_ptr<SSGManager> ssg_manager)
{
	ssg_manager_ = std::move(ssg_manager);
}

void Communicator::setupBWTSSGFrontEnd(std::shared_ptr<BWTSSGFrontEnd> bwt_ssg_frontend)
{
	bwt_ssg_frontend_ = std::move(bwt_ssg_frontend);
}

void Communicator::setupPredictivePlanning(std::shared_ptr<PredictivePlanning> predictive_planning)
{
	predictive_planning_ = std::move(predictive_planning);
}

void Communicator::setupGbplanner(std::shared_ptr<Gbplanner> gbplanner)
{
	gbplanner_ = std::move(gbplanner);
}

void Communicator::setupPlannerManager(std::shared_ptr<PlannerManager> planner_manager)
{
	planner_manager_ = std::move(planner_manager);
}