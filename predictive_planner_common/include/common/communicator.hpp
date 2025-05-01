#pragma once

#include "common/config_utils.hpp"

#include <thread>
#include <mutex>

class SSGManager;
class BWTSSGFrontEnd;
class PredictivePlanning;
class Gbplanner;
class PlannerManager;

class Communicator
{
private:
  std::shared_ptr<SSGManager> ssg_manager_;
	std::shared_ptr<BWTSSGFrontEnd> bwt_ssg_frontend_;
	std::shared_ptr<PredictivePlanning> predictive_planning_;
	std::shared_ptr<Gbplanner> gbplanner_;
	std::shared_ptr<PlannerManager> planner_manager_;
	
	std::shared_ptr<Config> config_;

public:

  Communicator(std::shared_ptr<Config> config);

	void setupSSGManager(std::shared_ptr<SSGManager> ssg_manager);
	void setupBWTSSGFrontEnd(std::shared_ptr<BWTSSGFrontEnd> bwt_ssg_frontend);
	void setupPredictivePlanning(std::shared_ptr<PredictivePlanning> predictive_planning);
	void setupGbplanner(std::shared_ptr<Gbplanner> gbplanner);
	void setupPlannerManager(std::shared_ptr<PlannerManager> planner_manager);

	inline std::shared_ptr<SSGManager> ssg_manager()
	{
		return ssg_manager_;
	}

	inline std::shared_ptr<BWTSSGFrontEnd> bwt_ssg_frontend()
	{
		return bwt_ssg_frontend_;
	}
	
	inline std::shared_ptr<PredictivePlanning> predictive_planning()
	{
		return predictive_planning_;
	}

	inline std::shared_ptr<Gbplanner> gbplanner()
	{
		return gbplanner_;
	}

	inline std::shared_ptr<PlannerManager> planner_manager()
	{
		return planner_manager_;
	}
};

