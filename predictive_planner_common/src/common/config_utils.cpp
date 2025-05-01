#include "common/config_utils.hpp"

Config::Config() {}

bool Config::loadAllParams(std::string ns) 
{
  ssg_params.loadParams(ns + "/SSGParams");
  subdue_params.loadParams(ns + "/SubsdueParams");
  system_params.loadParams(ns + "/SystemParams");
  prediction_params.loadParams(ns + "/PredictionParams");
  return true;
}
