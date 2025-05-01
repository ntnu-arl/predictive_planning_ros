#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "common/system.hpp"

int main(int argc, char** argv) 
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  ros::init(argc, argv, "predictive_planner_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  System system(nh, nh_private);

  ros::spin();

  return 0;
}
