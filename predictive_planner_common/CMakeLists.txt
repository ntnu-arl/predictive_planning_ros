cmake_minimum_required(VERSION 3.0.2)
project(predictive_planner_common)

find_package(catkin_simple REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -O3 -Werror=return-type) # -O3

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/common/config_utils.cpp
  src/common/communicator.cpp
  src/graph/graph.cpp
  src/graph/ssg_manager.cpp
  src/common/utils.cpp
)

# generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

##########
# EXPORT #
##########
cs_install()
cs_export(LIBRARIES)
