cmake_minimum_required(VERSION 3.0.2)
project(predictive_planning)

find_package(catkin_simple REQUIRED)
find_package(subdue REQUIRED)
find_package(GSL REQUIRED)
find_package(bwt_ssg_builder REQUIRED)
find_package(predictive_planner_common REQUIRED)
find_package(gbplanner REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

add_definitions(-Wall -O3 -Werror=return-type) # -O3

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/common/system.cpp
  src/common/config_ros.cpp
  src/predictive_planning.cpp
  src/planner_manager.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

cs_add_executable(predictive_planning_node src/predictive_planning_node.cpp)
target_link_libraries(predictive_planning_node ${PROJECT_NAME} ${subdue_LIBRARIES})
# target_link_libraries(predictive_planning_node ${PROJECT_NAME} ${subdue_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export(LIBRARIES)
