#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// anode: 50, stiff: 100, bt: 150, long: 200, wall: 250, manhole: 300 
/// Class labels (string <-> int)
#define L_ANODE 50
#define L_STIFF 100
#define L_BT 150
#define L_LONG 200
#define L_WALL 250
#define L_MANHOLE 300
#define L_MANHOLE_A 301
#define L_COMPARTMENT 350

namespace Eigen 
{
  typedef Matrix<double, 6, 1> Vector6d;
}


struct OBB
{
  Eigen::Matrix3f rotational_matrix;
  Eigen::Vector3f position;
  Eigen::Vector3f min_point;
  Eigen::Vector3f max_point;
};

struct Plane
{
  Plane()
  {
    points.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  int id;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  pcl::ModelCoefficients coefficients;
  Eigen::Vector3d centroid;
  Eigen::Vector3d normal;  // TODO: switch to this 
  OBB obb;
};

struct Line
{
  Line()
  {
    points.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  int id;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  pcl::ModelCoefficients coefficients;
  Eigen::Vector3d direction;
  Eigen::Vector3d center;
};

struct WallInstance
{
  int id;  // In real ssg this is the vertex id
  Plane plane;
  int num_detections = 0;
  bool seen;
  int associated_compartment_vertex_id;
  bool just_verified = true;
  std::set<int> associated_long_vertex_ids;
};

struct LongInstance
{
  int id;
  Line line;
  int num_detections = 0;
  bool seen;
  bool just_verified = true;
  int associated_wall_vertex_id;
};

struct CompartmentInstance
{
  int id;  // In real ssg this is the vertex id
  std::set<int> associated_wall_vertex_ids;
  Eigen::Vector6d state;
  int num_detections = 0;
  bool just_verified = true;
  bool seen = false;

  // CompartmentInstance(CompartmentInstance &c)
  // {
  //   id = c.id;
  //   associated_wall_vertex_ids = c.associated_wall_vertex_ids;
  //   state = c.state;
  //   num_detections = c.num_detections;
  //   just_verified = c.just_verified;
  // }
};

struct ManholeInstance
{
  int detected_id;  // ID of the manhole reported by the detector
  int v_for;
  int v_back;
  Eigen::Vector6d state;
  int vertex_id;
};

Eigen::Vector3i getColor(int val);
void smartInsert(visualization_msgs::MarkerArray &a1, visualization_msgs::MarkerArray &a2);

void transformState(const Eigen::Vector6d &s_og, Eigen::Vector6d &s_tfed, const geometry_msgs::TransformStamped &tfs);

void convert(const geometry_msgs::Pose & p, geometry_msgs::Transform & tf);
void convert(const Eigen::Vector6d &s, geometry_msgs::Pose &p);
void convert(const Eigen::Vector4d &s, geometry_msgs::Pose &p);
void convert(const geometry_msgs::Pose &p, Eigen::Vector6d &s);