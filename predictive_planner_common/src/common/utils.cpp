#include "common/utils.hpp"

Eigen::Vector3i getColor(int val)
{
  const double max_val = 360;
  // std::cout << std::floor(val/max_val)*max_val;
  double H = (double)val - std::floor(val / max_val) * max_val;
  // std::cout << " H: " << H;
  double V = 1;
  double S = 1;

  double hh, p, q, t, ff;
  long i;
  Eigen::Vector3d out;

  hh = H;
  if (hh >= 360.0)
    hh = 0.0;
  hh /= 60.0;
  i = (long)hh;
  ff = hh - i;
  p = V * (1.0 - S);
  q = V * (1.0 - (S * ff));
  t = V * (1.0 - (S * (1.0 - ff)));

  switch (i)
  {
  case 0:
    out.x() = V;
    out.y() = t;
    out.z() = p;
    break;
  case 1:
    out.x() = q;
    out.y() = V;
    out.z() = p;
    break;
  case 2:
    out.x() = p;
    out.y() = V;
    out.z() = t;
    break;

  case 3:
    out.x() = p;
    out.y() = q;
    out.z() = V;
    break;
  case 4:
    out.x() = t;
    out.y() = p;
    out.z() = V;
    break;
  case 5:
  default:
    out.x() = V;
    out.y() = p;
    out.z() = q;
    break;
  }

  Eigen::Vector3i out_rgb;
  out *= 255;
  out_rgb = out.cast<int>();
  return out_rgb;
}

void smartInsert(visualization_msgs::MarkerArray &a1, visualization_msgs::MarkerArray &a2)
{
  int id_offset = a1.markers.size();
  for(auto &m : a2.markers)
  {
    m.id += id_offset;
    a1.markers.push_back(m);
  }
}

void transformState(const Eigen::Vector6d &s_og, Eigen::Vector6d &s_tfed, const geometry_msgs::TransformStamped &tfs)
{
  geometry_msgs::Pose p_og, p_tfed;
  convert(s_og, p_og);
  tf2::doTransform(p_og, p_tfed, tfs);
  convert(p_tfed, s_tfed);
}

void convert(const geometry_msgs::Pose & p, geometry_msgs::Transform & tf)
{
  tf.translation.x = p.position.x;
  tf.translation.y = p.position.y;
  tf.translation.z = p.position.z;
  tf.rotation.w = p.orientation.w;
  tf.rotation.x = p.orientation.x;
  tf.rotation.y = p.orientation.y;
  tf.rotation.z = p.orientation.z;
}

void convert(const Eigen::Vector6d &s, geometry_msgs::Pose &p)
{
  tf::Quaternion quat;
  quat.setEuler(s(3), s(4), s(5));
  tf::Vector3 origin(s(0), s(1), s(2));
  tf::Pose poseTF(quat, origin);
  tf::poseTFToMsg(poseTF, p);
}

void convert(const geometry_msgs::Pose &p, Eigen::Vector6d &s)
{
  Eigen::Quaterniond quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(0,1,2);
  s.x() = p.position.x;
  s.y() = p.position.y;
  s.z() = p.position.z;
  s.tail(3) = rpy;
}

void convert(const Eigen::Vector4d &s, geometry_msgs::Pose &p)
{
  tf::Quaternion quat;
  quat.setEuler(s(3), 0.0, 0.0);
  tf::Vector3 origin(s(0), s(1), s(2));
  tf::Pose poseTF(quat, origin);
  tf::poseTFToMsg(poseTF, p);
}