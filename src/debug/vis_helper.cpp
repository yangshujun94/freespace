#include "vis_helper.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

std_msgs::msg::ColorRGBA fs::VisHelper::getColor(const RvizColor color)
{
  std_msgs::msg::ColorRGBA result;

  constexpr float alpha = 1.f;

  switch(color)
  {
  case RvizColor::RED:
    result.r = 0.8;
    result.g = 0.1;
    result.b = 0.1;
    result.a = alpha;
    break;
  case RvizColor::GREEN:
    result.r = 0.1;
    result.g = 0.8;
    result.b = 0.1;
    result.a = alpha;
    break;
  case RvizColor::GREY:
    result.r = 0.8;
    result.g = 0.8;
    result.b = 0.8;
    result.a = alpha;
    break;
  case RvizColor::DARK_GREY:
    result.r = 0.4;
    result.g = 0.4;
    result.b = 0.4;
    result.a = alpha;
    break;
  case RvizColor::WHITE:
    result.r = 1.0;
    result.g = 1.0;
    result.b = 1.0;
    result.a = alpha;
    break;
  case RvizColor::ORANGE:
    result.r = 1.0;
    result.g = 0.5;
    result.b = 0.0;
    result.a = alpha;
    break;
  case RvizColor::TRANSLUCENT_LIGHT:
    result.r = 0.1;
    result.g = 0.1;
    result.b = 0.1;
    result.a = 0.1;
    break;
  case RvizColor::TRANSLUCENT:
    result.r = 0.1;
    result.g = 0.1;
    result.b = 0.1;
    result.a = 0.25;
    break;
  case RvizColor::TRANSLUCENT_DARK:
    result.r = 0.1;
    result.g = 0.1;
    result.b = 0.1;
    result.a = 0.5;
    break;
  case RvizColor::BLACK:
    result.r = 0.0;
    result.g = 0.0;
    result.b = 0.0;
    result.a = alpha;
    break;
  case RvizColor::YELLOW:
    result.r = 1.0;
    result.g = 1.0;
    result.b = 0.0;
    result.a = alpha;
    break;
  case RvizColor::BROWN:
    result.r = 0.597;
    result.g = 0.296;
    result.b = 0.0;
    result.a = alpha;
    break;
  case RvizColor::PINK:
    result.r = 1.0;
    result.g = 0.4;
    result.b = 1;
    result.a = alpha;
    break;
  case RvizColor::LIME_GREEN:
    result.r = 0.6;
    result.g = 1.0;
    result.b = 0.2;
    result.a = alpha;
    break;
  case RvizColor::CLEAR:
    result.r = 1.0;
    result.g = 1.0;
    result.b = 1.0;
    result.a = 0.0;
    break;
  case RvizColor::PURPLE:
    result.r = 0.597;
    result.g = 0.0;
    result.b = 0.597;
    result.a = alpha;
    break;
  case RvizColor::CYAN:
    result.r = 0.0;
    result.g = 1.0;
    result.b = 1.0;
    result.a = alpha;
    break;
  case RvizColor::MAGENTA:
    result.r = 1.0;
    result.g = 0.0;
    result.b = 1.0;
    result.a = alpha;
    break;
  case RvizColor::BLUE:
    result.r = 65.f / 255.f;
    result.g = 105.f / 255.f;
    result.b = 225.f / 255.f;
    result.a = alpha;
    break;
  default:
    assert(false && "not supported");
  }

  return result;
}

geometry_msgs::msg::Vector3 fs::VisHelper::getScale(const RvizScale scale)
{
  geometry_msgs::msg::Vector3 result;

  double val(0.0);
  switch(scale)
  {
  case RvizScale::SMALL:
    val = 0.075;
    break;
  case RvizScale::MEDIUM:
    val = 0.5F;
    break;
  case RvizScale::LARGE:
    val = 2.f;
    break;
  default:
    assert(false && "Not implemented yet");
  }

  result.x = val;
  result.y = val;
  result.z = val;

  return result;
}

geometry_msgs::msg::Pose fs::VisHelper::getIdentityPose()
{
  geometry_msgs::msg::Pose pose;
  // Position
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;

  // Orientation on place
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  return pose;
}

geometry_msgs::msg::Pose fs::VisHelper::convertPose(const Eigen::Isometry3d& pose)
{
  geometry_msgs::msg::Pose pose_msg;
  tf2::convert(pose, pose_msg);
  return pose_msg;
}

geometry_msgs::msg::Point fs::VisHelper::convertPoint(const fs::EVector2& point)
{
  geometry_msgs::msg::Point point_msg;
  point_msg.x = point.x();
  point_msg.y = point.y();
  point_msg.z = 0.f;
  return point_msg;
}

visualization_msgs::msg::Marker fs::VisHelper::createWireframeCuboid(const std::array<EVector2, 4>& points, const std::string& ns, const int id, const std_msgs::msg::ColorRGBA& color, const rclcpp::Duration& lifetime)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "ego";
  marker.ns              = ns;
  marker.id              = id;
  marker.type            = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action          = visualization_msgs::msg::Marker::ADD;
  marker.lifetime        = lifetime;
  marker.pose            = VisHelper::getIdentityPose();

  marker.scale   = VisHelper::getScale(RvizScale::SMALL);
  marker.scale.y = 0.f;
  marker.scale.z = 0.f;
  marker.color   = color;
  marker.points.clear();
  marker.colors.clear();

  marker.points.push_back(VisHelper::convertPoint(points[0]));
  marker.points.push_back(VisHelper::convertPoint(points[1]));
  marker.colors.push_back(color);
  marker.colors.push_back(color);

  marker.points.push_back(VisHelper::convertPoint(points[0]));
  marker.points.push_back(VisHelper::convertPoint(points[3]));
  marker.colors.push_back(color);
  marker.colors.push_back(color);

  marker.points.push_back(VisHelper::convertPoint(points[1]));
  marker.points.push_back(VisHelper::convertPoint(points[2]));
  marker.colors.push_back(color);
  marker.colors.push_back(color);

  marker.points.push_back(VisHelper::convertPoint(points[2]));
  marker.points.push_back(VisHelper::convertPoint(points[3]));
  marker.colors.push_back(color);
  marker.colors.push_back(color);

  return marker;
}
