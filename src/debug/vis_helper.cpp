#include "vis_helper.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

geometry_msgs::msg::Vector3 fs::VisHelper::getScale(const fs::RvizScale scale, const float markerScale)
{
  geometry_msgs::msg::Vector3 result;

  double val(0.0);
  switch(scale)
  {
  case RvizScale::XXSMALL:
    val = 0.01;
    break;
  case RvizScale::XSMALL:
    val = 0.025;
    break;
  case RvizScale::SMALL:
    val = 0.05;
    break;
  case RvizScale::MEDIUM:
    val = 0.075;
    break;
  case RvizScale::LARGE:
    val = 0.1;
    break;
  case RvizScale::XLARGE:
    val = 0.5;
    break;
  case RvizScale::XXLARGE:
    val = 1.f;
    break;
  default:
    assert(false && "Not implemented yet");
  }

  result.x = val * markerScale;
  result.y = val * markerScale;
  result.z = val * markerScale;

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

geometry_msgs::msg::Point fs::VisHelper::convertPoint(const fs::FSVec3f& point)
{
  geometry_msgs::msg::Point point_msg;
  point_msg.x = point.x();
  point_msg.y = point.y();
  point_msg.z = point.z();
  return point_msg;
}

visualization_msgs::msg::Marker fs::VisHelper::creatRadarMarker(const fs::FSVec2f& position, const std::string& ns, const int id, const std_msgs::msg::ColorRGBA& color, const rclcpp::Duration& lifetime)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "ego";
  marker.ns              = ns;
  marker.id              = id;
  marker.type            = visualization_msgs::msg::Marker::POINTS;
  marker.action          = visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;
  marker.scale              = VisHelper::getScale(RvizScale::XXLARGE);
  marker.color              = color;
  // lifetime
  marker.lifetime = lifetime;
  geometry_msgs::msg::Point p;
  p.set__x(position.x());
  p.set__y(position.y());
  p.set__z(0);
  marker.points.push_back(p);
  return marker;
}

visualization_msgs::msg::Marker fs::VisHelper::creatLidarShapePointMarker(const fs::FSVec3f& position, const std::string& ns, const int id, const std_msgs::msg::ColorRGBA& color, const rclcpp::Duration& lifetime)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "ego";
  marker.ns              = ns;
  marker.id              = id;
  marker.type            = visualization_msgs::msg::Marker::SPHERE;
  marker.action          = visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;
  marker.scale              = VisHelper::getScale(RvizScale::XXLARGE, 1.5f);
  marker.color              = color;
  // lifetime
  marker.lifetime = lifetime;

  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z();
  return marker;
}

visualization_msgs::msg::Marker fs::VisHelper::createPolygon(const std::vector<FSVec3f>& points, const std::string& ns, const int id, const std_msgs::msg::ColorRGBA& color, geometry_msgs::msg::Vector3 scale, bool close, const rclcpp::Duration& lifetime)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "ego";
  marker.ns              = ns;
  marker.id              = id * points[0].x();
  marker.type            = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action          = visualization_msgs::msg::Marker::ADD;
  marker.lifetime        = lifetime;
  marker.pose            = VisHelper::getIdentityPose();

  marker.scale   = scale;
  marker.scale.y = 0.f;
  marker.scale.z = 0.f;
  marker.color   = color;
  marker.points.clear();
  marker.colors.clear();
  if(close)
  {
    for(int i = 0; i < points.size(); ++i)
    {
      marker.points.push_back(VisHelper::convertPoint(points[i % points.size()]));
      marker.points.push_back(VisHelper::convertPoint(points[(i + 1) % points.size()]));
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }
  }
  else
  {
    for(int i = 0; i < points.size() - 1; ++i)
    {
      marker.points.push_back(VisHelper::convertPoint(points[i]));
      marker.points.push_back(VisHelper::convertPoint(points[i + 1]));
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }
  }

  return marker;
}
