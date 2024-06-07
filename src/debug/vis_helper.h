#ifndef VIS_HELPER_H_
#define VIS_HELPER_H_

#include <opencv2/opencv.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/duration.hpp>
#include <Eigen/Geometry>
#include "peripheral/types.h"

namespace fs
{

  enum class RvizColor : uint8_t
  {
    BLACK,
    BROWN,
    BLUE,
    CYAN,
    GREY,
    DARK_GREY,
    GREEN,
    LIME_GREEN,
    MAGENTA,
    ORANGE,
    PURPLE,
    RED,
    PINK,
    WHITE,
    YELLOW,
    TRANSLUCENT,
    TRANSLUCENT_LIGHT,
    TRANSLUCENT_DARK,
    RAND,
    CLEAR,
  };

  enum class RvizScale : uint8_t
  {
    XXSMALL,
    XSMALL,
    SMALL,
    MEDIUM,
    LARGE,
    XLARGE,
    XXLARGE,
  };
  struct BoxString
  {
    cv::Rect    rect;
    std::string str;
  };

  class VisHelper
  {
  public:
    static std_msgs::msg::ColorRGBA        getColor(const RvizColor color);
    static geometry_msgs::msg::Vector3     getScale(const RvizScale scale, const float markerScale = 1.f);
    static geometry_msgs::msg::Pose        getIdentityPose();
    static geometry_msgs::msg::Pose        convertPose(const Eigen::Isometry3d& pose);
    static geometry_msgs::msg::Point       convertPoint(const FSVec3f& point);
    static visualization_msgs::msg::Marker createPolygon(const std::vector<FSVec3f>& points, const std::string& ns, const int id, const std_msgs::msg::ColorRGBA& color, geometry_msgs::msg::Vector3 scale, bool close = true, const rclcpp::Duration& lifetime = rclcpp::Duration(0, 0));
    static visualization_msgs::msg::Marker creatRadarMarker(const fs::FSVec2f& position, const std::string& ns, const int id, const std_msgs::msg::ColorRGBA& color, const rclcpp::Duration& lifetime = rclcpp::Duration(0, 0));
    static visualization_msgs::msg::Marker creatLidarShapePointMarker(const fs::FSVec3f& position, const std::string& ns, const int id, const std_msgs::msg::ColorRGBA& color, const rclcpp::Duration& lifetime = rclcpp::Duration(0, 0));
  };

} // namespace fs

#endif //VIS_HELPER_H_
