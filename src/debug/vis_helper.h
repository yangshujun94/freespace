#ifndef VIS_HELPER_H_
#define VIS_HELPER_H_

#include <opencv2/opencv.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/duration.hpp>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>
#include "peripheral/types.h"

namespace fs
{
  static const cv::Scalar COLOR_GRAY(130, 130, 130);
  static const cv::Scalar COLOR_B(255, 0, 0);
  static const cv::Scalar COLOR_G(0, 255, 0);
  static const cv::Scalar COLOR_R(0, 0, 255);
  static const cv::Scalar COLOR_Y(0, 255, 255);
  static const cv::Scalar COLOR_W(255, 255, 255);
  static const cv::Scalar COLOR_P(160, 32, 240);
  static const cv::Scalar COLOR_ORANGE(0, 165, 255);
  static const cv::Scalar COLOR_GOLD(0, 215, 255);
  static const cv::Scalar COLOR_PALE_GREEN(152, 251, 152);
  static const cv::Scalar COLOR_DEEP_SKY(205, 191, 0);
  static const cv::Scalar COLOR_OLIVE_DRAB(107, 142, 35);
  static const cv::Scalar COLOR_BROWN(0, 75, 152);
  static const cv::Scalar COLOR_GREY_BLUE(230, 224, 176);
  static const cv::Scalar COLOR_MAGENTA(128, 0, 128);

  // TODO: map rviz color to cv color
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
    SMALL,
    MEDIUM,
    LARGE,
  };

  class VisHelper
  {
  public:
    static std_msgs::msg::ColorRGBA        getColor(const RvizColor color);
    static geometry_msgs::msg::Vector3     getScale(const RvizScale scale);
    static geometry_msgs::msg::Pose        getIdentityPose();
    static geometry_msgs::msg::Pose        convertPose(const Eigen::Isometry3d& pose);
    static geometry_msgs::msg::Point       convertPoint(const EVector2& point);
    static visualization_msgs::msg::Marker createWireframeCuboid(const std::array<EVector2, 4>& points, const std::string& ns, const int id, const std_msgs::msg::ColorRGBA& color, const rclcpp::Duration& lifetime = rclcpp::Duration(0, 0));
  };

} // namespace fs

#endif //VIS_HELPER_H_
