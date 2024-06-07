#ifndef UTILS_H_
#define UTILS_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include "types.h"
#include "switcher.h"
#include "defs.h"
#include "peripheral/map_provider/map_provider.h"
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/TransverseMercatorExact.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <common/math/pose_transformer.h>
#if FS_CHECK(CFG_LOAD_RECORDING)
#include <rosbag2_cpp/reader.hpp>
#include <common/subscriber.h>
#endif
namespace fs
{
  class Utils
  {
  public:
#if FS_CHECK(CFG_LOAD_RECORDING)
    template<typename T>
    static std::unique_ptr<T> deserializeRosMessage(const rosbag2_storage::SerializedBagMessage& serializedBagMsg)
    {
      auto                     serializedMsg = rclcpp::SerializedMessage(*serializedBagMsg.serialized_data);
      auto                     msgPtr        = std::make_unique<T>();
      rclcpp::Serialization<T> serialization;
      serialization.deserialize_message(&serializedMsg, msgPtr.get());

      return msgPtr;
    }

    template<typename T>
    static std::unique_ptr<T> deserializeIdlMessage(const rosbag2_storage::SerializedBagMessage& serializedBagMsg)
    {
      auto                                             serializedMsg = rclcpp::SerializedMessage(*serializedBagMsg.serialized_data);
      uto::idl::SerializedProto                        idl;
      rclcpp::Serialization<uto::idl::SerializedProto> serialization;
      serialization.deserialize_message(&serializedMsg, &idl);

      auto msgPtr = std::make_unique<T>();
      msgPtr->ParseFromArray(idl.data.data(), idl.data.size());

      return msgPtr;
    }
#endif

    template<typename T>
    static inline T clamp(const T& x, const T& lo, const T& hi)
    {
      return (x < lo) ? lo : ((hi < x) ? hi : x);
    }

    template<typename T, typename T2>
    static inline bool isInGrid(const T& gird, T2 min_x, T2 max_x, T2 min_y, T2 max_y)
    {
      return ((gird.y() >= min_y) && (gird.y() < max_y) && (gird.x() >= min_x) && (gird.x() < max_x));
    }
    template<typename T, typename T2>
    static inline bool isInGrid(const T& gird, T2 area)
    {
      return ((gird.y() >= area.z()) && (gird.y() < area.w()) && (gird.x() >= area.x()) && (gird.x() < area.y()));
    }
    inline static constexpr float mod(const float value)
    {
      return std::fmod(std::fmod(value, GRID_MAP_M) + GRID_MAP_M, GRID_MAP_M);
    }

    inline static constexpr int modN(const int n)
    {
      return (n % GRID_MAP_SIZE + GRID_MAP_SIZE) % GRID_MAP_SIZE;
    }
    static constexpr double mod(const double value)
    {
      return std::fmod(std::fmod(value, GRID_MAP_M) + GRID_MAP_M, GRID_MAP_M);
    }

    static bool pointInPolygonQuick(float px, float py, const std::vector<cv::Point2f>& poly);

    static bool pointInPoly(float px, float py, const std::vector<cv::Point2f>& poly);

    static bool pointInPoly(float px, float py, const std::vector<Eigen::Vector2f>& poly);

    static bool pointInPolys(float px, float py, const std::vector<std::vector<Eigen::Vector2f>>& polys);

    static bool pointInPolys(float px, float py, const std::vector<std::vector<cv::Point2f>>& polys);

    static bool pointInPolygonQuick(int px, int py, const std::vector<cv::Point>& poly);

    static std::string hostPath2DockerPath(const std::string& docker_path);

    static void getBresenhamLine(int x0, int y0, int x1, int y1, int[GRID_MAP_SIZE][GRID_MAP_SIZE]);

    static void getBresenhamLine(int x0, int y0, int x1, int y1, std::vector<cv::Point>& lines);

    static void getBresenhamLine(int x0, int y0, int x1, int y1, std::vector<FSVec2>& lines);

    static FSMat4x4 makeTFrom6Dof(float yaw, float pitch, float roll, float tx, float ty, float tz);

    static FSMat4x4 makeTFrom6Dof(const EgoMotion& ego_motion);

    static FSMat4x4 inverse(const FSMat4x4& Twb);

    static Eigen::Vector2d convertLLA2UTM(const double longitude, const double latitude);

    static Eigen::Vector2d convertLLA2LTM(const double longitude, const double latitude, const Eigen::Vector3d& originPoint);

    static double convertHeadingLLA2UTM(const double longitude, const double latitude, const double heading);

    static cv::Point findB(const cv::Point& C, const cv::Point& D, float x);

    static float interpolate(const std::map<const float, const float>& expand_map, const float x);

    static CloudPose::Ptr tranfromFreespace2CloudPose(const LidarFS* fs_lidar_ptr, const EgoMotion* ego_motion);

    static std::string makeDirSimple(const std::string& _output_dir);

    static std::string getCurrentTimeStamp(int time_stamp_type = 1);

    static float cross(const cv::Point2f& A, const cv::Point2f& B, const cv::Point2f& O)
    {
      return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }
    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    static std::vector<cv::Point2f> findConvexHull(std::vector<cv::Point2f>& cld)
    {
      size_t n = cld.size(), k = 0;
      if(n <= 3)
      {
        return cld;
      }
      std::vector<cv::Point2f> polygon(2 * n);

      // Sort points lexicographically
      sort(cld.begin(), cld.end(), [&](const cv::Point2f& a, const cv::Point2f& b) -> bool { return a.x < b.x || (a.x == b.x && a.y < b.y); });

      // Build lower hull
      for(size_t i = 0; i < n; ++i)
      {
        while(k >= 2 && cross(polygon[k - 2], polygon[k - 1], cld[i]) <= 0)
        {
          k--;
        }
        polygon[k++] = cld[i];
      }

      // Build upper hull
      for(size_t i = n - 1, t = k + 1; i > 0; --i)
      {
        while(k >= t && cross(polygon[k - 2], polygon[k - 1], cld[i - 1]) <= 0)
        {
          k--;
        }
        polygon[k++] = cld[i - 1];
      }

      polygon.resize(k - 1);
      return polygon;
    }

    template<typename T>
    static std::string to_string_with_precision(const T a_value, const int n = 6)
    {
      std::ostringstream out;
      out.precision(n);
      out << std::fixed << a_value;
      return out.str();
    }

    template<typename T>
    static bool pointOnPolynomial(T x, T y, const std::vector<T>& coeff)
    {
      return y >= getPolynomialValue<T>(x, coeff);
    }

    template<typename T>
    static bool pointLowPolynomial(T x, T y, const std::vector<T>& coeff)
    {
      return y <= getPolynomialValue<T>(x, coeff);
    }

    template<typename T>
    static float getPolynomialValue(T x, const std::vector<T>& coeff)
    {
      T line_y = 0.f;
      T x_n    = 1.0f;
      for(int i = 0; i < coeff.size(); ++i)
      {
        line_y += coeff[i] * x_n;
        x_n *= x;
      }
      return line_y;
    }
  };

#define UFIX std::fixed << std::setprecision(4)
  struct TIME final
  {
    explicit TIME(const std::string& str, double thr_ms = -1.)
    {
      str_  = str;
      start = std::chrono::steady_clock::now();
      thr   = thr_ms;
    }

    ~TIME()
    {
      auto end        = std::chrono::steady_clock::now();
      auto elapsed_ms = (end - start).count() * 1e-6;
      if(thr > 0. && elapsed_ms > thr)
      {
        UWARN << UFIX << str_ << " elapsed time: " << elapsed_ms << " ms";
      }
    }

    std::chrono::steady_clock::time_point start;
    std::string                           str_;
    double                                thr;
  };

} // namespace fs

#endif //UTILS_H_
