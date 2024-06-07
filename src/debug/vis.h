#ifndef VIS_H_
#define VIS_H_

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <perception_freespace.pb.h>
#include <rclcpp/time.hpp>
#include <perception_camera.pb.h>
#include "peripheral/switcher.h"
#include "peripheral/types.h"
#include <perception_road_model.pb.h>

#define CFG_VIS_DRAW_PERCEPTION_FREESPACE FS_SWITCH(0 && FS_CHECK(CFG_VIS_ENABLE))

namespace fs
{
  static constexpr float VIS_IMAGE_VIEW_SCALING = 0.3F;

  class GridMap;
  class Object;

  class Vis
  {
  public:
    static const cv::Mat&                              getCenterFrontView() { return m_centerFrontView; }
    static const cv::Mat&                              getFisheyeFcfView() { return m_fisheyeFcfView; }
    static const cv::Mat&                              getFisheyeFllView() { return m_fisheyeFllView; }
    static const cv::Mat&                              getFisheyeFrrView() { return m_fisheyeFrrView; }
    static const cv::Mat&                              getFisheyeBcbView() { return m_fisheyeBcbView; }
    static const cv::Mat&                              getFisheyeBllView() { return m_fisheyeBllView; }
    static const cv::Mat&                              getFisheyeBrrView() { return m_fisheyeBrrView; }
    static const visualization_msgs::msg::MarkerArray& getMarkerArray() { return m_markers; }
    static const sensor_msgs::msg::PointCloud&         getFusionPointCloud() { return m_fusionPointCloud; }
    static const sensor_msgs::msg::PointCloud&         getPassThroughPointCloud() { return m_passThroughPointCloud; }
    static const sensor_msgs::msg::PointCloud&         getLidarPointCloud() { return m_lidarPointCloud; }
    static const sensor_msgs::msg::PointCloud&         getPerceptionFreespace() { return m_perceptionFreespace; }
    static const sensor_msgs::msg::PointCloud&         getRoadModelPointCloud() { return m_roadModelPointCloud; }

    static void prepare(const int64_t                            timestamp,
                        const sensor_msgs::msg::CompressedImage* imageFcPtr,
                        const sensor_msgs::msg::CompressedImage* imageFisheyeFcfPtr,
                        const sensor_msgs::msg::CompressedImage* imageFisheyeFllPtr,
                        const sensor_msgs::msg::CompressedImage* imageFisheyeFrrPtr,
                        const sensor_msgs::msg::CompressedImage* imageFisheyeBcbPtr,
                        const sensor_msgs::msg::CompressedImage* imageFisheyeBllPtr,
                        const sensor_msgs::msg::CompressedImage* imageFisheyeBrrPtr);

    static void drawFusionPointCloud(const GridMap& gridMap, const std::vector<EVector2>& passThroughPoints);
    static void drawLidarPointCloud(const uto::proto::PerceptionFreespace& fsLidar);
    static void drawLidarPointImage(const uto::proto::PerceptionFreespace& fsLidar);
    static void drawObject(const Object& obj);
    static void drawCameraFs(const uto::proto::CameraFreespace& camFs);
    static void drawPerceptionFreespace(const uto::proto::PerceptionFreespace& perceptionFs);
    static void drawGates(const std::vector<fs::Rect>& gateRects);
    static void drawLidarPointFisheye(const uto::proto::PerceptionFreespace& lidarFs, const SensorId sensorId);
    static void drawFisheyeFs(const uto::proto::CameraFreespace& fisheyeFs, const SensorId sensorId);
    static void drawGeoFence();
    static void drawDrivingBoundary(const uto::proto::PerceptionFreespace& lidarFs);
    static void drawRoadModelPointCloud(const uto::proto::RoadModel* RoadModelPoints);

  private:
    static void drawEgo();
    static void drawSafetyZone();
    static void clearMarkers();
    static void drawTimestamp(const int64_t timestamp);
    static void prepareRawImage(const sensor_msgs::msg::CompressedImage& src, const SensorId sensorId, cv::Mat& dst);
    static void prepareRawCylinderImage(const sensor_msgs::msg::CompressedImage& src, const SensorId sensorId, cv::Mat& dst);
    static void drawPassThroughPointCloud(const std::vector<EVector2>& passThroughPoints);

    static cv::Scalar              cameraClass2Color(const CameraClass cameraClass);
    static std::array<EVector2, 4> rect2ShapePoints(const Rect& rect);

    static cv::Mat m_centerFrontView;
    static cv::Mat m_fisheyeFcfView;
    static cv::Mat m_fisheyeFllView;
    static cv::Mat m_fisheyeFrrView;
    static cv::Mat m_fisheyeBcbView;
    static cv::Mat m_fisheyeBllView;
    static cv::Mat m_fisheyeBrrView;

    static visualization_msgs::msg::MarkerArray m_markers;
    static sensor_msgs::msg::PointCloud         m_fusionPointCloud;
    static sensor_msgs::msg::PointCloud         m_passThroughPointCloud;
    static sensor_msgs::msg::PointCloud         m_lidarPointCloud;
    static sensor_msgs::msg::PointCloud         m_perceptionFreespace;
    static sensor_msgs::msg::PointCloud         m_roadModelPointCloud;
  };
} // namespace fs

#endif //VIS_H_
