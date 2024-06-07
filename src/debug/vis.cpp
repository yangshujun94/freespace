#include "vis.h"

#define private public

#include "vis_helper.h"
#include "peripheral/defs.h"
#include "peripheral/camera_manager.h"
#include "core/grid_map.h"
#include "core/object.h"
#include "peripheral/fs_math.h"
#include "peripheral/vehicle.h"
#include "peripheral/utils.h"
#include "fs_log.h"
#include "peripheral/map_provider/map_provider.h"

cv::Mat                              fs::Vis::m_centerFrontView(216, 384, CV_8UC3);
cv::Mat                              fs::Vis::m_fisheyeFcfView(216, 384, CV_8UC3);
cv::Mat                              fs::Vis::m_fisheyeFllView(216, 384, CV_8UC3);
cv::Mat                              fs::Vis::m_fisheyeFrrView(216, 384, CV_8UC3);
cv::Mat                              fs::Vis::m_fisheyeBcbView(216, 384, CV_8UC3);
cv::Mat                              fs::Vis::m_fisheyeBllView(216, 384, CV_8UC3);
cv::Mat                              fs::Vis::m_fisheyeBrrView(216, 384, CV_8UC3);
visualization_msgs::msg::MarkerArray fs::Vis::m_markers{};
sensor_msgs::msg::PointCloud         fs::Vis::m_fusionPointCloud{};
sensor_msgs::msg::PointCloud         fs::Vis::m_passThroughPointCloud{};
sensor_msgs::msg::PointCloud         fs::Vis::m_lidarPointCloud{};
sensor_msgs::msg::PointCloud         fs::Vis::m_perceptionFreespace{};
sensor_msgs::msg::PointCloud         fs::Vis::m_roadModelPointCloud{};

void fs::Vis::prepare(const int64_t                            timestamp,
                      const sensor_msgs::msg::CompressedImage* imageFcPtr,
                      const sensor_msgs::msg::CompressedImage* imageFisheyeFcfPtr,
                      const sensor_msgs::msg::CompressedImage* imageFisheyeFllPtr,
                      const sensor_msgs::msg::CompressedImage* imageFisheyeFrrPtr,
                      const sensor_msgs::msg::CompressedImage* imageFisheyeBcbPtr,
                      const sensor_msgs::msg::CompressedImage* imageFisheyeBllPtr,
                      const sensor_msgs::msg::CompressedImage* imageFisheyeBrrPtr)
{
  static cv::Mat centerFrontViewBkp = cv::Mat::zeros(CameraManager::getCamera(SensorId::CAMERA_CENTER_FRONT).getImageHeight() * VIS_IMAGE_VIEW_SCALING, CameraManager::getCamera(SensorId::CAMERA_CENTER_FRONT).getImageWidth() * VIS_IMAGE_VIEW_SCALING, CV_8UC3);
#if FS_CHECK(CFG_USE_FISHEYE_FS)
  static cv::Mat fisheyeFcfViewBkp = cv::Mat::zeros(CameraManager::getFisheye(SensorId::FISHEYE_FCF).getImageHeight() * VIS_IMAGE_VIEW_SCALING, CameraManager::getFisheye(SensorId::FISHEYE_FCF).getImageWidth() * VIS_IMAGE_VIEW_SCALING, CV_8UC3);
  static cv::Mat fisheyeFllViewBkp = cv::Mat::zeros(CameraManager::getFisheye(SensorId::FISHEYE_FLL).getImageHeight() * VIS_IMAGE_VIEW_SCALING, CameraManager::getFisheye(SensorId::FISHEYE_FLL).getImageWidth() * VIS_IMAGE_VIEW_SCALING, CV_8UC3);
  static cv::Mat fisheyeFrrViewBkp = cv::Mat::zeros(CameraManager::getFisheye(SensorId::FISHEYE_FRR).getImageHeight() * VIS_IMAGE_VIEW_SCALING, CameraManager::getFisheye(SensorId::FISHEYE_FRR).getImageWidth() * VIS_IMAGE_VIEW_SCALING, CV_8UC3);
  static cv::Mat fisheyeBcbViewBkp = cv::Mat::zeros(CameraManager::getFisheye(SensorId::FISHEYE_BCB).getImageHeight() * VIS_IMAGE_VIEW_SCALING, CameraManager::getFisheye(SensorId::FISHEYE_BCB).getImageWidth() * VIS_IMAGE_VIEW_SCALING, CV_8UC3);
  static cv::Mat fisheyeBllViewBkp = cv::Mat::zeros(CameraManager::getFisheye(SensorId::FISHEYE_BLL).getImageHeight() * VIS_IMAGE_VIEW_SCALING, CameraManager::getFisheye(SensorId::FISHEYE_BLL).getImageWidth() * VIS_IMAGE_VIEW_SCALING, CV_8UC3);
  static cv::Mat fisheyeBrrViewBkp = cv::Mat::zeros(CameraManager::getFisheye(SensorId::FISHEYE_BRR).getImageHeight() * VIS_IMAGE_VIEW_SCALING, CameraManager::getFisheye(SensorId::FISHEYE_BRR).getImageWidth() * VIS_IMAGE_VIEW_SCALING, CV_8UC3);
#endif
  if(nullptr != imageFcPtr)
  {
    prepareRawImage(*imageFcPtr, SensorId::CAMERA_CENTER_FRONT, m_centerFrontView);
    m_centerFrontView.copyTo(centerFrontViewBkp);
  }
  else
  {
    centerFrontViewBkp.copyTo(m_centerFrontView);
  }

#if FS_CHECK(CFG_USE_FISHEYE_FS)
  if(nullptr != imageFisheyeFcfPtr)
  {
    prepareRawCylinderImage(*imageFisheyeFcfPtr, SensorId::FISHEYE_FCF, m_fisheyeFcfView);
    m_fisheyeFcfView.copyTo(fisheyeFcfViewBkp);
  }
  else
  {
    fisheyeFcfViewBkp.copyTo(m_fisheyeFcfView);
  }

  if(nullptr != imageFisheyeFllPtr)
  {
    prepareRawCylinderImage(*imageFisheyeFllPtr, SensorId::FISHEYE_FLL, m_fisheyeFllView);
    m_fisheyeFllView.copyTo(fisheyeFllViewBkp);
  }
  else
  {
    fisheyeFllViewBkp.copyTo(m_fisheyeFllView);
  }

  if(nullptr != imageFisheyeFrrPtr)
  {
    prepareRawCylinderImage(*imageFisheyeFrrPtr, SensorId::FISHEYE_FRR, m_fisheyeFrrView);
    m_fisheyeFrrView.copyTo(fisheyeFrrViewBkp);
  }
  else
  {
    fisheyeFrrViewBkp.copyTo(m_fisheyeFrrView);
  }

  if(nullptr != imageFisheyeBllPtr)
  {
    prepareRawCylinderImage(*imageFisheyeBllPtr, SensorId::FISHEYE_BLL, m_fisheyeBllView);
    m_fisheyeBllView.copyTo(fisheyeBllViewBkp);
  }
  else
  {
    fisheyeBllViewBkp.copyTo(m_fisheyeBllView);
  }

  if(nullptr != imageFisheyeBrrPtr)
  {
    prepareRawCylinderImage(*imageFisheyeBrrPtr, SensorId::FISHEYE_BRR, m_fisheyeBrrView);
    m_fisheyeBrrView.copyTo(fisheyeBrrViewBkp);
  }
  else
  {
    fisheyeBrrViewBkp.copyTo(m_fisheyeBrrView);
  }

  if(nullptr != imageFisheyeBcbPtr)
  {
    prepareRawCylinderImage(*imageFisheyeBcbPtr, SensorId::FISHEYE_BCB, m_fisheyeBcbView);
    m_fisheyeBcbView.copyTo(fisheyeBcbViewBkp);
  }
  else
  {
    fisheyeBcbViewBkp.copyTo(m_fisheyeBcbView);
  }
#endif

  drawTimestamp(timestamp);
  clearMarkers();
  drawEgo();
  drawSafetyZone();
}

void fs::Vis::prepareRawImage(const sensor_msgs::msg::CompressedImage& src, const fs::SensorId sensorId, cv::Mat& dst)
{
  assert(!Utils::isFisheye(sensorId));
  cv::Mat       img    = cv::imdecode(src.data, cv::IMREAD_COLOR);
  const Camera& camera = CameraManager::getCamera(sensorId);
  cv::resize(img, dst, cv::Size{VIS_IMAGE_VIEW_SCALING * camera.m_imageWidth, VIS_IMAGE_VIEW_SCALING * camera.m_imageHeight});
}

#if FS_CHECK(CFG_USE_FISHEYE_FS)
void fs::Vis::prepareRawCylinderImage(const sensor_msgs::msg::CompressedImage& src, const fs::SensorId sensorId, cv::Mat& dst)
{
  assert(Utils::isFisheye(sensorId));
  cv::Mat        img     = cv::imdecode(src.data, cv::IMREAD_COLOR);
  const Fisheye& fisheye = CameraManager::getFisheye(sensorId);
  cv::resize(img, img, cv::Size{fisheye.m_imageWidth, fisheye.m_imageHeight});
  cv::remap(img, dst, fisheye.m_map1, fisheye.m_map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  cv::resize(dst, dst, cv::Size{VIS_IMAGE_VIEW_SCALING * fisheye.m_imageWidth, VIS_IMAGE_VIEW_SCALING * fisheye.m_imageHeight});
}
#endif

void fs::Vis::drawTimestamp(const int64_t timestamp)
{
  std::stringstream str;
  str << timestamp / 1000;
  cv::putText(m_centerFrontView, str.str(), cv::Point2i(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_R);
  cv::putText(m_fisheyeFcfView, str.str(), cv::Point2i(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_R);
}

void fs::Vis::drawEgo()
{
  constexpr float length = 5.f;

  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "ego vehicle";
    marker.id              = 0;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.type            = visualization_msgs::msg::Marker::CYLINDER;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.color           = VisHelper::getColor(RvizColor::RED);
    marker.pose            = VisHelper::convertPose(Eigen::Translation3d(0.5 * length, 0, 0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));

    marker.scale.x = 0.1f * length;
    marker.scale.y = 0.1f * length;
    marker.scale.z = length;

    m_markers.markers.push_back(marker);
  }

  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "ego vehicle";
    marker.id              = 1;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.type            = visualization_msgs::msg::Marker::CYLINDER;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.color           = VisHelper::getColor(RvizColor::GREEN);
    marker.pose            = VisHelper::convertPose(Eigen::Translation3d(0, 0.5 * length, 0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));

    marker.scale.x = 0.1f * length;
    marker.scale.y = 0.1f * length;
    marker.scale.z = length;

    m_markers.markers.push_back(marker);
  }

  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "ego vehicle";
    marker.id              = 2;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.type            = visualization_msgs::msg::Marker::CYLINDER;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.color           = VisHelper::getColor(RvizColor::BLUE);
    marker.pose            = VisHelper::convertPose(Eigen::Translation3d(0, 0, 0.5 * length) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));

    marker.scale.x = 0.1f * length;
    marker.scale.y = 0.1f * length;
    marker.scale.z = length;

    m_markers.markers.push_back(marker);
  }

  {
    const auto& veh     = Vehicle::getVehicle();
    const float centerX = -(veh.m_vehicleLength * 0.5f - veh.m_distBumper2Ego);

    const std::array<EVector2, 4> cuboidPointsEgo = Utils::createCuboid(centerX,
                                                                        0.0f,
                                                                        veh.m_vehicleLength,
                                                                        veh.m_vehicleWidth,
                                                                        0.0f);

    m_markers.markers.push_back(VisHelper::createWireframeCuboid(cuboidPointsEgo, "ego vehicle", 3, VisHelper::getColor(RvizColor::YELLOW)));
  }
}

void fs::Vis::drawSafetyZone()
{
  const auto& veh = Vehicle::getVehicle();
  m_markers.markers.push_back(VisHelper::createWireframeCuboid(rect2ShapePoints(veh.m_dangerZone), "danger zone", 3, VisHelper::getColor(RvizColor::RED)));
  m_markers.markers.push_back(VisHelper::createWireframeCuboid(rect2ShapePoints(veh.m_cautionZone), "caution zone", 3, VisHelper::getColor(RvizColor::ORANGE)));
}

void fs::Vis::clearMarkers()
{
  auto markers = m_markers.markers;
  m_markers.markers.clear();
  for(auto& marker : markers)
  {
    if(marker.action != visualization_msgs::msg::Marker::DELETE)
    {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      m_markers.markers.push_back(marker);
    }
  }
}

void fs::Vis::drawFusionPointCloud(const GridMap& gridMap, const std::vector<EVector2>& passThroughPoints)
{
  m_fusionPointCloud.header.frame_id = "ego";
  m_fusionPointCloud.points.clear();
  m_fusionPointCloud.points.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_fusionPointCloud.channels.resize(10);
  m_fusionPointCloud.channels[0].name = "row";
  m_fusionPointCloud.channels[1].name = "col";
  m_fusionPointCloud.channels[2].name = "row prev";
  m_fusionPointCloud.channels[3].name = "col prev";
  m_fusionPointCloud.channels[4].name = "logit prev";
  m_fusionPointCloud.channels[5].name = "logit lidar";
  m_fusionPointCloud.channels[6].name = "logit";
  m_fusionPointCloud.channels[7].name = "history";
  m_fusionPointCloud.channels[8].name = "free in cam";
  m_fusionPointCloud.channels[9].name = "border type";
  m_fusionPointCloud.channels[0].values.clear();
  m_fusionPointCloud.channels[1].values.clear();
  m_fusionPointCloud.channels[2].values.clear();
  m_fusionPointCloud.channels[3].values.clear();
  m_fusionPointCloud.channels[4].values.clear();
  m_fusionPointCloud.channels[5].values.clear();
  m_fusionPointCloud.channels[6].values.clear();
  m_fusionPointCloud.channels[7].values.clear();
  m_fusionPointCloud.channels[8].values.clear();
  m_fusionPointCloud.channels[9].values.clear();
  m_fusionPointCloud.channels[0].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_fusionPointCloud.channels[1].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_fusionPointCloud.channels[2].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_fusionPointCloud.channels[3].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_fusionPointCloud.channels[4].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_fusionPointCloud.channels[5].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_fusionPointCloud.channels[6].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_fusionPointCloud.channels[7].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_fusionPointCloud.channels[8].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_fusionPointCloud.channels[9].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);

  if(gridMap.isToPublish())
  {
    for(int j = 0; j < GRID_MAP_SIZE_ROWS; ++j)
    {
      for(int i = 0; i < GRID_MAP_SIZE_COLS; ++i)
      {
        const auto& grid = gridMap.m_gridMapBuffer.front()[j][i];
        if(grid.isBorder && grid.logit > LOGIT_OCCUPIED_THRESH)
        {
          auto& point = m_fusionPointCloud.points.emplace_back();
          point.x     = grid.point.x();
          point.y     = grid.point.y();
          point.z     = 0.f;
          m_fusionPointCloud.channels[0].values.emplace_back(j);
          m_fusionPointCloud.channels[1].values.emplace_back(i);
          m_fusionPointCloud.channels[6].values.emplace_back(grid.logit);
          m_fusionPointCloud.channels[8].values.emplace_back(grid.isFreeInCam);
          m_fusionPointCloud.channels[9].values.emplace_back(grid.isBorder);
#if FS_CHECK(CFG_DEBUG_GRID)
          m_fusionPointCloud.channels[4].values.emplace_back(grid.logitPrev);
          m_fusionPointCloud.channels[5].values.emplace_back(grid.logitLidar);
          m_fusionPointCloud.channels[7].values.emplace_back(grid.history);
          m_fusionPointCloud.channels[2].values.emplace_back(grid.rowPrev);
          m_fusionPointCloud.channels[3].values.emplace_back(grid.colPrev);
#endif
        }
      }
    }
  }

  drawPassThroughPointCloud(passThroughPoints);
}

void fs::Vis::drawPassThroughPointCloud(const std::vector<EVector2>& passThroughPoints)
{
  m_passThroughPointCloud.header.frame_id = "ego";
  m_passThroughPointCloud.points.clear();
  m_passThroughPointCloud.points.reserve(passThroughPoints.size());

  for(const auto& passThroughPoint : passThroughPoints)
  {
    auto& point = m_passThroughPointCloud.points.emplace_back();
    point.x     = passThroughPoint.x();
    point.y     = passThroughPoint.y();
    point.z     = 0.f;
  }
}

void fs::Vis::drawPerceptionFreespace(const uto::proto::PerceptionFreespace& perceptionFs)
{
  m_perceptionFreespace.header.frame_id = "ego";
  m_perceptionFreespace.points.clear();
  m_perceptionFreespace.points.reserve(perceptionFs.perception_gridmap_size());
  m_perceptionFreespace.channels.resize(2);
  m_perceptionFreespace.channels[0].name = "label";
  m_perceptionFreespace.channels[1].name = "object id";
  m_perceptionFreespace.channels[0].values.clear();
  m_perceptionFreespace.channels[1].values.clear();
  m_perceptionFreespace.channels[0].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);
  m_perceptionFreespace.channels[1].values.reserve(GRID_MAP_SIZE_ROWS * GRID_MAP_SIZE_COLS);

  const EMatrix2 rot = Eigen::Rotation2Df{perceptionFs.origin_point_theta()}.toRotationMatrix();
  const EVector2 vec{perceptionFs.origin_point().x(), perceptionFs.origin_point().y()};

  for(const auto& grid : perceptionFs.perception_gridmap())
  {
    if(uto::proto::PerceptionGrid_MoveStatus::PerceptionGrid_MoveStatus_MOVING != grid.move_status())
    {
      assert(grid.label() != uto::proto::PerceptionGrid_Label_UNOCCUPIED);

      auto& point = m_perceptionFreespace.points.emplace_back();

      const EVector2 pointEgo = rot.transpose() * (EVector2{grid.position().x(), grid.position().y()} - vec);

      point.x = pointEgo.x();
      point.y = pointEgo.y();
      point.z = grid.bottom();

      m_perceptionFreespace.channels[0].values.emplace_back(grid.label());
      m_perceptionFreespace.channels[1].values.emplace_back(grid.object_id());
    }
  }
}

void fs::Vis::drawLidarPointCloud(const uto::proto::PerceptionFreespace& fsLidar)
{
  m_lidarPointCloud.header.frame_id = "ego";
  m_lidarPointCloud.points.clear();
  m_lidarPointCloud.points.reserve(fsLidar.perception_gridmap_size());

  for(const auto& grid : fsLidar.perception_gridmap())
  {
    assert(grid.label() != uto::proto::PerceptionGrid_Label_UNOCCUPIED);

    geometry_msgs::msg::Point32& point = m_lidarPointCloud.points.emplace_back();

    point.x = grid.position().x();
    point.y = grid.position().y();
    point.z = grid.bottom();
  }
}

void fs::Vis::drawRoadModelPointCloud(const uto::proto::RoadModel* RoadModelPoints)
{
  if(RoadModelPoints == nullptr)
  {
    return;
  }
  m_roadModelPointCloud.header.frame_id = "ego";
  m_roadModelPointCloud.points.clear();
  m_roadModelPointCloud.points.reserve(RoadModelPoints->grids().size());

  for(int i = 0; i < RoadModelPoints->grids().size();)
  {
    geometry_msgs::msg::Point32& point = m_roadModelPointCloud.points.emplace_back();
    point.x                            = (RoadModelPoints->grids()[i] - RoadModelPoints->ego_x_index()) * RoadModelPoints->grid_resolution();
    point.y                            = (-RoadModelPoints->grids()[i + 1] + RoadModelPoints->ego_y_index()) * RoadModelPoints->grid_resolution();
    point.z                            = RoadModelPoints->grids()[i + 2] * RoadModelPoints->height_resolution();
    i += 3;
  }
  return;
}

void fs::Vis::drawLidarPointImage(const uto::proto::PerceptionFreespace& fsLidar)
{
  const Camera& camera = CameraManager::getCamera(SensorId::CAMERA_CENTER_FRONT);
  for(const auto& lidarPoint : fsLidar.perception_gridmap())
  {
    const EVector3 pointCam = camera.transformEgo2Cam(EVector3{lidarPoint.position().x(), lidarPoint.position().y(), 0.f});
    if(pointCam.z() > 0.f)
    {
      const Eigen::Vector2i pointImg = camera.transformCam2Img(pointCam);
      if(pointImg.x() > 0 && pointImg.x() * VIS_IMAGE_VIEW_SCALING < m_centerFrontView.cols && pointImg.y() * VIS_IMAGE_VIEW_SCALING < m_centerFrontView.rows && pointImg.y() > 0)
      {
        cv::circle(m_centerFrontView,
                   VIS_IMAGE_VIEW_SCALING * cv::Point2i{pointImg.x(), pointImg.y()},
                   0,
                   COLOR_GREY_BLUE,
                   -1);
      }
    }
  }
}

#if FS_CHECK(CFG_USE_FISHEYE_FS)
void fs::Vis::drawLidarPointFisheye(const uto::proto::PerceptionFreespace& lidarFs, const fs::SensorId sensorId)
{
  const Fisheye& fisheye = CameraManager::getFisheye(sensorId);

  cv::Mat* matPtr = nullptr;

  switch(sensorId)
  {
  case SensorId::FISHEYE_FCF:
    matPtr = &m_fisheyeFcfView;
    break;
  case SensorId::FISHEYE_FLL:
    matPtr = &m_fisheyeFllView;
    break;
  case SensorId::FISHEYE_FRR:
    matPtr = &m_fisheyeFrrView;
    break;
  case SensorId::FISHEYE_BCB:
    matPtr = &m_fisheyeBcbView;
    break;
  case SensorId::FISHEYE_BLL:
    matPtr = &m_fisheyeBllView;
    break;
  case SensorId::FISHEYE_BRR:
    matPtr = &m_fisheyeBrrView;
    break;
  default:
    assert(false && "not supported");
  }

  for(const auto& lidarPoint : lidarFs.perception_gridmap())
  {
    const EVector3 pointCam = fisheye.transformEgo2Cam(EVector3{lidarPoint.position().x(), lidarPoint.position().y(), 0.f});
    if(pointCam.z() > 0.f)
    {
      const EVector2 pointCylinder = fisheye.transformCam2Cylinder(pointCam);
      if(pointCylinder.x() > 0 && pointCylinder.x() * VIS_IMAGE_VIEW_SCALING < matPtr->cols && pointCylinder.y() * VIS_IMAGE_VIEW_SCALING < matPtr->rows && pointCylinder.y() > 0)
      {
        cv::circle(*matPtr,
                   VIS_IMAGE_VIEW_SCALING * cv::Point2i{pointCylinder.x(), pointCylinder.y()},
                   0,
                   COLOR_GREY_BLUE,
                   -1);
      }
    }
  }
}
#endif

void fs::Vis::drawDrivingBoundary(const uto::proto::PerceptionFreespace& lidarFs)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "ego";
  marker.ns              = "boundary layer";
  marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action          = visualization_msgs::msg::Marker::ADD;
  marker.lifetime        = rclcpp::Duration(0, 0);
  marker.pose            = VisHelper::getIdentityPose();
  marker.scale           = VisHelper::getScale(RvizScale::SMALL);
  marker.scale.y         = 0.f;
  marker.scale.z         = 0.f;
  marker.color           = VisHelper::getColor(RvizColor::PINK);

  for(int layerIdx = 0; layerIdx < lidarFs.drivable_boundary().boundary_layers_size(); ++layerIdx)
  {
    for(int boundaryIdx = 0; boundaryIdx < lidarFs.drivable_boundary().boundary_layers(layerIdx).boundaries_size(); ++boundaryIdx)
    {
      marker.id = boundaryIdx + layerIdx * 100000;
      marker.points.clear();

      const auto& boundary = lidarFs.drivable_boundary().boundary_layers(layerIdx).boundaries(boundaryIdx);
      for(int pointIdx = 0; pointIdx < boundary.points_size(); pointIdx += 2)
      {
        marker.points.push_back(VisHelper::convertPoint(EVector2{boundary.points(pointIdx), boundary.points(pointIdx + 1)}));
      }
      marker.points.push_back(marker.points[0]);
      m_markers.markers.push_back(marker);
    }
  }
}

void fs::Vis::drawObject(const Object& obj)
{
  assert(!obj.m_contour.empty() && !obj.m_dilatedContour.empty());

  // original contour
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "obj contour";
    marker.id              = obj.m_id;
    marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.pose            = VisHelper::getIdentityPose();

    marker.scale   = VisHelper::getScale(RvizScale::SMALL);
    marker.scale.y = 0.f;
    marker.scale.z = 0.f;
    marker.color   = VisHelper::getColor(RvizColor::YELLOW);
    marker.points.clear();

    for(const auto& pointEgo : obj.m_contour)
    {
      marker.points.push_back(VisHelper::convertPoint(pointEgo));
    }
    marker.points.push_back(marker.points[0]);
    m_markers.markers.push_back(marker);
  }

  // dilated contour
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "obj dilated contour";
    marker.id              = obj.m_id;
    marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.pose            = VisHelper::getIdentityPose();

    marker.scale   = VisHelper::getScale(RvizScale::SMALL);
    marker.scale.y = 0.f;
    marker.scale.z = 0.f;
    marker.color   = VisHelper::getColor(RvizColor::GREEN);
    marker.points.clear();

    for(const auto& pointEgo : obj.m_dilatedContour)
    {
      marker.points.push_back(VisHelper::convertPoint(pointEgo));
    }
    marker.points.push_back(marker.points[0]);
    m_markers.markers.push_back(marker);
  }

  // text
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "object id";
    marker.id              = obj.m_id;
    marker.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.pose.position   = VisHelper::convertPoint(obj.m_contour.front());
    marker.scale           = VisHelper::getScale(RvizScale::LARGE);
    marker.color           = VisHelper::getColor(RvizColor::RED);
    std::stringstream ss;
    ss << "#" << obj.m_id;
    marker.text = ss.str();

    m_markers.markers.push_back(marker);
  }
}

void fs::Vis::drawCameraFs(const uto::proto::CameraFreespace& camFs)
{
  for(const auto& fsPoint : camFs.camera_freespace_points())
  {
    const CameraClass cameraClass = cameraFreespacePointType2CameraClass(fsPoint.type());
    const cv::Scalar  color       = cameraClass2Color(cameraClass);
    for(size_t k = 0; k < m_centerFrontView.channels(); ++k)
    {
      const int v = std::clamp(fsPoint.point().y(), 0.f, static_cast<float>(camFs.image_height() - 1)) * VIS_IMAGE_VIEW_SCALING;
      const int u = std::clamp(fsPoint.point().x(), 0.f, static_cast<float>(camFs.image_width() - 1)) * VIS_IMAGE_VIEW_SCALING;

      m_centerFrontView.at<cv::Vec3b>(v, u)[k] = color[k];
    }
  }
}

#if FS_CHECK(CFG_USE_FISHEYE_FS)
void fs::Vis::drawFisheyeFs(const uto::proto::CameraFreespace& fisheyeFs, const fs::SensorId sensorId)
{
  const Fisheye& fisheye = CameraManager::getFisheye(sensorId);

  cv::Mat* matPtr = nullptr;

  switch(sensorId)
  {
  case SensorId::FISHEYE_FCF:
    matPtr = &m_fisheyeFcfView;
    break;
  case SensorId::FISHEYE_FLL:
    matPtr = &m_fisheyeFllView;
    break;
  case SensorId::FISHEYE_FRR:
    matPtr = &m_fisheyeFrrView;
    break;
  case SensorId::FISHEYE_BCB:
    matPtr = &m_fisheyeBcbView;
    break;
  case SensorId::FISHEYE_BLL:
    matPtr = &m_fisheyeBllView;
    break;
  case SensorId::FISHEYE_BRR:
    matPtr = &m_fisheyeBrrView;
    break;
  default:
    assert(false && "not supported");
  }

  std::vector<cv::Point2f> pointFisheyeVec;
  pointFisheyeVec.reserve(fisheyeFs.camera_freespace_points_size());
  for(const auto& fsPoint : fisheyeFs.camera_freespace_points())
  {
    pointFisheyeVec.emplace_back(fsPoint.point().x(), fsPoint.point().y());
  }
  const std::vector<EVector2> pointCylinderVec = fisheye.transformFisheye2Cylinder(pointFisheyeVec);
  assert(pointCylinderVec.size() == fisheyeFs.camera_freespace_points_size());

  for(int i = FISHEYE_INVALID_MARGIN; i < fisheyeFs.camera_freespace_points_size() - FISHEYE_INVALID_MARGIN; ++i)
  {
    const auto&       fsPoint       = fisheyeFs.camera_freespace_points(i);
    const CameraClass cameraClass   = cameraFreespacePointType2CameraClass(fsPoint.type());
    const cv::Scalar  color         = cameraClass2Color(cameraClass);
    const EVector2&   pointCylinder = pointCylinderVec[i];

    if(pointCylinder.x() >= 0 && pointCylinder.x() * VIS_IMAGE_VIEW_SCALING < matPtr->cols)
    {
      for(size_t k = 0; k < matPtr->channels(); ++k)
      {
        const int v = std::clamp(pointCylinder.y(), 0.f, static_cast<float>(fisheyeFs.image_height() - 1)) * VIS_IMAGE_VIEW_SCALING;
        const int u = std::clamp(pointCylinder.x(), 0.f, static_cast<float>(fisheyeFs.image_width() - 1)) * VIS_IMAGE_VIEW_SCALING;

        matPtr->at<cv::Vec3b>(v, u)[k] = color[k];
      }
    }
  }
}
#endif

cv::Scalar fs::Vis::cameraClass2Color(const CameraClass cameraClass)
{
  cv::Scalar color{};
  switch(cameraClass)
  {
  case INVALID:
    color = cv::Scalar{0, 0, 0};
    break;
  case STATIC_BORDER:
    color = COLOR_B;
    break;
  case VPR:
    color = COLOR_P;
    break;
  case NON_OBSTACLE:
    color = COLOR_ORANGE;
    break;
  default:
    assert(false && "unknown camera class");
    break;
  }

  return color;
}

std::array<fs::EVector2, 4> fs::Vis::rect2ShapePoints(const fs::Rect& rect)
{
  std::array<EVector2, 4> shapePoints;
  shapePoints[ShapePointIndex::RL] = EVector2{rect.minX, rect.maxY};
  shapePoints[ShapePointIndex::FL] = EVector2{rect.maxX, rect.maxY};
  shapePoints[ShapePointIndex::FR] = EVector2{rect.maxX, rect.minY};
  shapePoints[ShapePointIndex::RR] = EVector2{rect.minX, rect.minY};

  return shapePoints;
}

void fs::Vis::drawGates(const std::vector<fs::Rect>& gateRects)
{
  for(int i = 0; i < gateRects.size(); i++)
  {
    visualization_msgs::msg::Marker marker;
    m_markers.markers.push_back(VisHelper::createWireframeCuboid(rect2ShapePoints(gateRects[i]), "gate", i, VisHelper::getColor(RvizColor::GREEN)));
  }
}

void fs::Vis::drawGeoFence()
{
  int id = -1;

  const MapProvider& mapProvider = MapProvider::getMapProvider();
  const Vehicle&     vehicle     = Vehicle::getVehicle();

  for(const auto& area : mapProvider.getWeighbridgeAreaLtm())
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "weighbridge fence";
    marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.pose            = VisHelper::getIdentityPose();
    marker.scale           = VisHelper::getScale(RvizScale::SMALL);
    marker.scale.y         = 0.f;
    marker.scale.z         = 0.f;
    marker.color           = VisHelper::getColor(RvizColor::GREEN);
    marker.points.clear();
    marker.id = ++id;

    for(const auto& pointLtm : area)
    {
      EVector3 pointEgo = vehicle.transformLtm2Ego(Eigen::Vector3d(pointLtm.x(), pointLtm.y(), 0));
      marker.points.push_back(VisHelper::convertPoint(EVector2{pointEgo.x(), pointEgo.y()}));
    }
    marker.points.push_back(marker.points[0]);
    m_markers.markers.push_back(marker);
  }

  for(const auto& pointLtms : mapProvider.getTunnelAreaLtm())
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "tunnel fence";
    marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.pose            = VisHelper::getIdentityPose();
    marker.scale           = VisHelper::getScale(RvizScale::SMALL);
    marker.scale.y         = 0.f;
    marker.scale.z         = 0.f;
    marker.color           = VisHelper::getColor(RvizColor::GREEN);
    marker.points.clear();
    marker.id = ++id;

    for(const auto& pointLtm : pointLtms)
    {
      EVector3 pointEgo = vehicle.transformLtm2Ego(Eigen::Vector3d(pointLtm.x(), pointLtm.y(), 0));
      marker.points.push_back(VisHelper::convertPoint(EVector2{pointEgo.x(), pointEgo.y()}));
    }
    marker.points.push_back(marker.points[0]);
    m_markers.markers.push_back(marker);
  }
}