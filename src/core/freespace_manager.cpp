#include "freespace_manager.h"
#include "peripheral/vehicle.h"
#include "debug/vis.h"
#include "debug/fs_log.h"
#include "peripheral/camera.h"
#include "peripheral/camera_manager.h"
#include <numeric>
#include "peripheral/utils.h"
#include "peripheral/map_provider/map_provider.h"

void fs::FreespaceManager::fuseFreespace(const int64_t                          timestampNs,
                                         const uto::proto::PerceptionFreespace& fsLidar,
                                         const uto::proto::CameraFreespace*     fsCfPtr,
                                         const uto::proto::CameraFreespace*     fsFisheyeFcfPtr,
                                         const uto::proto::CameraFreespace*     fsFisheyeFllPtr,
                                         const uto::proto::CameraFreespace*     fsFisheyeFrrPtr,
                                         const uto::proto::CameraFreespace*     fsFisheyeBcbPtr,
                                         const uto::proto::CameraFreespace*     fsFisheyeBllPtr,
                                         const uto::proto::CameraFreespace*     fsFisheyeBrrPtr,
                                         const uto::proto::RoadModel*           roadModelPtr,
                                         const uto::proto::PerceptionObstacles* odPtr,
                                         const uto::proto::PerceptionGates*     gatePtr)
{
  auto start = std::chrono::steady_clock::now();

  UWARN << "start to fuse freespace, timestamp: " << timestampNs << " ns";

  m_gridMap.updateEgo();
  m_gridMap.applyPrior();
  refineRoadModel(roadModelPtr, fsLidar);

  m_objectList.clear();
  if(nullptr != odPtr)
  {
    fillObjectList(*odPtr, timestampNs);
  }

  const std::vector<Rect> gateRects = fillGateList(gatePtr);

  if(nullptr != fsCfPtr && fsCfPtr->image_width() > 0)
  {
    m_camFsScale = static_cast<float>(fsCfPtr->camera_freespace_points_size()) / fsCfPtr->image_width();
  }

#if FS_CHECK(CFG_USE_FISHEYE_FS)
  if(nullptr != fsFisheyeFcfPtr && fsFisheyeFcfPtr->image_width() > 0 && !m_isFisheye2CylinderMapValid)
  {
    setupFisheye2CylinderMap(*fsFisheyeFcfPtr, SensorId::FISHEYE_FCF);
  }

  const std::vector<EVector2> fsPointsCylinderFcf = nullptr != fsFisheyeFcfPtr ? transformFisheyeFs2Cylinder(*fsFisheyeFcfPtr, SensorId::FISHEYE_FCF) : std::vector<EVector2>{};
  const std::vector<EVector2> fsPointsCylinderFll = nullptr != fsFisheyeFllPtr ? transformFisheyeFs2Cylinder(*fsFisheyeFllPtr, SensorId::FISHEYE_FLL) : std::vector<EVector2>{};
  const std::vector<EVector2> fsPointsCylinderFrr = nullptr != fsFisheyeFrrPtr ? transformFisheyeFs2Cylinder(*fsFisheyeFrrPtr, SensorId::FISHEYE_FRR) : std::vector<EVector2>{};
  const std::vector<EVector2> fsPointsCylinderBcb = nullptr != fsFisheyeBcbPtr ? transformFisheyeFs2Cylinder(*fsFisheyeBcbPtr, SensorId::FISHEYE_BCB) : std::vector<EVector2>{};
  const std::vector<EVector2> fsPointsCylinderBll = nullptr != fsFisheyeBllPtr ? transformFisheyeFs2Cylinder(*fsFisheyeBllPtr, SensorId::FISHEYE_BLL) : std::vector<EVector2>{};
  const std::vector<EVector2> fsPointsCylinderBrr = nullptr != fsFisheyeBrrPtr ? transformFisheyeFs2Cylinder(*fsFisheyeBrrPtr, SensorId::FISHEYE_BRR) : std::vector<EVector2>{};
#endif

  const Camera& camera = CameraManager::getCamera(SensorId::CAMERA_CENTER_FRONT);
  m_camOriginEgo       = camera.transformCam2Ego(EVector3::Zero()).x();
  for(const auto& lidarPoint : fsLidar.perception_gridmap())
  {
    assert(lidarPoint.label() != uto::proto::PerceptionGrid_Label_UNOCCUPIED);

    const GridMap::Index idx = GridMap::pos2idx(EVector2{lidarPoint.position().x(), lidarPoint.position().y()});
    if(idx.row < GRID_MAP_SIZE_ROWS && idx.col < GRID_MAP_SIZE_COLS && idx.row >= 0 && idx.col >= 0 && !m_gridMap.at(idx.row, idx.col).locked)
    {
      Grid& grid     = m_gridMap.at(idx.row, idx.col);
      grid.point.x() = lidarPoint.position().x();
      grid.point.y() = lidarPoint.position().y();
      grid.top       = lidarPoint.top();
      grid.bottom    = lidarPoint.bottom();
      grid.locked    = true;

      if(uto::proto::PerceptionGrid_Label_UNKNOWN == lidarPoint.label())
      {
        grid.logit = std::clamp(grid.logit + LOGIT_NOISE_LIDAR - PRIOR_LOGIT, MIN_LOGIT, MAX_LOGIT);
      }
      else
      {
        grid.logit = std::clamp(grid.logit + LOGIT_OBSTACLE_LIDAR - PRIOR_LOGIT, MIN_LOGIT, MAX_LOGIT);
      }

#if FS_CHECK(CFG_DEBUG_GRID)
      grid.history += 1;
      grid.logitLidar = grid.logit;
#endif

#if FS_CHECK(CFG_USE_CF_FS)
      if(nullptr != fsCfPtr)
      {
        fuseCameraFreespace(grid, *fsCfPtr);
      }
#endif

#if FS_CHECK(CFG_USE_FISHEYE_FS)
      if(lidarPoint.label() == uto::proto::PerceptionGrid_Label_UNKNOWN)
      {
        if(nullptr != fsFisheyeFcfPtr)
        {
          fuseFisheyeFreespace(grid, *fsFisheyeFcfPtr, SensorId::FISHEYE_FCF, fsPointsCylinderFcf);
        }
        if(nullptr != fsFisheyeFllPtr)
        {
          fuseFisheyeFreespace(grid, *fsFisheyeFllPtr, SensorId::FISHEYE_FLL, fsPointsCylinderFll);
        }
        if(nullptr != fsFisheyeFrrPtr)
        {
          fuseFisheyeFreespace(grid, *fsFisheyeFrrPtr, SensorId::FISHEYE_FRR, fsPointsCylinderFrr);
        }
        if(nullptr != fsFisheyeBcbPtr)
        {
          fuseFisheyeFreespace(grid, *fsFisheyeBcbPtr, SensorId::FISHEYE_BCB, fsPointsCylinderBcb);
        }
        if(nullptr != fsFisheyeBllPtr)
        {
          fuseFisheyeFreespace(grid, *fsFisheyeBllPtr, SensorId::FISHEYE_BLL, fsPointsCylinderBll);
        }
        if(nullptr != fsFisheyeBrrPtr)
        {
          fuseFisheyeFreespace(grid, *fsFisheyeBrrPtr, SensorId::FISHEYE_BRR, fsPointsCylinderBrr);
        }
      }
#endif

      // remove gate fs points
      for(const auto& gateRect : gateRects)
      {
        if(lidarPoint.position().x() > gateRect.minX && lidarPoint.position().x() < gateRect.maxX &&
           lidarPoint.position().y() > gateRect.minY && lidarPoint.position().y() < gateRect.maxY)
        {
          grid.logit = PRIOR_LOGIT;
        }
      }
    }
  }

  assignObjectProperty();
  m_gridMap.extractBorders();
  processTunnelGeoFence();

#if FS_CHECK(CFG_USE_PASS_THROUGH_REGION)
  processPassThroughRegion(fsLidar);
#endif

  processWeighbridgeGeofence(fsLidar);
  processDrivingBoundary(fsLidar);

  const int duration = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
  UWARN << "================= process time: " << duration << " ms =================";

  UWARN_IF(duration > 50) << "================= process time exceeded limit: " << duration << " ms =================";

#if FS_CHECK(CFG_VIS_ENABLE)
  Vis::drawLidarPointImage(fsLidar);
  Vis::drawGates(gateRects);
  Vis::drawGeoFence();
  Vis::drawDrivingBoundary(fsLidar);

  if(nullptr != fsCfPtr)
  {
    Vis::drawCameraFs(*fsCfPtr);
  }

#if FS_CHECK(CFG_USE_FISHEYE_FS)
  Vis::drawLidarPointFisheye(fsLidar, SensorId::FISHEYE_FCF);
  Vis::drawLidarPointFisheye(fsLidar, SensorId::FISHEYE_FLL);
  Vis::drawLidarPointFisheye(fsLidar, SensorId::FISHEYE_FRR);
  Vis::drawLidarPointFisheye(fsLidar, SensorId::FISHEYE_BCB);
  Vis::drawLidarPointFisheye(fsLidar, SensorId::FISHEYE_BLL);
  Vis::drawLidarPointFisheye(fsLidar, SensorId::FISHEYE_BRR);

  if(nullptr != fsFisheyeFcfPtr)
  {
    Vis::drawFisheyeFs(*fsFisheyeFcfPtr, SensorId::FISHEYE_FCF);
  }
  if(nullptr != fsFisheyeFllPtr)
  {
    Vis::drawFisheyeFs(*fsFisheyeFllPtr, SensorId::FISHEYE_FLL);
  }
  if(nullptr != fsFisheyeFrrPtr)
  {
    Vis::drawFisheyeFs(*fsFisheyeFrrPtr, SensorId::FISHEYE_FRR);
  }
  if(nullptr != fsFisheyeBcbPtr)
  {
    Vis::drawFisheyeFs(*fsFisheyeBcbPtr, SensorId::FISHEYE_BCB);
  }
  if(nullptr != fsFisheyeBllPtr)
  {
    Vis::drawFisheyeFs(*fsFisheyeBllPtr, SensorId::FISHEYE_BLL);
  }
  if(nullptr != fsFisheyeBrrPtr)
  {
    Vis::drawFisheyeFs(*fsFisheyeBrrPtr, SensorId::FISHEYE_BRR);
  }
#endif

  for(const auto& obj : m_objectList)
  {
    Vis::drawObject(obj);
  }
#endif
}

void fs::FreespaceManager::backupMode(const int64_t timestampNs, const uto::proto::PerceptionFreespace& lidarFs)
{
  auto start = std::chrono::steady_clock::now();

  UINFO << "start backup mode, timestamp: " << timestampNs << " ns";

  m_passThroughPoints.clear();
  m_passThroughPoints.reserve(lidarFs.perception_gridmap_size());
  for(const auto& lidarPoint : lidarFs.perception_gridmap())
  {
    if(lidarPoint.position().x() > -PUB_RANGE_REAR &&
       lidarPoint.position().x() < PUB_RANGE_FRONT &&
       lidarPoint.position().y() > -PUB_RANGE_LEFT_RIGHT &&
       lidarPoint.position().y() < PUB_RANGE_LEFT_RIGHT)
    {
      m_passThroughPoints.emplace_back(lidarPoint.position().x(), lidarPoint.position().y());
    }
  }

  const int duration = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
  UINFO << "================= backup mode process time: " << duration << " ms =================";

  UWARN_IF(duration > 50) << "================= backup mode process time exceeded limit: " << duration << " ms =================";
}

void fs::FreespaceManager::fillObjectList(const uto::proto::PerceptionObstacles& fusedObjects, const int64_t timestampNs)
{
  const float deltaTime = 1e-9f * (timestampNs - fusedObjects.header().time_meas());

  const Vehicle& vehicle            = Vehicle::getVehicle();
  const auto [rotTitoT0, vecTitoT0] = vehicle.calcRotAndVecTitoT0(fusedObjects.header().time_meas() / 1000);

  for(const auto& fusedObj : fusedObjects.perception_obstacles())
  {
    assert(!fusedObj.sub_obstacles().empty());
    if(std::abs(fusedObj.sub_obstacles(0).position().x()) - 0.5f * fusedObj.sub_obstacles(0).length() < OD_RELEVANT_RANGE)
    {
      m_objectList.emplace_back(fusedObj,
                                deltaTime,
                                rotTitoT0.topLeftCorner<2, 2>(),
                                vecTitoT0.head<2>());
    }
  }
}

void fs::FreespaceManager::assignObjectProperty()
{
  // TODO: tangxu: raise CRANE logit
  for(const auto& obj : m_objectList)
  {
    const auto [topLeftPoint, bottomRightPoint] = obj.calcMinBoundingRect();
    const GridMap::Index topLeftIdx             = GridMap::pos2idx(topLeftPoint);
    const GridMap::Index bottomRightIdx         = GridMap::pos2idx(bottomRightPoint);
    const ObstacleClass  objClass               = obj.getClass();
    for(int j = std::max(0, topLeftIdx.row); j < std::min(bottomRightIdx.row, GRID_MAP_SIZE_ROWS); ++j)
    {
      for(int i = std::max(0, topLeftIdx.col); i < std::min(bottomRightIdx.col, GRID_MAP_SIZE_COLS); ++i)
      {
        auto& grid = m_gridMap.at(j, i);
        if(obj.isInsidePoint(grid.point))
        {
          switch(objClass)
          {
          case ObstacleClass::VRU:
            grid.logit = MIN_LOGIT;
            break;
          case ObstacleClass::VEHICLE:
          case ObstacleClass::CRANE:
            grid.logit = PRIOR_LOGIT;
            break;
          case ObstacleClass::STATIC_OBJECT:
            // do nothing
            break;
          default:
            assert(false && "unknown class");
          }
        }
      }
    }
  }
}

void fs::FreespaceManager::fuseCameraFreespace(Grid& grid, const uto::proto::CameraFreespace& fsCamera) const
{
  if(grid.point.x() > m_camOriginEgo && grid.logit > PRIOR_LOGIT)
  {
    const Camera&         camera   = CameraManager::getCamera(SensorId::CAMERA_CENTER_FRONT);
    const Eigen::Vector2i pointImg = camera.transformEgo2Img(EVector3{grid.point.x(), grid.point.y(), 0.f});
    if(pointImg.x() > 0 && pointImg.x() < fsCamera.image_width() && pointImg.y() > 0)
    {
      assert(static_cast<int>(pointImg.x() * m_camFsScale) < fsCamera.camera_freespace_points_size());
      const auto&       cameraPoint = fsCamera.camera_freespace_points(static_cast<int>(pointImg.x() * m_camFsScale));
      const CameraClass cameraClass = cameraFreespacePointType2CameraClass(cameraPoint.type());

      switch(cameraClass)
      {
      case INVALID:
        // do nothing
        break;
      case STATIC_BORDER:
      case VPR:
        if(pointImg.y() > static_cast<int>(cameraPoint.point().y()))
        {
          // it is a noise
          grid.logit       = std::clamp(grid.logit + LOGIT_FREE_CAMERA, MIN_LOGIT, MAX_LOGIT);
          grid.isFreeInCam = true;
        }
        break;
      case NON_OBSTACLE:
        grid.logit       = std::clamp(grid.logit + LOGIT_FREE_CAMERA, MIN_LOGIT, MAX_LOGIT);
        grid.isFreeInCam = true;
        break;
      default:
        assert(false && "unknown class");
      }
    }
  }
}

#if FS_CHECK(CFG_USE_FISHEYE_FS)
void fs::FreespaceManager::fuseFisheyeFreespace(Grid& grid, const uto::proto::CameraFreespace& fisheyeFreespace, const fs::SensorId sensorId, const std::vector<EVector2>& fsPointsCylinder) const
{
  const Fisheye& fisheye = CameraManager::getFisheye(sensorId);
  const EVector3 gridCam = fisheye.transformEgo2Cam(EVector3{grid.point.x(), grid.point.y(), 0.f});

  if(gridCam.z() > 0.1f && !grid.isFreeInCam)
  {
    const EVector2 gridCylinder = fisheye.transformCam2Cylinder(gridCam);
    const int      cameraFsIdx  = m_fisheyeFs2CylinderMap[static_cast<int>(gridCylinder.x())];
    if(-1 != cameraFsIdx)
    {
      const auto&       cameraPoint = fisheyeFreespace.camera_freespace_points(cameraFsIdx);
      const CameraClass cameraClass = cameraFreespacePointType2CameraClass(cameraPoint.type());

      switch(cameraClass)
      {
      case INVALID:
        // do nothing
        break;
      case STATIC_BORDER:
      case VPR:
        if(gridCylinder.y() > fsPointsCylinder[static_cast<int>(gridCylinder.x())].y())
        {
          // it is a noise
          grid.logit       = std::clamp(grid.logit + LOGIT_FREE_CAMERA, MIN_LOGIT, MAX_LOGIT);
          grid.isFreeInCam = true;
        }
        break;
      case NON_OBSTACLE:
        grid.logit       = std::clamp(grid.logit + LOGIT_FREE_CAMERA, MIN_LOGIT, MAX_LOGIT);
        grid.isFreeInCam = true;
        break;
      default:
        assert(false && "unknown class");
      }
    }
  }
}

void fs::FreespaceManager::setupFisheye2CylinderMap(const uto::proto::CameraFreespace& fisheyeFs, const fs::SensorId sensorId)
{
  const std::vector<EVector2> pointCylinderVec = transformFisheyeFs2Cylinder(fisheyeFs, sensorId);
  assert(pointCylinderVec.size() == fisheyeFs.camera_freespace_points_size());

  for(int i = FISHEYE_INVALID_MARGIN; i < pointCylinderVec.size() - FISHEYE_INVALID_MARGIN; ++i)
  {
    const auto& pointCylinder = pointCylinderVec[i];
    assert(pointCylinder.x() >= 0 && pointCylinder.x() < m_fisheyeFs2CylinderMap.max_size());
    m_fisheyeFs2CylinderMap[static_cast<int>(pointCylinder.x())] = i;
  }

  // fill the holes
  for(int i = static_cast<int>(pointCylinderVec[FISHEYE_INVALID_MARGIN].x()) + 1; i < static_cast<int>(pointCylinderVec[pointCylinderVec.size() - FISHEYE_INVALID_MARGIN].x()); ++i)
  {
    if(-1 == m_fisheyeFs2CylinderMap[i])
    {
      m_fisheyeFs2CylinderMap[i] = m_fisheyeFs2CylinderMap[i - 1];
    }
  }
}

std::vector<fs::EVector2> fs::FreespaceManager::transformFisheyeFs2Cylinder(const uto::proto::CameraFreespace& fisheyeFs, const SensorId sensorId)
{
  std::vector<cv::Point2f> pointFisheyeVec;
  pointFisheyeVec.reserve(fisheyeFs.camera_freespace_points_size());
  for(const auto& fsPoint : fisheyeFs.camera_freespace_points())
  {
    pointFisheyeVec.emplace_back(fsPoint.point().x(), fsPoint.point().y());
  }

  const Fisheye& fisheye = CameraManager::getFisheye(sensorId);
  return fisheye.transformFisheye2Cylinder(pointFisheyeVec);
}
#endif

std::vector<fs::Rect> fs::FreespaceManager::fillGateList(const uto::proto::PerceptionGates* gatePtr)
{
  std::vector<Rect> gateRectVec{};

  if(gatePtr != nullptr)
  {
    const Vehicle& vehicle = Vehicle::getVehicle();

    constexpr float maxGridX = EGO_INDEX_ROW * GRID_SCALE;
    constexpr float minGridX = (EGO_INDEX_ROW - GRID_MAP_SIZE_ROWS) * GRID_SCALE;
    constexpr float maxGridY = EGO_INDEX_COL * GRID_SCALE;
    constexpr float minGridY = (EGO_INDEX_COL - GRID_MAP_SIZE_COLS) * GRID_SCALE;

    for(const auto& gate : gatePtr->gates())
    {
      if(gate.gate_status() != uto::proto::GatesInfo_GateStatus::GatesInfo_GateStatus_OPEN)
      {
        const EVector3 gateCenter = vehicle.transformTitoT0(EVector3{gate.position().x(), gate.position().y(), 0.f}, gatePtr->header().time_meas() / 1000);

        const float maxY = gateCenter.y() + GATES_LEN * 2.5f;
        const float minY = gateCenter.y() - GATES_LEN * 2.0f;
        const float maxX = gateCenter.x() + GATES_RANGE;
        const float minX = gateCenter.x() - GATES_RANGE;

        if(minX >= maxGridX || maxX < minGridX || minY >= maxGridY || maxY < minGridY)
        {
          // -- 不在栅格内
        }
        else
        {
          Rect& rect = gateRectVec.emplace_back();
          rect.minX  = std::max(minX, minGridX);
          rect.maxX  = std::min(maxX, maxGridX);
          rect.minY  = std::max(minY, minGridY);
          rect.maxY  = std::min(maxY, maxGridY);
        }
      }
    }
  }

  return gateRectVec;
}

void fs::FreespaceManager::processPassThroughRegion(const uto::proto::PerceptionFreespace& lidarFs)
{
  for(const auto& lidarPoint : lidarFs.perception_gridmap())
  {
    if(lidarPoint.position().y() > REGION_PASS_THROUGH_LEFT &&
       lidarPoint.position().y() < REGION_PASS_THROUGH_RIGHT &&
       lidarPoint.label() != uto::proto::PerceptionGrid_Label_UNKNOWN)
    {
      bool isInOd = false;

      for(const auto& obj : m_objectList)
      {
        const auto [boxMaxXY, boxMinXY] = obj.calcMinBoundingRect();
        if(boxMaxXY.y() > REGION_PASS_THROUGH_LEFT &&
           boxMinXY.y() < REGION_PASS_THROUGH_RIGHT &&
           lidarPoint.position().x() > boxMinXY.x() &&
           lidarPoint.position().x() < boxMaxXY.x() &&
           lidarPoint.position().y() > boxMinXY.y() &&
           lidarPoint.position().y() < boxMaxXY.y() &&
           obj.isInsidePoint(EVector2{lidarPoint.position().x(), lidarPoint.position().y()}))
        {
          isInOd = true;
          break;
        }
      }

      if(!isInOd)
      {
        m_passThroughPoints.emplace_back(lidarPoint.position().x(), lidarPoint.position().y());
      }
    }
  }
}

void fs::FreespaceManager::processWeighbridgeGeofence(const uto::proto::PerceptionFreespace& lidarFs)
{
  const Vehicle& vehicle = Vehicle::getVehicle();
  if(vehicle.isInGeofence(MapProvider::getMapProvider().getWeighbridgeAreaLtm()))
  {
    m_gridMap.setNoPublish();
#if FS_CHECK(CFG_USE_WEIGHBRIDGE)
    for(const auto& lidarPoint : lidarFs.perception_gridmap())
    {
      if(lidarPoint.label() == uto::proto::PerceptionGrid_Label_ROAD_EDGE)
      {
        m_passThroughPoints.emplace_back(lidarPoint.position().x(), lidarPoint.position().y());
      }
    }
#endif
  }
}

void fs::FreespaceManager::processDrivingBoundary(const uto::proto::PerceptionFreespace& fsLidar)
{
  if(fsLidar.drivable_boundary().xmax() > 0)
  {
    const GridMap::Index topLeftIdx     = GridMap::pos2idx(EVector2{fsLidar.drivable_boundary().xmax(), fsLidar.drivable_boundary().ymax()});
    const GridMap::Index bottomRightIdx = GridMap::pos2idx(EVector2{fsLidar.drivable_boundary().xmin(), fsLidar.drivable_boundary().ymin()});
    for(int j = std::max(0, topLeftIdx.row); j < std::min(bottomRightIdx.row, GRID_MAP_SIZE_ROWS); ++j)
    {
      for(int i = std::max(0, topLeftIdx.col); i < std::min(bottomRightIdx.col, GRID_MAP_SIZE_COLS); ++i)
      {
        Grid& grid = m_gridMap.at(j, i);
        if(grid.logit > PRIOR_LOGIT)
        {
          grid.logit = LOGIT_OCCUPIED_THRESH - LOGIT_OBSTACLE_LIDAR + 1;
        }
      }
    }
  }
}

void fs::FreespaceManager::processTunnelGeoFence()
{
  const Vehicle& vehicle = Vehicle::getVehicle();
  if(vehicle.isInGeofence(MapProvider::getMapProvider().getTunnelAreaLtm()))
  {
    m_gridMap.setNoPublish();
    for(int row = 0; row < GRID_MAP_SIZE_ROWS; row++)
    {
      for(int col = 0; col < GRID_MAP_SIZE_COLS; col++)
      {
        Grid& grid = m_gridMap.at(row, col);
        if(grid.isBorder && grid.bottom - grid.ground < vehicle.getVehicleHeight() + 0.5)
        {
          m_passThroughPoints.emplace_back(grid.point.x(), grid.point.y());
        }
      }
    }
  }
}

void fs::FreespaceManager::refineRoadModel(const uto::proto::RoadModel* roadModelPtr, const uto::proto::PerceptionFreespace& fsLidar)
{
  if(roadModelPtr == nullptr)
  {
    return;
  }

  // 解析roadmodel 存为MAP映射, key为roadmodel点位置(整型), value是对应的高度值, 用于后续快速查找
  std::map<std::pair<int, int>, float> RoadModelPointMap;
  for(int i = 0; i < roadModelPtr->grids().size();)
  {
    int RoadmodelGridPosX = EGO_INDEX_ROW - (roadModelPtr->grids()[i] - roadModelPtr->ego_x_index()) * roadModelPtr->grid_resolution(); // 地面模型点在自车移动栅格中的位置
    int RoadmodelGridPosy = EGO_INDEX_COL - (-roadModelPtr->grids()[i + 1] + roadModelPtr->ego_y_index()) * roadModelPtr->grid_resolution();

    RoadModelPointMap[std::pair<int, int>(RoadmodelGridPosX, RoadmodelGridPosy)] = roadModelPtr->grids()[i + 2] * roadModelPtr->height_resolution();
    i += 3;
  }

  // 查找当前栅格位置8个点(将栅格位置强行转为int)对应的roadmodel点,储存对应的高度值
  std::vector<std::pair<int, int>> res;

  auto searchInNeighbor = [&RoadModelPointMap, &res](const std::pair<int, int>& GridPos) {
    for(int i = -1; i <= 1; ++i)
    {
      for(int j = -1; j <= 1; ++j)
      {
        if(RoadModelPointMap.find(std::pair<int, int>(GridPos.first + i, GridPos.second + j)) != RoadModelPointMap.end())
        {
          res.emplace_back(GridPos.first + i, GridPos.second + j);
        }
      }
    }
    return res.empty();
  };

  // 遍历要发出去的栅格, 完成每个栅格的ground赋值
  for(int row = PUB_MIN_ROW; row < PUB_MAX_ROW; row++)
  {
    for(int col = PUB_MIN_COL; col < PUB_MAX_COL; col++)
    {
      Grid& grid = m_gridMap.at(row, col);
      if(grid.isBorder && grid.logit > LOGIT_OCCUPIED_THRESH)
      {
        const EVector2 gridposVec = GridMap::idx2pos(row, col);
        res.clear();

        // 得到当前栅格的邻居结果
        searchInNeighbor(std::pair<int, int>(gridposVec.x(), gridposVec.y()));

        // 如果找到的有对应的地面点
        if(!res.empty())
        {
          float sum   = std::accumulate(res.begin(), res.end(), 0.0f, [&RoadModelPointMap](float acc, const std::pair<int, int>& gridpos) {
            return acc + RoadModelPointMap[gridpos];
          });
          grid.ground = sum / static_cast<float>(res.size());
        }
        else
        {
          grid.ground = std::min(0.0f, grid.bottom);
        }
      }
    }
  }
  return;
}
