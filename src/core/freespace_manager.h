#ifndef FREESPACE_MANAGER_H_
#define FREESPACE_MANAGER_H_

#include <perception_freespace.pb.h>
#include <perception_camera.pb.h>
#include <common/ring_buffer.h>
#include <perception_signal.pb.h>

#include "peripheral/switcher.h"
#include "grid_map.h"
#include "peripheral/types.h"
#include "object.h"

namespace fs
{
  class FreespaceManager
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(FreespaceManager);

    FreespaceManager()
    {
      m_fisheyeFs2CylinderMap.fill(-1);
    }

    void fuseFreespace(const int64_t                          timestampNs,
                       const uto::proto::PerceptionFreespace& fsLidar,
                       const uto::proto::CameraFreespace*     fsCfPtr,
                       const uto::proto::CameraFreespace*     fsFisheyeFcfPtr,
                       const uto::proto::CameraFreespace*     fsFisheyeFllPtr,
                       const uto::proto::CameraFreespace*     fsFisheyeFrrPtr,
                       const uto::proto::CameraFreespace*     fsFisheyeBcbPtr,
                       const uto::proto::CameraFreespace*     fsFisheyeBllPtr,
                       const uto::proto::CameraFreespace*     fsFisheyeBrrPtr,
                       const uto::proto::PerceptionObstacles* odPtr,
                       const uto::proto::PerceptionGates*     gatePtr);

    void backupMode(const int64_t timestampNs, const uto::proto::PerceptionFreespace& lidarFs);
    void resetGridMap() { m_gridMap.reset(); }
    void resetPassThroughPoints() { m_passThroughPoints.clear(); }

    const GridMap&               getGridMap() const { return m_gridMap; }
    const std::vector<EVector2>& getPassThroughPoints() const { return m_passThroughPoints; }

  private:
    void fillObjectList(const uto::proto::PerceptionObstacles& fusedObjects, const int64_t timestampNs);
    void assignObjectProperty();
    void fuseCameraFreespace(Grid& grid, const uto::proto::CameraFreespace& fsCamera) const;
    void fuseFisheyeFreespace(Grid& grid, const uto::proto::CameraFreespace& fisheyeFreespace, const fs::SensorId sensorId, const std::vector<EVector2>& fsPointsCylinder) const;
    void setupFisheye2CylinderMap(const uto::proto::CameraFreespace& fisheyeFs, const SensorId sensorId);
    void processDrivingBoundary(const uto::proto::PerceptionFreespace& fsLidar);

    /// @brief Enter the Weighbridge area.
    /// If you open the "CFG_USE_WEIGHBRIDGE" macro, only output points whose attributes are the RoadEdge;
    /// If you close the "CFG_USE_WEIGHBRIDGE" macro, no points will be output;
    /// @param lidarFs
    void processWeighbridgeGeofence(const uto::proto::PerceptionFreespace& lidarFs);

    /// @brief AIV3 & AIV5
    /// Output all points outside the OD box within a 2m horizontal range of the vehicle whose attributes are not "UNKNOWN".
    /// @param lidarFs
    void processPassThroughRegion(const uto::proto::PerceptionFreespace& lidarFs);

    static std::vector<fs::Rect> fillGateList(const uto::proto::PerceptionGates* gatePtr);
    static std::vector<EVector2> transformFisheyeFs2Cylinder(const uto::proto::CameraFreespace& fisheyeFs, const SensorId sensorId);

    GridMap    m_gridMap{};
    ObjectList m_objectList{};
    float      m_camOriginEgo               = 0.f;
    float      m_camFsScale                 = 0.f;
    bool       m_isFisheye2CylinderMapValid = false;

    std::vector<EVector2> m_passThroughPoints{};
    std::array<int, 1280> m_fisheyeFs2CylinderMap{};
  };
} // namespace fs

#endif //FREESPACE_MANAGER_H_
