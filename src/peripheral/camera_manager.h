#ifndef CAMERA_MANAGER_H_
#define CAMERA_MANAGER_H_

#include "camera.h"
#include "fisheye.h"

namespace fs
{
  class CameraManager
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(CameraManager);

    static CameraManager& instance();

    bool isAllCameraValid() const;

    void updateCalibInfo(const uto::proto::CameraCalib& calib, const SensorId sensorId);
    void updateConfigInfo(const uto::proto::SensorTable& sensorTable);

    static const Camera& getCamera(const SensorId sensorId)
    {
      assert(sensorId >= SensorId::CAMERA_CENTER_FRONT && sensorId <= SensorId::CAMERA_CENTER_FRONT);
      return instance().m_cameraList[static_cast<int>(sensorId)];
    }

#if FS_CHECK(CFG_USE_FISHEYE_FS)
    static const Fisheye& getFisheye(const SensorId sensorId)
    {
      assert(sensorId >= SensorId::FISHEYE_FCF && sensorId <= SensorId::FISHEYE_BRR);
      return instance().m_fisheyeList[static_cast<int>(sensorId) - static_cast<int>(SensorId::FISHEYE_FCF)];
    }
#endif

  private:
    CameraManager();

    std::array<Camera, static_cast<int>(SensorId::CAMERA_CENTER_FRONT) + 1> m_cameraList{};

#if FS_CHECK(CFG_USE_FISHEYE_FS)
    std::array<Fisheye, static_cast<int>(SensorId::FISHEYE_BRR) - static_cast<int>(SensorId::FISHEYE_FCF) + 1> m_fisheyeList{};
#endif
  };
} // namespace fs

#endif //CAMERA_MANAGER_H_
