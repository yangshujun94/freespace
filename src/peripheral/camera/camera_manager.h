#ifndef CAMERA_MANAGER_H_
#define CAMERA_MANAGER_H_

#include "peripheral/types.h"
#include "camera.h"
#include "fisheye.h"
#include "pinhole.h"
#include "peripheral/macro.h"
#include "peripheral/switcher.h"

namespace fs
{
  class CameraManager
  {
  public:
    DISALLOW_COPY_AND_ASSIGN(CameraManager);

    static CameraManager& instance();

    [[nodiscard]] bool isAllCameraValid() const;

#if FS_CHECK(CFG_ROS2)
    void updateCalibInfo(const uto::proto::CameraCalib& calib, SensorId sensor_id);
    void updateConfigInfo(const uto::proto::SensorTable& sensorTable);
#else
    void readCalibInfo(const int vehicle_id);
#endif

    static const std::shared_ptr<Camera> getCamera(const SensorId sensor_id)
    {
      return instance().camera_list_[static_cast<int>(sensor_id)] ? instance().camera_list_[static_cast<int>(sensor_id)] : nullptr;
      //      int sensorIndex = static_cast<int>(sensor_id);
      //      std::shared_ptr<Camera> cameraPtr = instance().camera_list_[sensorIndex];
      //
      //      if (!cameraPtr)
      //      {
      //        UERROR << "SensorId " << sensorIndex << " is not initialized." ;
      //      }
      //
      //      return cameraPtr;
    }

  private:
    CameraManager();

    std::array<std::shared_ptr<Camera>, static_cast<int>(SensorId::MAX_SENSOR_NUM) + 1> camera_list_{};
    const std::map<const SensorId, const bool>                                          sensor_used_ = SENSORS_USED;
  };
} // namespace fs

#endif //CAMERA_MANAGER_H_
