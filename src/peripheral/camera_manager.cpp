#include "camera_manager.h"
#include "defs.h"
#include "utils.h"

fs::CameraManager::CameraManager()
{
  m_cameraList[static_cast<std::size_t>(SensorId::CAMERA_CENTER_FRONT)].setSensorId(SensorId::CAMERA_CENTER_FRONT);
#if FS_CHECK(CFG_USE_FISHEYE_FS)
  m_fisheyeList[static_cast<std::size_t>(SensorId::FISHEYE_FCF) - static_cast<std::size_t>(SensorId::FISHEYE_FCF)].setSensorId(SensorId::FISHEYE_FCF);
  m_fisheyeList[static_cast<std::size_t>(SensorId::FISHEYE_FLL) - static_cast<std::size_t>(SensorId::FISHEYE_FCF)].setSensorId(SensorId::FISHEYE_FLL);
  m_fisheyeList[static_cast<std::size_t>(SensorId::FISHEYE_FRR) - static_cast<std::size_t>(SensorId::FISHEYE_FCF)].setSensorId(SensorId::FISHEYE_FRR);
  m_fisheyeList[static_cast<std::size_t>(SensorId::FISHEYE_BCB) - static_cast<std::size_t>(SensorId::FISHEYE_FCF)].setSensorId(SensorId::FISHEYE_BCB);
  m_fisheyeList[static_cast<std::size_t>(SensorId::FISHEYE_BLL) - static_cast<std::size_t>(SensorId::FISHEYE_FCF)].setSensorId(SensorId::FISHEYE_BLL);
  m_fisheyeList[static_cast<std::size_t>(SensorId::FISHEYE_BRR) - static_cast<std::size_t>(SensorId::FISHEYE_FCF)].setSensorId(SensorId::FISHEYE_BRR);
#endif
}

fs::CameraManager& fs::CameraManager::instance()
{
  static CameraManager instance;
  return instance;
}

bool fs::CameraManager::isAllCameraValid() const
{
  bool ret = true;

#if FS_CHECK(CFG_USE_CF_FS)
  for(const auto& item : m_cameraList)
  {
    ret = ret && item.isValid();
  }
#endif

#if FS_CHECK(CFG_USE_FISHEYE_FS)
  for(const auto& item : m_fisheyeList)
  {
    ret = ret && item.isValid();
  }
#endif

  return ret;
}

void fs::CameraManager::updateCalibInfo(const uto::proto::CameraCalib& calib, const fs::SensorId sensorId)
{
  if(!Utils::isFisheye(sensorId) && !m_cameraList[static_cast<int>(sensorId)].isValid())
  {
    m_cameraList[static_cast<int>(sensorId)].updateCalibInfo(calib);
  }

#if FS_CHECK(CFG_USE_FISHEYE_FS)
  if(Utils::isFisheye(sensorId) && !m_fisheyeList[static_cast<int>(sensorId) - static_cast<int>(SensorId::FISHEYE_FCF)].isValid())
  {
    m_fisheyeList[static_cast<int>(sensorId) - static_cast<int>(SensorId::FISHEYE_FCF)].updateCalibInfo(calib);
  }
#endif
}

void fs::CameraManager::updateConfigInfo(const uto::proto::SensorTable& sensorTable)
{
  const auto& camConfig = sensorTable.camera_config();

#if FS_CHECK(CFG_USE_CF_FS)
  for(auto& camera : m_cameraList)
  {
    if(!camera.isValid())
    {
      const SensorId sensorId = camera.getSensorId();
      const auto     iter     = std::find_if(camConfig.cbegin(), camConfig.cend(), [sensorId](const auto& camConfig) { return camConfig.id() == SENSOR_ID_TO_CAMERA_ID.at(sensorId); });
      assert(iter != camConfig.cend());
      camera.updateConfigInfo(*iter);
    }
  }
#endif

#if FS_CHECK(CFG_USE_FISHEYE_FS)
  for(auto& fisheye : m_fisheyeList)
  {
    if(!fisheye.isValid())
    {
      const SensorId sensorId = fisheye.getSensorId();
      const auto     iter     = std::find_if(camConfig.cbegin(), camConfig.cend(), [sensorId](const auto& camConfig) { return camConfig.id() == SENSOR_ID_TO_CAMERA_ID.at(sensorId); });
      assert(iter != camConfig.cend());
      fisheye.updateConfigInfo(*iter);
    }
  }
#endif
}
