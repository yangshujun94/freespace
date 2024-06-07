#include "camera_manager.h"

fs::CameraManager::CameraManager()
{
}

fs::CameraManager& fs::CameraManager::instance()
{
  static CameraManager instance;
  return instance;
}

bool fs::CameraManager::isAllCameraValid() const
{
  bool ret = true;
  for(const auto& item : sensor_used_)
  {
    if(item.second)
    {
      if(!camera_list_[static_cast<int>(item.first)])
      {
        ret = false;
      }
      else
      {
        if(item.second)
        {
          ret = ret && camera_list_[static_cast<int>(item.first)]->isValid();
        }
      }
    }
  }

  return ret;
}

#if FS_CHECK(CFG_ROS2)
void fs::CameraManager::updateCalibInfo(const uto::proto::CameraCalib& calib, SensorId sensor_id)
{
  if(camera_list_[static_cast<int>(sensor_id)] == nullptr)
  {
    if(calib.distortion_model() == uto::proto::CameraCalib_DistortionModel_RADTAN)
    {
      camera_list_[static_cast<int>(sensor_id)] = std::make_shared<Pinhole>();
    }
    else if(calib.distortion_model() == uto::proto::CameraCalib_DistortionModel_EQUIDISTANT)
    {
      camera_list_[static_cast<int>(sensor_id)] = std::make_shared<Fisheye>();
    }
    else
    {
      UERROR << "This camera model is not supported.";
    }
  }
  // UERROR << "sensor_used_ " << sensor_used_.size();
  // UERROR << "sensor_used_ " << (sensor_used_.find(sensor_id) == sensor_used_.end());

  if(sensor_used_.find(sensor_id) != sensor_used_.end())
  {
    if(sensor_used_.at(sensor_id) && !camera_list_[static_cast<int>(sensor_id)]->isCalibValid())
    {
      camera_list_[static_cast<int>(sensor_id)]->updateCalibInfo(calib);
    }
  }
}
void fs::CameraManager::updateConfigInfo(const uto::proto::SensorTable& sensorTable)
{
  const auto& camConfig = sensorTable.camera_config();
  // UERROR << "sensor_used_ " << sensor_used_.size();
  for(const auto& item : sensor_used_)
  {
    const auto sensor_id = item.first;
    // UERROR << "sensor_used_ " << (sensor_used_.find(sensor_id) == sensor_used_.end()) << " camera_list_ " << camera_list_.size() << " " << (SENSOR_ID_TO_STRING.find(sensor_id) == SENSOR_ID_TO_STRING.end());
    if(SENSOR_ID_TO_STRING.find(sensor_id) != SENSOR_ID_TO_STRING.end())
    {
      if(item.second && camera_list_[static_cast<int>(sensor_id)] && !camera_list_[static_cast<int>(sensor_id)]->isSizeValid())
      {
        const auto iter = std::find_if(camConfig.cbegin(), camConfig.cend(), [sensor_id](const auto& camConfig) { return camConfig.id() == SENSOR_ID_TO_CAMERA_ID.at(sensor_id); });
        if(iter != camConfig.cend())
        {
          camera_list_[static_cast<int>(sensor_id)]->updateImageSize(*iter);
          UERROR << " [" << SENSOR_ID_TO_STRING.at(sensor_id) << "]"
                 << " -> "
                 << "img height is " << iter->image_size().height() << " width is " << iter->image_size().width();
        }
        else
        {
          UERROR << SENSOR_ID_TO_STRING.at(sensor_id) << " not in sensor table";
        }
      }
    }
  }
}
#else
void fs::CameraManager::readCalibInfo(const int vehicle_id)
{
     if(!isAllCameraValid())
  {
    UINFO << "init camera ";
    for(const auto& item : sensor_used_)
    {
      if(item.second)
      {
        camera_list_[static_cast<int>(item.first)]->readCalibInfo(vehicle_id, item.first);
      }
    }
  }
}
#endif
