#include <common/math/rotation_conversion.h>
#include "camera.h"

void fs::Camera::updateCalibInfo(const uto::proto::CameraCalib& calib)
{
  m_fx = calib.intrinsic().fx();
  m_fy = calib.intrinsic().fy();
  m_cx = calib.intrinsic().cx();
  m_cy = calib.intrinsic().cy();

  m_rotCam2Ego = uto::rotmatFromEuler<float, float>(uto::degToRad(calib.rotation_rpy_deg().x()), uto::degToRad(calib.rotation_rpy_deg().y()), uto::degToRad(calib.rotation_rpy_deg().z()));
  m_vecCam2Ego << calib.translation().x(), calib.translation().y(), calib.translation().z();

  m_isCalibValid = true;
}

void fs::Camera::updateConfigInfo(const uto::proto::SensorTable::CameraConfig& config)
{
  m_isConfigValid = true;
  m_imageWidth    = config.image_size().width();
  m_imageHeight   = config.image_size().height();
}
