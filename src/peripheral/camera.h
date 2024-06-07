#ifndef CAMERA_H_
#define CAMERA_H_

#include <camera_calib.pb.h>

#include "types.h"
#include "macro.h"

namespace fs
{
  class Camera
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(Camera);

    Camera() = default;

    void updateCalibInfo(const uto::proto::CameraCalib& calib);
    void updateConfigInfo(const uto::proto::SensorTable::CameraConfig& config);
    void setSensorId(const SensorId sensorId) { m_sensorId = sensorId; }

    EVector3        transformCam2Ego(const EVector3& pointCam) const { return m_rotCam2Ego * pointCam + m_vecCam2Ego; }
    EVector3        transformEgo2Cam(const EVector3& pointEgo) const { return m_rotCam2Ego.transpose() * (pointEgo - m_vecCam2Ego); }
    Eigen::Vector2i transformCam2Img(const EVector3& pointCam) const { return Eigen::Vector2i{m_fx * pointCam[0] / pointCam[2] + m_cx, m_fy * pointCam[1] / pointCam[2] + m_cy}; }
    Eigen::Vector2i transformEgo2Img(const EVector3& pointEgo) const { return transformCam2Img(transformEgo2Cam(pointEgo)); }

    bool     isValid() const { return m_isCalibValid && m_isConfigValid; }
    int      getImageWidth() const { return m_imageWidth; }
    int      getImageHeight() const { return m_imageHeight; }
    float    getFx() const { return m_fx; }
    float    getFy() const { return m_fy; }
    float    getCx() const { return m_cx; }
    float    getCy() const { return m_cy; }
    SensorId getSensorId() const { return m_sensorId; }

  private:
    SensorId m_sensorId = SensorId::MAX_SENSOR_NUM;

    bool  m_isCalibValid  = false;
    bool  m_isConfigValid = false;
    int   m_imageHeight   = 0;
    int   m_imageWidth    = 0;
    float m_fx            = 0.0F;
    float m_fy            = 0.0F;
    float m_cx            = 0.0F;
    float m_cy            = 0.0F;

    EVector3 m_vecCam2Ego;
    EMatrix3 m_rotCam2Ego;
  };
} // namespace fs

#endif //CAMERA_H_
