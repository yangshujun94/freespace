#ifndef FISHEYE_H_
#define FISHEYE_H_

#include <camera_calib.pb.h>
#include <opencv2/core/core.hpp>

#include "types.h"
#include "macro.h"

namespace fs
{
  class Fisheye
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(Fisheye);

    Fisheye() = default;

    void updateCalibInfo(const uto::proto::CameraCalib& calib);
    void updateConfigInfo(const uto::proto::SensorTable::CameraConfig& config);
    void setSensorId(const SensorId sensorId) { m_sensorId = sensorId; }

    EVector3 transformCam2Ego(const EVector3& pointCam) const { return m_rotCam2Ego * pointCam + m_vecCam2Ego; }
    EVector3 transformEgo2Cam(const EVector3& pointEgo) const { return m_rotCam2Ego.transpose() * (pointEgo - m_vecCam2Ego); }
    EVector2 transformCam2Cylinder(const fs::EVector3& pointCam) const { return ray2Cylinder(m_rotCylinder2Cam.transpose() * pointCam); }

    std::vector<EVector2> transformFisheye2Cylinder(const std::vector<cv::Point2f>& distortedPoints) const;

    bool     isValid() const { return m_isCalibValid && m_isConfigValid; }
    int      getImageWidth() const { return m_imageWidth; }
    int      getImageHeight() const { return m_imageHeight; }
    SensorId getSensorId() const { return m_sensorId; }

  private:
    void updateRotCylinder2Cam(const uto::proto::CameraCalib& calib);
    void updateCylinderProjectionMap();

    EVector2 ray2Cylinder(const EVector3& ray) const;

    SensorId m_sensorId = SensorId::MAX_SENSOR_NUM;

    bool  m_isCalibValid  = false;
    bool  m_isConfigValid = false;
    int   m_imageHeight   = 0;
    int   m_imageWidth    = 0;
    float m_fx            = 0.f;
    float m_fy            = 0.f;
    float m_cx            = 0.f;
    float m_cy            = 0.f;

    cv::Mat m_map1;
    cv::Mat m_map2;
    cv::Mat m_originalIntrinsics;
    cv::Mat m_newIntrinsics;
    cv::Mat m_distortionCoefficients;

    EVector3 m_vecCam2Ego;
    EMatrix3 m_rotCam2Ego;
    EMatrix3 m_rotCylinder2Cam;
  };
} // namespace fs

#endif //FISHEYE_H_
