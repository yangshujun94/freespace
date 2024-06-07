#include <opencv2/calib3d.hpp>
#include <common/math/rotation_conversion.h>
#include "fisheye.h"
#include "defs.h"
#include "debug/fs_log.h"

void fs::Fisheye::updateConfigInfo(const uto::proto::SensorTable::CameraConfig& config)
{
  m_isConfigValid = true;
  m_imageWidth    = config.image_size().width();
  m_imageHeight   = config.image_size().height();
  LOG_DEBUG("[image size][sensor: %-5s][width: %d][height: %d]", SENSOR_ID_TO_STRING.at(m_sensorId).c_str(), m_imageWidth, m_imageHeight);
}

void fs::Fisheye::updateCalibInfo(const uto::proto::CameraCalib& calib)
{
  assert(calib.distortion_model() == uto::proto::CameraCalib_DistortionModel_EQUIDISTANT);

  m_originalIntrinsics     = (cv::Mat_<float>(3, 3) << calib.intrinsic().fx(), 0.f, calib.intrinsic().cx(), 0.f, calib.intrinsic().fy(), calib.intrinsic().cy(), 0.f, 0.f, 1.f);
  m_distortionCoefficients = (cv::Mat_<float>(4, 1) << calib.fisheye_coeffs().k1(), calib.fisheye_coeffs().k2(), calib.fisheye_coeffs().k3(), calib.fisheye_coeffs().k4());

  const cv::Mat  E = cv::Mat::eye(3, 3, cv::DataType<float>::type);
  const cv::Size imageSize{FISHEYE_IMAGE_WIDTH, FISHEYE_IMAGE_HEIGHT};

  m_newIntrinsics = m_originalIntrinsics;
  cv::fisheye::initUndistortRectifyMap(m_originalIntrinsics, m_distortionCoefficients, E, m_newIntrinsics, imageSize, CV_16SC2, m_map1, m_map2);

  m_fx = m_newIntrinsics.at<float>(0, 0);
  m_fy = m_newIntrinsics.at<float>(1, 1);
  m_cx = m_newIntrinsics.at<float>(0, 2);
  m_cy = m_newIntrinsics.at<float>(1, 2);

  m_rotCam2Ego = uto::rotmatFromEuler<float, float>(uto::degToRad(calib.rotation_rpy_deg().x()), uto::degToRad(calib.rotation_rpy_deg().y()), uto::degToRad(calib.rotation_rpy_deg().z()));
  m_vecCam2Ego << calib.translation().x(), calib.translation().y(), calib.translation().z();

  updateRotCylinder2Cam(calib);

  if(m_isConfigValid)
  {
    updateCylinderProjectionMap();
    m_isCalibValid = true;
  }
}

void fs::Fisheye::updateRotCylinder2Cam(const uto::proto::CameraCalib& calib)
{
  const float roll  = uto::degToRad(calib.rotation_rpy_deg().x());
  const float pitch = uto::degToRad(calib.rotation_rpy_deg().y());

  float yaw = uto::degToRad(calib.rotation_rpy_deg().z());
  if(yaw > M_PI_4f32 && yaw < 3.f * M_PI_4f32)
  {
    yaw = M_PI_2f32 - yaw;
  }
  else if(yaw > 3.f * M_PI_4f32 && yaw <= M_PIf32)
  {
    yaw = M_PIf32 - yaw;
  }
  else if(yaw > -3.f * M_PI_4f32 && yaw < -M_PI_4f32)
  {
    yaw = -M_PI_2f32 - yaw;
  }
  else if(yaw >= -M_PIf32 && yaw < -3.f * M_PI_4f32)
  {
    yaw = -M_PIf32 - yaw;
  }
  else
  {
    yaw = 0 - yaw;
  }

  m_rotCylinder2Cam = uto::rotmatFromEuler<float, float>(-roll - M_PI_2f32, -pitch, -yaw);
}

void fs::Fisheye::updateCylinderProjectionMap()
{
  m_map1.create(m_imageHeight, m_imageWidth, CV_32F);
  m_map2.create(m_imageHeight, m_imageWidth, CV_32F);
  const float resolutionWidth  = M_PIf32 / m_imageWidth;
  const float resolutionHeight = (M_PIf32 - FISHEYE_IMAGE_MARGIN) / m_imageHeight;

  for(int j = 0; j < m_imageHeight; ++j)
  {
    for(int i = 0; i < m_imageWidth; ++i)
    {
      const float theta = i * resolutionWidth;
      const float phi   = j * resolutionHeight + FISHEYE_IMAGE_MARGIN;

      const EVector3 cylinderPoint{-std::sin(phi) * std::cos(theta), -std::cos(phi), std::sin(phi) * std::sin(theta)};
      const EVector3 fisheyePoint = m_rotCylinder2Cam * cylinderPoint;

      const std::vector<cv::Point3f> objectPoints{cv::Point3f(fisheyePoint.x(), fisheyePoint.y(), fisheyePoint.z())};
      std::vector<cv::Point2f>       imagePoints;
      cv::fisheye::projectPoints(objectPoints, imagePoints, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), m_originalIntrinsics, m_distortionCoefficients);

      const float x = imagePoints[0].x;
      const float y = imagePoints[0].y;
      if(x >= 0 && x < m_imageWidth && y >= 0 && y < m_imageHeight && objectPoints[0].z > 0.f)
      {
        m_map1.at<float>(j, i) = x;
        m_map2.at<float>(j, i) = y;
      }
      else
      {
        m_map1.at<float>(j, i) = -1;
        m_map2.at<float>(j, i) = -1;
      }
    }
  }
}

fs::EVector2 fs::Fisheye::ray2Cylinder(const EVector3& ray) const
{
  const float sqInv = 1.f / ray.norm();
  const float phi   = std::acos(-ray.y() * sqInv);
  const float theta = std::acos(std::clamp(-ray.x() * sqInv / std::sin(phi), -1.f, 1.f));
  const float u     = theta * m_imageWidth / M_PIf32;
  const float v     = (phi - FISHEYE_IMAGE_MARGIN) * m_imageHeight / (M_PIf32 - FISHEYE_IMAGE_MARGIN);

  return {u, v};
}

std::vector<fs::EVector2> fs::Fisheye::transformFisheye2Cylinder(const std::vector<cv::Point2f>& distortedPoints) const
{
  std::vector<cv::Point2f> undistortedPoints;
  cv::fisheye::undistortPoints(distortedPoints, undistortedPoints, m_originalIntrinsics, m_distortionCoefficients);

  std::vector<EVector2> output;
  output.reserve(undistortedPoints.size());
  for(const auto& undistortedPoint : undistortedPoints)
  {
    output.emplace_back(transformCam2Cylinder(EVector3{undistortedPoint.x, undistortedPoint.y, 1.f}));
  }

  return output;
}
