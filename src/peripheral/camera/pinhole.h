#ifndef CAMERAMODELS_PINHOLE_H
#define CAMERAMODELS_PINHOLE_H
#include <assert.h>
#include "camera.h"

namespace fs
{
  class Pinhole : public Camera
  {
  public:
    Pinhole()
    {
      param_.resize(4);
    }
    ~Pinhole() override{};
    Eigen::Vector2d transformCam2Img(const Eigen::Vector3d& v3D) const override;
    Eigen::Vector2f transformCam2Img(const Eigen::Vector3f& v3D) const override;
    Eigen::Vector3f transformCam2Ego(const Eigen::Vector3f& pointCam) const override;
    Eigen::Vector3f transformEgo2Cam(const Eigen::Vector3f& pointEgo) const override;
    Eigen::Vector3f unprojectEig(const cv::Point2f& p2D) const override;
    cv::Point3f     unproject(const cv::Point2f& p2D) const override;
    cv::Mat         toK() const override;
    Eigen::Matrix3f toK_() const override;
    cv::Vec4f       toD() const override;
    Eigen::Matrix3f getRotMat() const override;

#if FS_CHECK(CFG_ROS2)
    void updateIntrinsics(const uto::proto::CameraCalib& calib) override;
    void updateExtrinsics(const uto::proto::CameraCalib& calib) override;
    void updateCalibInfo(const uto::proto::CameraCalib& calib) override;
    void updateImageSize(const uto::proto::SensorTable_CameraConfig& calib) override;
#else
    void readCalibInfo(const int vehicleId, SensorId sensorId) override;
    bool updateIntrinsics(const int vehicleId, SensorId sensorId) override;
    bool updateExtrinsics(const int vehicleId, SensorId sensorId) override;
#endif
    // void calculateCameraRotation(const uto::proto::CameraCalib& calib) override;

    friend std::ostream& operator<<(std::ostream& os, const Pinhole& ph);
    friend std::istream& operator>>(std::istream& os, Pinhole& ph);
  };
} // namespace fs

#endif //CAMERAMODELS_PINHOLE_H