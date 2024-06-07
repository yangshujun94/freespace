
#ifndef CAMERAMODELS_KANNALABRANDT8_H
#define CAMERAMODELS_KANNALABRANDT8_H

#include <assert.h>
#include "camera.h"
#define MARGIN (45 * CV_PI / 180.)

namespace fs
{
  class Fisheye : public Camera
  {
  private:
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> myHash;
    std::pair<cv::Mat, cv::Mat>                                             remap_;

  public:
    Fisheye():
      Camera()
    {
      param_.resize(8);
    }
    ~Fisheye() override{};

    Eigen::Vector2d transformCam2Img(const Eigen::Vector3d& v3D) const override;
    Eigen::Vector2f transformCam2Img(const Eigen::Vector3f& v3D) const override;
    Eigen::Vector3f transformCam2Ego(const Eigen::Vector3f& pointCam) const override;
    Eigen::Vector3f transformEgo2Cam(const Eigen::Vector3f& pointEgo) const override;
    Eigen::Vector3f unprojectEig(const cv::Point2f& p2D) const override;
    cv::Point3f     unproject(const cv::Point2f& p2D) const override;
    cv::Mat         toK() const override;
    cv::Vec4f       toD() const override;
    Eigen::Matrix3f toK_() const override;
    Eigen::Matrix3f getRotMat() const override;
    void            calculateCameraRotation(const uto::proto::CameraCalib& calib);
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
    void                        convert2Map();
    cv::Point2f                 projectToCylindrical(const Eigen::Vector3f& ray, int cols, int rows);
    cv::Point2f                 unprojectFisheye(const cv::Point2f& point, const Camera* const camera, int cols, int rows);
    std::vector<int>            fov2Pixel(const float& theta1, const float& theta2);
    std::pair<cv::Mat, cv::Mat> getMap() const { return remap_; };

    friend std::ostream& operator<<(std::ostream& os, const Fisheye& kb);
    friend std::istream& operator>>(std::istream& is, Fisheye& kb);
  };
} // namespace fs

#endif //CAMERAMODELS_KANNALABRANDT8_H
