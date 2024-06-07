
#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <vector>
#include "eigen3/Eigen/Dense"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <camera_calib.pb.h>
#include "common/math/rotation_conversion.h"
#include "peripheral/types.h"
#include "peripheral/defs.h"

namespace fs
{
  class Camera
  {
  public:
    Camera() {}
    Camera(const std::vector<float>& _vParameters):
      param_(_vParameters) {}
    virtual ~Camera() {}

    virtual Eigen::Vector2d transformCam2Img(const Eigen::Vector3d& v3D) const = 0;
    virtual Eigen::Vector2f transformCam2Img(const Eigen::Vector3f& v3D) const = 0;
    virtual FSVec3f         transformCam2Ego(const FSVec3f& pointCam) const    = 0;
    virtual FSVec3f         transformEgo2Cam(const FSVec3f& pointEgo) const    = 0;
    virtual cv::Point3f     unproject(const cv::Point2f& p2D) const            = 0;
    virtual Eigen::Vector3f unprojectEig(const cv::Point2f& p2D) const         = 0;
    virtual cv::Mat         toK() const                                        = 0;
    virtual cv::Vec4f       toD() const                                        = 0;
    virtual Eigen::Matrix3f toK_() const                                       = 0;
    virtual Eigen::Matrix3f getRotMat() const                                  = 0;

#if FS_CHECK(CFG_ROS2)
    virtual void updateIntrinsics(const uto::proto::CameraCalib& calib)             = 0;
    virtual void updateExtrinsics(const uto::proto::CameraCalib& calib)             = 0;
    virtual void updateCalibInfo(const uto::proto::CameraCalib& calib)              = 0;
    virtual void updateImageSize(const uto::proto::SensorTable_CameraConfig& calib) = 0;
#else
    virtual void readCalibInfo(const int vehicleId, SensorId sensorId)    = 0;
    virtual bool updateIntrinsics(const int vehicleId, SensorId sensorId) = 0;
    virtual bool updateExtrinsics(const int vehicleId, SensorId sensorId) = 0;
    cv::Mat      getmap1() const { return m_map1; }
    cv::Mat      getmap2() const { return m_map2; }
#endif
    // virtual void            calculateCameraRotation(const uto::proto::CameraCalib& calib)      = 0;
    bool isCalibValid() const { return valid_calib_; }
    bool isSizeValid() const { return valid_size_; }
    bool isValid() const { return valid_calib_ && valid_size_; }
    int  getImageWidth() const { return image_width_; }
    int  getImageHeight() const { return image_height_; }

  protected:
    bool               valid_calib_ = false;
    bool               valid_size_  = false;
    std::vector<float> param_; // -- Pinhole fx fy cx cy  KB8: fx fy cx cy k1 k2 k3 k4
    int                image_width_  = 0;
    int                image_height_ = 0;
    Eigen::Vector3f    tbc_;
    Eigen::Matrix3f    Rbc_;
    Eigen::Matrix4f    Tbc_;
    Eigen::Matrix3f    rot_mat_;

#if !FS_CHECK(CFG_ROS2)
    // following attributes only for opencv distortion
    CameraModel m_model;

    float    fx_ = 0.f;
    float    fy_ = 0.f;
    float    cx_ = 0.f;
    float    cy_ = 0.f;
    FSMat3x3 K_;
    FSMat3x3 k_inv_;
    cv::Mat  m_map1;
    cv::Mat  m_map2;
    cv::Mat  m_originalIntrinsics;
    cv::Mat  m_newIntrinsics;
    cv::Mat  m_distortionCoefficients;
    FSMat3x3 rotCam2Tlr_; ///< rotation matrix camera to trailer, only work for rear center camera
#endif
  };
} // namespace fs

#endif //CAMERAMODELS_GEOMETRICCAMERA_H
