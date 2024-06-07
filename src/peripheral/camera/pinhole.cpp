#include "pinhole.h"

namespace fs
{

  Eigen::Vector2d Pinhole::transformCam2Img(const Eigen::Vector3d &v3D) const
  {
    Eigen::Vector2d res;
    res[0] = param_[0] * v3D[0] / v3D[2] + param_[2];
    res[1] = param_[1] * v3D[1] / v3D[2] + param_[3];

    return res;
  }

  Eigen::Vector2f Pinhole::transformCam2Img(const Eigen::Vector3f &v3D) const
  {
    Eigen::Vector2f res;
    res[0] = param_[0] * v3D[0] / v3D[2] + param_[2];
    res[1] = param_[1] * v3D[1] / v3D[2] + param_[3];

    return res;
  }

  FSVec3f Pinhole::transformCam2Ego(const FSVec3f &pointCam) const
  {
    return Rbc_ * pointCam + tbc_;
  }

  FSVec3f Pinhole::transformEgo2Cam(const FSVec3f &pointEgo) const
  {
    return Rbc_.transpose() * (pointEgo - tbc_);
  }

  cv::Point3f Pinhole::unproject(const cv::Point2f &p2D) const
  {
    return cv::Point3f(
      (p2D.x - param_[2]) / param_[0],
      (p2D.y - param_[3]) / param_[1],
      1.f);
  }

  Eigen::Vector3f Pinhole::unprojectEig(const cv::Point2f &p2D) const
  {
    return Eigen::Vector3f(
      (p2D.x - param_[2]) / param_[0],
      (p2D.y - param_[3]) / param_[1],
      1.f);
  }

  cv::Mat Pinhole::toK() const
  {
    // clang-format off
    cv::Mat K = (cv::Mat_<float>(3, 3) << param_[0],             0.f, param_[2],
                                                0.f,       param_[1], param_[3],
                                                0.f,             0.f,       1.f);
    // clang-format on
    return K;
  }

  cv::Vec4f Pinhole::toD() const
  {
    cv::Vec4f D(param_[4], param_[5], param_[6], param_[7]);
    return D;
  }

  Eigen::Matrix3f Pinhole::toK_() const
  {
    Eigen::Matrix3f K;
    // clang-format off
    K << param_[0],       0.f, param_[2],
               0.f, param_[1], param_[3],
               0.f,       0.f,       1.f;
    // clang-format on
    return K;
  }

#if FS_CHECK(CFG_ROS2)
  void Pinhole::updateCalibInfo(const uto::proto::CameraCalib &calib)
  {
    updateIntrinsics(calib);
    updateExtrinsics(calib);
    valid_calib_ = true;
  }

  void Pinhole::updateImageSize(const uto::proto::SensorTable_CameraConfig &calib)
  {
    image_height_ = calib.image_size().height();
    image_width_  = calib.image_size().width();
    valid_size_   = true;
  }

  void Pinhole::updateIntrinsics(const uto::proto::CameraCalib &calib)
  {
    param_ = {calib.intrinsic().fx(), calib.intrinsic().fy(), calib.intrinsic().cx(), calib.intrinsic().cy()};
  }

  void Pinhole::updateExtrinsics(const uto::proto::CameraCalib &calib)
  {
    Rbc_ = uto::rotmatFromEuler<float, float>(uto::degToRad(calib.rotation_rpy_deg().x()), uto::degToRad(calib.rotation_rpy_deg().y()), uto::degToRad(calib.rotation_rpy_deg().z()));
    tbc_ << calib.translation().x(), calib.translation().y(), calib.translation().z();
    T2R(Tbc_) = Rbc_;
    T2t(Tbc_) = tbc_;
  }
#else
  void Pinhole::readCalibInfo(const int vehicleId, SensorId sensorId)
  {
    valid_size_  = true;
    valid_calib_ = updateIntrinsics(vehicleId, sensorId) && updateExtrinsics(vehicleId, sensorId);
  }

  bool Pinhole::updateIntrinsics(const int vehicleId, SensorId sensorId)
  {
    bool ret = false;

    const std::string intrinsicFile = "config/" + std::to_string(vehicleId) + "/" + SENSOR_ID_TO_STRING.at(sensorId) + "/cameraIntrinsic.xml";

    const cv::FileStorage fs(intrinsicFile, cv::FileStorage::READ);
    if(fs.isOpened())
    {
      std::string cameraModel;
      fs["camIntrinsicMat"] >> m_originalIntrinsics;
      fs["distortion_coefficients"] >> m_distortionCoefficients;
      fs["camera_model"] >> cameraModel;
      m_originalIntrinsics.convertTo(m_originalIntrinsics, CV_32F);
      m_distortionCoefficients.convertTo(m_distortionCoefficients, CV_32F);

      m_model = STRING_TO_CAMERA_MODEL.at(cameraModel);

      switch(sensorId)
      {
      case SensorId::CAMERA_FW:
        image_width_  = 3840;
        image_height_ = 2160;
        break;
      case SensorId::CAMERA_FN:
        image_width_  = 3840;
        image_height_ = 2160;
        break;
      case SensorId::CAMERA_FL:
        break;
      case SensorId::CAMERA_FR:
        break;
      case SensorId::CAMERA_RW:
        break;
      case SensorId::CAMERA_RN:
        break;
      case SensorId::CAMERA_RL:
        break;
      case SensorId::CAMERA_RR:
        image_width_  = 2880;
        image_height_ = 1860;
        break;
      default:
        UERROR << "unknown camera";
        return false;
      }

      const cv::Mat  E = cv::Mat::eye(3, 3, cv::DataType<float>::type);
      const cv::Size imageSize{image_width_, image_height_};

      switch(m_model)
      {
      case CameraModel::PINHOLE:
        m_newIntrinsics = cv::getOptimalNewCameraMatrix(m_originalIntrinsics, m_distortionCoefficients, imageSize, 1);
        cv::initUndistortRectifyMap(m_originalIntrinsics, m_distortionCoefficients, E, m_newIntrinsics, imageSize, CV_16SC2, m_map1, m_map2);
        break;
      case CameraModel::FISHEYE:
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(m_originalIntrinsics, m_distortionCoefficients, imageSize, E, m_newIntrinsics);
        cv::fisheye::initUndistortRectifyMap(m_originalIntrinsics, m_distortionCoefficients, E, m_newIntrinsics, imageSize, CV_16SC2, m_map1, m_map2);
        break;
      default:
        UERROR << "unknown camera model";
        return false;
      }

      cv::cv2eigen(m_newIntrinsics, K_);

      fx_    = K_(0, 0);
      fy_    = K_(1, 1);
      cx_    = K_(0, 2);
      cy_    = K_(1, 2);
      k_inv_ = K_.inverse();

      ret = true;
    }
    else
    {
      UERROR << "load intrinsic file failed: " << intrinsicFile.c_str();
    }

    return ret;
  }

  bool Pinhole::updateExtrinsics(const int vehicleId, SensorId sensorId)
  {
    bool ret = false;

    const std::string     extrinsicFile = "config/" + std::to_string(vehicleId) + "/" + SENSOR_ID_TO_STRING.at(sensorId) + "/cameraExtrinsic.xml";
    const cv::FileStorage fs(extrinsicFile, cv::FileStorage::READ);

    if(fs.isOpened())
    {
      cv::Mat     extrinsicMat;
      cv::Point2f headLocation;
      fs["extrinsicMatrix"] >> extrinsicMat;
      fs["headLocation"] >> headLocation;

      Eigen::Matrix<float, 4, 4> transformA2O;
      transformA2O << 1.f, 0.f, 0.f, -headLocation.x,
        0.f, 1.f, 0.f, -headLocation.y,
        0.f, 0.f, 1.f, 0.f,
        0.f, 0.f, 0.f, 1.f;

      Eigen::Matrix<float, 4, 4> transformCam2A;
      cv::cv2eigen(extrinsicMat, transformCam2A);

      Eigen::Matrix<float, 4, 4> transformO2Ego;
      if(SensorId::CAMERA_RW == sensorId)
      {
        transformO2Ego << 0.f, -1.f, 0.f, -2900.f,
          1.f, 0.f, 0.f, 0.f,
          0.f, 0.f, 1.f, 0.f,
          0.f, 0.f, 0.f, 0.f;
      }
      else
      {
        transformO2Ego << 0.f, 1.f, 0.f, 5046.f,
          -1.f, 0.f, 0.f, 0.f,
          0.f, 0.f, 1.f, 0.f,
          0.f, 0.f, 0.f, 1.f;
      }

      const Eigen::Matrix<float, 4, 4> transformCam2Ego = transformO2Ego * transformA2O * transformCam2A.inverse();

      Rbc_ = transformCam2Ego.block<3, 3>(0, 0);
      tbc_ = 0.001f * transformCam2Ego.block<3, 1>(0, 3);
      if(SensorId::CAMERA_RW == sensorId)
      {
        rotCam2Tlr_ = Rbc_;
        tbc_.x() -= 8.2f;
      }
      T2R(Tbc_) = Rbc_;
      T2t(Tbc_) = tbc_;

      ret = true;
    }
    else
    {
      UERROR << "load extrinsic file failed: " << extrinsicFile.c_str();
    }

    return ret;
  }
#endif

  Eigen::Matrix3f Pinhole::getRotMat() const
  {
    Eigen::Matrix3f rot_mat;
    rot_mat = rot_mat_;

    return rot_mat;
  }
  std::ostream &operator<<(std::ostream &os, const Pinhole &ph)
  {
    os << ph.param_[0] << " " << ph.param_[1] << " " << ph.param_[2] << " " << ph.param_[3];
    return os;
  }

  std::istream &operator>>(std::istream &is, Pinhole &ph)
  {
    float nextParam;
    for(size_t i = 0; i < 4; i++)
    {
      assert(is.good()); //Make sure the input stream is good
      is >> nextParam;
      ph.param_[i] = nextParam;
    }
    return is;
  }
} // namespace fs