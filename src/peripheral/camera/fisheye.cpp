#include "fisheye.h"
namespace fs
{
  Eigen::Vector2d Fisheye::transformCam2Img(const Eigen::Vector3d &v3D) const
  {
    const double x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
    const double theta      = atan2f(sqrtf(x2_plus_y2), v3D[2]);
    const double psi        = atan2f(v3D[1], v3D[0]);

    const double theta2 = theta * theta;
    const double theta3 = theta * theta2;
    const double theta5 = theta3 * theta2;
    const double theta7 = theta5 * theta2;
    const double theta9 = theta7 * theta2;
    const double r      = theta + param_[4] * theta3 + param_[5] * theta5 + param_[6] * theta7 + param_[7] * theta9;

    Eigen::Vector2d res;
    res[0] = param_[0] * r * cos(psi) + param_[2];
    res[1] = param_[1] * r * sin(psi) + param_[3];

    return res;
  }

  Eigen::Vector2f Fisheye::transformCam2Img(const Eigen::Vector3f &v3D) const
  {
    const float x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
    const float theta      = atan2f(sqrtf(x2_plus_y2), v3D[2]);
    const float psi        = atan2f(v3D[1], v3D[0]);

    const float theta2 = theta * theta;
    const float theta3 = theta * theta2;
    const float theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2;
    const float theta9 = theta7 * theta2;
    const float r      = theta + param_[4] * theta3 + param_[5] * theta5 + param_[6] * theta7 + param_[7] * theta9;

    Eigen::Vector2f res;
    res[0] = param_[0] * r * cos(psi) + param_[2];
    res[1] = param_[1] * r * sin(psi) + param_[3];

    return res;
  }

  Eigen::Vector3f Fisheye::transformCam2Ego(const Eigen::Vector3f &pointCam) const
  {
    return Rbc_ * pointCam + tbc_;
  }

  Eigen::Vector3f Fisheye::transformEgo2Cam(const Eigen::Vector3f &pointEgo) const
  {
    return Rbc_.transpose() * (pointEgo - tbc_);
  }

  cv::Point3f Fisheye::unproject(const cv::Point2f &p2D) const
  {
    // Use Newthon method to solve for theta with good precision (err ~ e-6)
    cv::Point2f pw((p2D.x - param_[2]) / param_[0], (p2D.y - param_[3]) / param_[1]);
    float       scale   = 1.f;
    float       theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);                 // sin(psi) = yc / r
    theta_d             = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f); // 不能超过180度

    if(theta_d > 1e-8)
    {
      // Compensate distortion iteratively
      // θ的初始值定为了θd
      float theta = theta_d;

      // 开始迭代
      for(int j = 0; j < 10; j++)
      {
        float theta2    = theta * theta,
              theta4    = theta2 * theta2,
              theta6    = theta4 * theta2,
              theta8    = theta4 * theta4;
        float k0_theta2 = param_[4] * theta2,
              k1_theta4 = param_[5] * theta4;
        float k2_theta6 = param_[6] * theta6,
              k3_theta8 = param_[7] * theta8;
        float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                          (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
        theta = theta - theta_fix;
        if(std::fabs(theta_fix) < 1e-6) // 如果更新量变得很小，表示接近最终值
        {
          break;
        }
      }
      // scale = theta - theta_d;
      // 求得tan(θ) / θd
      scale = std::tan(theta) / theta_d;
    }

    return cv::Point3f(pw.x * scale, pw.y * scale, 1.f);
  }

  Eigen::Vector3f Fisheye::unprojectEig(const cv::Point2f &p2D) const
  {
    cv::Point3f ray = this->unproject(p2D);
    return Eigen::Vector3f(ray.x, ray.y, ray.z);
  }

  cv::Mat Fisheye::toK() const
  {
    cv::Mat K = (cv::Mat_<float>(3, 3) << param_[0], 0.f, param_[2], 0.f, param_[1], param_[3], 0.f, 0.f, 1.f);
    return K;
  }

  Eigen::Matrix3f Fisheye::toK_() const
  {
    Eigen::Matrix3f K;
    K << param_[0], 0.f, param_[2], 0.f, param_[1], param_[3], 0.f, 0.f, 1.f;
    return K;
  }

  cv::Vec4f Fisheye::toD() const
  {
    cv::Vec4f D(param_[4], param_[5], param_[6], param_[7]);
    return D;
  }

#if FS_CHECK(CFG_ROS2)
  void Fisheye::updateCalibInfo(const uto::proto::CameraCalib &calib)
  {
    updateIntrinsics(calib);
    updateExtrinsics(calib);
    calculateCameraRotation(calib);
    valid_calib_ = true;
  }

  void Fisheye::updateImageSize(const uto::proto::SensorTable_CameraConfig &calib)
  {
    image_height_ = calib.image_size().height();
    image_width_  = calib.image_size().width();
    valid_size_   = true;
  }

  void Fisheye::updateIntrinsics(const uto::proto::CameraCalib &calib)
  {
    param_ = {calib.intrinsic().fx(),
              calib.intrinsic().fy(),
              calib.intrinsic().cx(),
              calib.intrinsic().cy(),
              calib.fisheye_coeffs().k1(),
              calib.fisheye_coeffs().k2(),
              calib.fisheye_coeffs().k3(),
              calib.fisheye_coeffs().k4()};
  }

  void Fisheye::updateExtrinsics(const uto::proto::CameraCalib &calib)
  {
    Rbc_ = uto::rotmatFromEuler<float, float>(uto::degToRad(calib.rotation_rpy_deg().x()), uto::degToRad(calib.rotation_rpy_deg().y()), uto::degToRad(calib.rotation_rpy_deg().z()));
    tbc_ << calib.translation().x(), calib.translation().y(), calib.translation().z();
    T2R(Tbc_) = Rbc_;
    T2t(Tbc_) = tbc_;
  }

#else
  void Fisheye::readCalibInfo(const int vehicleId, SensorId sensorId)
  {
    valid_size_  = true;
    valid_calib_ = updateIntrinsics(vehicleId, sensorId) && updateExtrinsics(vehicleId, sensorId);
  }

  bool Fisheye::updateIntrinsics(const int vehicleId, SensorId sensorId)
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

  bool Fisheye::updateExtrinsics(const int vehicleId, SensorId sensorId)
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

  void Fisheye::calculateCameraRotation(const uto::proto::CameraCalib &calib)
  {
    // float roll = M_PI/2.0 - (uto::degToRad(calib.rotation_rpy_deg().x()) + M_PI);
    float roll  = uto::degToRad(calib.rotation_rpy_deg().x());
    float pitch = uto::degToRad(calib.rotation_rpy_deg().y());
    // float yaw = 0.0;
    float yaw = uto::degToRad(calib.rotation_rpy_deg().z());
    UINFO << " x " << calib.rotation_rpy_deg().x() << " y " << calib.rotation_rpy_deg().y() << "yaw: " << calib.rotation_rpy_deg().z();

    if(yaw > M_PI / 4.0 && yaw < M_PI * 3.0 / 4.0)
    {
      yaw = M_PI / 2.0 - yaw;
    }
    else if(yaw > M_PI * 3.0 / 4.0 && yaw <= M_PI)
    {
      yaw = M_PI - yaw;
    }
    else if(yaw > -M_PI * 3.0 / 4.0 && yaw < -M_PI / 4.0)
    {
      yaw = -M_PI / 2.0 - yaw;
    }
    else if(yaw >= -M_PI && yaw < -M_PI * 3.0 / 4.0)
    {
      yaw = -M_PI - yaw;
    }
    else
    {
      yaw = 0 - yaw;
    }
    UINFO << "roll: " << roll * 180 / M_PI << " " << pitch * 180 / M_PI << " " << yaw * 180 / M_PI;
    // rot_mat_ = uto::rotmatFromEuler<float, float>(roll, pitch, yaw);
    // 相机系下
    rot_mat_ = uto::rotmatFromEuler<float, float>(-roll - M_PI / 2.0, -pitch, -yaw);
    UINFO << "roll: " << (-roll - M_PI / 2.0) * 180 / M_PI << " " << -pitch * 180 / M_PI << " " << -yaw * 180 / M_PI;

    UINFO << " rot_mat_ " << rot_mat_;
  }

  cv::Point2f Fisheye::projectToCylindrical(const Eigen::Vector3f &ray, int cols, int rows)
  {
    // float margin = 45 * CV_PI / 180.;
    double sq    = std::sqrt(ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2]);
    float  theta = acosf(-ray[1] / sq);
    float  phi   = acosf(-ray[0] / (sq * sin(theta)));
    float  u     = phi * cols / CV_PI;
    float  v     = (theta - MARGIN) * rows / (CV_PI - MARGIN);
    return cv::Point2f(u, v);
  }
  cv::Point2f Fisheye::unprojectFisheye(const cv::Point2f &point, const Camera *const camera, int cols, int rows)
  {
    std::vector<cv::Point2f> distortedPoints = {point};
    std::vector<cv::Point2f> undistortedPoints;
    // 使用畸变系数校正 2D 点
    cv::fisheye::undistortPoints(distortedPoints, undistortedPoints, camera->toK(), camera->toD());
    Eigen::Vector3f p_cam;
    p_cam << undistortedPoints[0].x, undistortedPoints[0].y, 1.0;

    Eigen::Vector3f ray = camera->getRotMat().inverse() * p_cam;
    // 计算柱面坐标
    return projectToCylindrical(ray, cols, rows);
  }

  Eigen::Matrix3f Fisheye::getRotMat() const
  {
    Eigen::Matrix3f rot_mat;
    rot_mat = rot_mat_;

    return rot_mat;
  }
  std::vector<int> Fisheye::fov2Pixel(const float &theta1, const float &theta2)
  {
    const float w = param_[0] * theta1;
    const float h = param_[1] * theta2;

    const int start_x = param_[2] - w;
    const int end_x   = param_[2] + w;

    const int start_y = param_[3] - h;
    const int end_y   = param_[3] + h;
    return std::vector<int>{start_x, start_y, end_x, end_y};
  }

  void Fisheye::convert2Map()
  {
    float   height = image_height_ * VIS_IMAGE_VIEW_SCALING;
    float   width  = image_width_ * VIS_IMAGE_VIEW_SCALING;
    cv::Mat map1, map2;
    map1.create(image_height_ * VIS_IMAGE_VIEW_SCALING, image_width_ * VIS_IMAGE_VIEW_SCALING, CV_32F);
    map2.create(image_height_ * VIS_IMAGE_VIEW_SCALING, image_width_ * VIS_IMAGE_VIEW_SCALING, CV_32F);
    cv::Mat     cylindrical_image;
    const float width_pi         = image_width_ * VIS_IMAGE_VIEW_SCALING / CV_PI;
    const float height_pi_margin = height / (CV_PI - MARGIN);
    for(int row_index = 0; row_index < image_height_ * VIS_IMAGE_VIEW_SCALING; ++row_index)
    {
      for(int col_index = 0; col_index < image_width_ * VIS_IMAGE_VIEW_SCALING; ++col_index)
      {
        double theta = col_index / width_pi;
        double phi   = row_index / height_pi_margin + MARGIN;
        //        UINFO << "theta: " << width_pi << " phi: " << phi;

        Eigen::Vector3f cylindric_point3(-sin(phi) * cos(theta), -cos(phi), sin(phi) * sin(theta));
        cylindric_point3 = getRotMat() * cylindric_point3;

        std::vector<cv::Point3f> objectPoints{cv::Point3f(cylindric_point3[0], cylindric_point3[1], cylindric_point3[2])};
        std::vector<cv::Point2f> imagePoints;
        cv::fisheye::projectPoints(objectPoints, imagePoints, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), toK(), toD());

        float x = imagePoints[0].x * VIS_IMAGE_VIEW_SCALING;
        float y = imagePoints[0].y * VIS_IMAGE_VIEW_SCALING;
        if(x < 0 || x >= width || y < 0 || y >= height || objectPoints[0].z < 0)
        {
          map1.at<float>(row_index, col_index) = -1;
          map2.at<float>(row_index, col_index) = -1;
        }
        else
        {
          map1.at<float>(row_index, col_index) = x;
          map2.at<float>(row_index, col_index) = y;
        }
      }
    }
    remap_ = std::make_pair(map1, map2);
  }

  std::ostream &operator<<(std::ostream &os, const Fisheye &kb)
  {
    os << kb.param_[0] << " " << kb.param_[1] << " " << kb.param_[2] << " " << kb.param_[3] << " "
       << kb.param_[4] << " " << kb.param_[5] << " " << kb.param_[6] << " " << kb.param_[7];
    return os;
  }

  std::istream &operator>>(std::istream &is, Fisheye &kb)
  {
    float nextParam;
    for(size_t i = 0; i < 8; i++)
    {
      assert(is.good()); // Make sure the input stream is good
      is >> nextParam;
      kb.param_[i] = nextParam;
    }
    return is;
  }

} // namespace fs
