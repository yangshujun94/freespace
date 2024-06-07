#include "fs_node.h"

#if FS_CHECK(CFG_VIS_ENABLE)
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#endif

#include <common/proto_utils.h>
#include <project_config.pb.h>
#include "utils.h"
#include "camera_manager.h"
#include "vehicle.h"
#include "peripheral/diagnoser.h"
#include "peripheral/map_provider/map_provider.h"
#include "version.h"
#include "debug/fs_log.h"

extern const char* GIT_COMMIT_SHA1;
extern const char* GIT_COMMIT_DATE;
extern const char* GIT_COMMIT_SUBJECT;
extern const char* CICD_VEHICLE_TYPE;
extern const char* CICD_TARGET_BOARD;
extern const char* CICD_MIDDLEWARE;
extern const char* BUILD_PROJECT_FLAG;
extern const char* BUILD_CMAKE_TYPE;
extern const char* BUILD_VIS_ENABLE;
extern const char* BUILD_LOAD_ENABLE;

void fs::FsNode::init()
{
  UERROR << "commit version: " << GIT_COMMIT_SHA1;
  UERROR << "commit date   : " << GIT_COMMIT_DATE;
  UERROR << "commit message: " << GIT_COMMIT_SUBJECT;
  UERROR << "cicd vehicle_type:" << CICD_VEHICLE_TYPE << ",target_board:" << CICD_TARGET_BOARD << ",middleware:" << CICD_MIDDLEWARE;
  UERROR << "build project_flag:" << BUILD_PROJECT_FLAG << ",cmake_type:" << BUILD_CMAKE_TYPE << ",vis:" << BUILD_VIS_ENABLE << ",load_log:" << BUILD_LOAD_ENABLE;
  UERROR << "fs software version: " << MAJOR_VERSION << "." << MINOR_VERSION << "." << PATCH_VERSION;

  uto::proto::ProjectConfig cfg;
  std::string               strProjectName;
  if(uto::loadProtoTextFile(PROJECT_CONFIG_FILE, &cfg))
  {
    strProjectName = cfg.project_name();
    UERROR << "load parameter 'project_config' success, project_name is " << strProjectName.c_str() << ", license_plate is " << cfg.license_plate().c_str();
  }
  else
  {
    UERROR << "loadProtoBinaryFile failed, use ros args!";
    this->declare_parameter<std::string>("project_name", "none");
    this->get_parameter("project_name", strProjectName);
  }
  fs::MapProvider::create(strProjectName);

  std::string recordingPath;
  declare_parameter<std::string>("recording_path", "none");
  get_parameter<std::string>("recording_path", recordingPath);

#if FS_CHECK(CFG_LOAD_RECORDING)
  assert(access(recordingPath.c_str(), 0) == F_OK);
  m_bagReaderPtr->open(recordingPath);
#endif

#if FS_CHECK(CFG_VIS_ENABLE)
  std::string mcapPath;
  declare_parameter<std::string>("mcap_path", "/home");
  get_parameter<std::string>("mcap_path", mcapPath);
  rosbag2_storage::StorageOptions write_option;
  write_option.uri        = getOutFilePath(mcapPath, recordingPath);
  write_option.storage_id = "mcap";
  m_rosWriterPtr->open(write_option);
#endif
}

std::string fs::FsNode::getOutFilePath(const std::string& pathOut, const std::string& bagPath)
{
  const auto now           = std::chrono::system_clock::now();
  const auto now_in_time_t = std::chrono::system_clock::to_time_t(now) + 8 * 3600; // 将时间转换为东8区时间

  // 找到最后一个'/'的位置
  const size_t lastSlash = bagPath.find_last_of('/');

  // 提取文件名
  const std::string bagName = bagPath.substr(lastSlash + 1);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_in_time_t), "%Y_%m_%d-%H_%M_%S");
  return pathOut + "/" + bagName + "_" + ss.str();
}

void fs::FsNode::loadBag()
{
  while(true)
  {
    if(m_bagReaderPtr->has_next())
    {
      const auto message = m_bagReaderPtr->read_next();

      if(message->topic_name == IMAGE_CF_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        auto guard = m_rwLock.write_guard();
        m_imageCfBuffer.pushFrontForce(std::move(msgPtr));
      }

      if(message->topic_name == IMAGE_FISHEYE_FCF_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        auto guard = m_rwLock.write_guard();
        m_imageFisheyeFcfBuffer.pushFrontForce(std::move(msgPtr));
      }

      if(message->topic_name == IMAGE_FISHEYE_FLL_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        auto guard = m_rwLock.write_guard();
        m_imageFisheyeFllBuffer.pushFrontForce(std::move(msgPtr));
      }

      if(message->topic_name == IMAGE_FISHEYE_FRR_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        auto guard = m_rwLock.write_guard();
        m_imageFisheyeFrrBuffer.pushFrontForce(std::move(msgPtr));
      }

      if(message->topic_name == IMAGE_FISHEYE_BLL_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        auto guard = m_rwLock.write_guard();
        m_imageFisheyeBllBuffer.pushFrontForce(std::move(msgPtr));
      }

      if(message->topic_name == IMAGE_FISHEYE_BRR_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        auto guard = m_rwLock.write_guard();
        m_imageFisheyeBrrBuffer.pushFrontForce(std::move(msgPtr));
      }

      if(message->topic_name == IMAGE_FISHEYE_BCB_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        auto guard = m_rwLock.write_guard();
        m_imageFisheyeBcbBuffer.pushFrontForce(std::move(msgPtr));
      }

      if(message->topic_name == CALIB_CF_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        auto guard = m_rwLock.write_guard();
        CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_CENTER_FRONT);
      }

#if FS_CHECK(CFG_USE_FISHEYE_FS)
      if(message->topic_name == CALIB_FISHEYE_FCF_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        auto guard = m_rwLock.write_guard();
        CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_FCF);
      }

      if(message->topic_name == CALIB_FISHEYE_FLL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        auto guard = m_rwLock.write_guard();
        CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_FLL);
      }

      if(message->topic_name == CALIB_FISHEYE_FRR_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        auto guard = m_rwLock.write_guard();
        CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_FRR);
      }

      if(message->topic_name == CALIB_FISHEYE_BLL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        auto guard = m_rwLock.write_guard();
        CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_BLL);
      }

      if(message->topic_name == CALIB_FISHEYE_BRR_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        auto guard = m_rwLock.write_guard();
        CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_BRR);
      }

      if(message->topic_name == CALIB_FISHEYE_BCB_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        auto guard = m_rwLock.write_guard();
        CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_BCB);
      }
#endif

#if FS_CHECK(CFG_USE_CF_FS)
      if(message->topic_name == FS_CF_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);
        processCameraFs(m_fsCfBuffer, msgPtr, SensorId::CAMERA_CENTER_FRONT);
      }
#endif

#if FS_CHECK(CFG_USE_FISHEYE_FS)
      if(message->topic_name == FS_FISHEYE_FCF_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);
        processCameraFs(m_fsFisheyeFcfBuffer, msgPtr, SensorId::FISHEYE_FCF);
      }

      if(message->topic_name == FS_FISHEYE_FLL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);
        processCameraFs(m_fsFisheyeFllBuffer, msgPtr, SensorId::FISHEYE_FLL);
      }

      if(message->topic_name == FS_FISHEYE_FRR_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);
        processCameraFs(m_fsFisheyeFrrBuffer, msgPtr, SensorId::FISHEYE_FRR);
      }

      if(message->topic_name == FS_FISHEYE_BLL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);
        processCameraFs(m_fsFisheyeBllBuffer, msgPtr, SensorId::FISHEYE_BLL);
      }

      if(message->topic_name == FS_FISHEYE_BRR_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);
        processCameraFs(m_fsFisheyeBrrBuffer, msgPtr, SensorId::FISHEYE_BRR);
      }

      if(message->topic_name == FS_FISHEYE_BCB_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);
        processCameraFs(m_fsFisheyeBcbBuffer, msgPtr, SensorId::FISHEYE_BCB);
      }
#endif

      if(message->topic_name == LIDAR_FS_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::PerceptionFreespace>(*message);
        processLidarFs(msgPtr);
      }

      if(message->topic_name == OD_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::PerceptionObstacles>(*message);
        processOd(msgPtr);
      }

      if(message->topic_name == GATES_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::PerceptionGates>(*message);

        auto guard = m_rwLock.write_guard();
        m_gatesBuffer.pushBackForce(std::move(msgPtr));
      }

      if(message->topic_name == VEHICLE_POSE_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::VehiclePose>(*message);

        processVehiclePose(*msgPtr);

        if(++m_vehicleFrameCount >= 5)
        {
          m_vehicleFrameCount = 0;
          break;
        }
      }

      if(message->topic_name == MECHANICAL_INFO_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::MechanicalInfo>(*message); //TODO: waitting for common update
        auto guard  = m_rwLock.write_guard();
        Vehicle::instance().updateMechanicInfo(*msgPtr);
      }

      if(message->topic_name == SENSOR_TABLE_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::SensorTable>(*message);
        auto guard  = m_rwLock.write_guard();
        CameraManager::instance().updateConfigInfo(*msgPtr);
      }

      if(message->topic_name == ROAD_MODEL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::RoadModel>(*message);
        auto guard  = m_rwLock.write_guard();
        m_roadmodelBuffer.pushBackForce(std::move(msgPtr));
      }

#if FS_CHECK(CFG_VIS_DRAW_PERCEPTION_FREESPACE)
      if(message->topic_name == FUSION_FS_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::PerceptionFreespace>(*message);
        auto guard  = m_rwLock.write_guard();
        publishVisPerceptionFreespace(*msgPtr);
      }
#endif
    }
    else
    {
#if FS_CHECK(CFG_VIS_ENABLE)
      m_rosWriterPtr.reset();
#endif
      exit(0);
    }
  }
}

void fs::FsNode::subscribeImageCf(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  m_imageCfBuffer.pushFrontForce(std::move(msgPtr));
}

void fs::FsNode::subscribeImageFisheyeFcf(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  m_imageFisheyeFcfBuffer.pushFrontForce(std::move(msgPtr));
}

void fs::FsNode::subscribeImageFisheyeFll(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  m_imageFisheyeFllBuffer.pushFrontForce(std::move(msgPtr));
}

void fs::FsNode::subscribeImageFisheyeFrr(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  m_imageFisheyeFrrBuffer.pushFrontForce(std::move(msgPtr));
}

void fs::FsNode::subscribeImageFisheyeBll(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  m_imageFisheyeBllBuffer.pushFrontForce(std::move(msgPtr));
}

void fs::FsNode::subscribeImageFisheyeBrr(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  m_imageFisheyeBrrBuffer.pushFrontForce(std::move(msgPtr));
}

void fs::FsNode::subscribeImageFisheyeBcb(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  m_imageFisheyeBcbBuffer.pushFrontForce(std::move(msgPtr));
}

void fs::FsNode::subscribeCalibCf(std::unique_ptr<uto::proto::CameraCalib> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_CENTER_FRONT);
}

#if FS_CHECK(CFG_USE_FISHEYE_FS)
void fs::FsNode::subscribeCalibFisheyeFcf(std::unique_ptr<uto::proto::CameraCalib> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_FCF);
}

void fs::FsNode::subscribeCalibFisheyeFll(std::unique_ptr<uto::proto::CameraCalib> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_FLL);
}

void fs::FsNode::subscribeCalibFisheyeFrr(std::unique_ptr<uto::proto::CameraCalib> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_FRR);
}

void fs::FsNode::subscribeCalibFisheyeBll(std::unique_ptr<uto::proto::CameraCalib> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_BLL);
}

void fs::FsNode::subscribeCalibFisheyeBrr(std::unique_ptr<uto::proto::CameraCalib> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_BRR);
}

void fs::FsNode::subscribeCalibFisheyeBcb(std::unique_ptr<uto::proto::CameraCalib> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::FISHEYE_BCB);
}
#endif

void fs::FsNode::subscribeGates(std::unique_ptr<uto::proto::PerceptionGates> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  m_gatesBuffer.pushBackForce(std::move(msgPtr));
}

void fs::FsNode::subscribeMechanicalInfo(std::unique_ptr<uto::proto::MechanicalInfo> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  Vehicle::instance().updateMechanicInfo(*msgPtr);
}

void fs::FsNode::subscribeSensorTable(std::unique_ptr<uto::proto::SensorTable> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  CameraManager::instance().updateConfigInfo(*msgPtr);
}

void fs::FsNode::subscribeRoadModel(std::unique_ptr<uto::proto::RoadModel> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  m_roadmodelBuffer.pushBackForce(std::move(msgPtr));
}

#if FS_CHECK(CFG_VIS_DRAW_PERCEPTION_FREESPACE)
void fs::FsNode::subscribePerceptionFreespace(std::unique_ptr<uto::proto::PerceptionFreespace> msgPtr)
{
  auto guard = m_rwLock.write_guard();
  publishVisPerceptionFreespace(*msgPtr);
}
#endif

void fs::FsNode::processLidarFs(std::unique_ptr<uto::proto::PerceptionFreespace>& msgPtr)
{
  auto guard = m_rwLock.write_guard();
  Diagnoser::instance().healSensorTimeoutCount(SensorId::LIDAR);
  const auto&   header       = msgPtr->header();
  const int64_t emoTimestamp = Vehicle::getVehicle().getEmoBuffer().back().timestampUs * 1000;
  const int64_t delayTime    = emoTimestamp - header.time_meas();

  if(delayTime < SYNC_EARLY_ARRIVAL_NS || delayTime > SYNC_WARNING_NS)
  {
    Diagnoser::instance().incrementSensorOutOfRangeCount(SensorId::LIDAR);
    UWARN << "ldr fs " << SENSOR_ID_TO_STRING.at(SensorId::LIDAR) << " timestamp out of range, "
          << "meas ts: " << header.time_meas() << " ns, "
          << "emo ts: " << emoTimestamp << " ns, "
          << "pub - meas: " << header.time_pub() - header.time_meas() << " ns, "
          << "emo - meas: " << delayTime << " ns";
  }
  else
  {
    const int64_t cycleTime   = emoTimestamp - m_lidarLastSubTimestampNs;
    m_lidarLastSubTimestampNs = emoTimestamp;
    LOG_WARN("[sub lidar timestamp: %ld][meas delay: %ld][cycle time: %ld]", header.time_meas(), delayTime, cycleTime);

    Diagnoser::instance().healSensorOutOfRangeCount(SensorId::LIDAR);
  }

  m_lidarFsBuffer.pushBackForce(std::move(msgPtr));
}

void fs::FsNode::processCameraFs(fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>& camFsBuffer, std::unique_ptr<uto::proto::CameraFreespace>& msgPtr, const fs::SensorId sensorId)
{
  auto guard = m_rwLock.write_guard();
  Diagnoser::instance().healSensorTimeoutCount(sensorId);
  const auto&   header       = msgPtr->header();
  const int64_t emoTimestamp = Vehicle::getVehicle().getEmoBuffer().back().timestampUs * 1000;
  const int64_t delayTime    = emoTimestamp - header.time_meas();

  if(delayTime < SYNC_EARLY_ARRIVAL_NS || delayTime > SYNC_WARNING_NS)
  {
    Diagnoser::instance().incrementSensorOutOfRangeCount(sensorId);
    UWARN << "cam fs " << SENSOR_ID_TO_STRING.at(sensorId) << " timestamp out of range, "
          << "meas ts: " << header.time_meas() << " ns, "
          << "emo ts: " << emoTimestamp << " ns, "
          << "pub - meas: " << header.time_pub() - header.time_meas() << " ns, "
          << "emo - meas: " << delayTime << " ns";
  }
  else
  {
    if(SensorId::CAMERA_CENTER_FRONT == sensorId)
    {
      const int64_t cycleTime    = emoTimestamp - m_cameraLastSubTimestampNs;
      m_cameraLastSubTimestampNs = emoTimestamp;
      LOG_WARN("[sub camera timestamp: %ld][meas delay: %ld][cycle time: %ld]", header.time_meas(), delayTime, cycleTime);
    }
#if FS_CHECK(CFG_USE_FISHEYE_FS)
    else if(SensorId::FISHEYE_FCF == sensorId)
    {
      const int64_t cycleTime     = emoTimestamp - m_fisheyeLastSubTimestampNs;
      m_fisheyeLastSubTimestampNs = emoTimestamp;
      LOG_WARN("[sub fisheye timestamp: %ld][meas delay: %ld][cycle time: %ld]", header.time_meas(), delayTime, cycleTime);
    }
#endif
    Diagnoser::instance().healSensorOutOfRangeCount(sensorId);
  }

  camFsBuffer.pushBackForce(std::move(msgPtr));
}

void fs::FsNode::processOd(std::unique_ptr<uto::proto::PerceptionObstacles>& msgPtr)
{
  auto guard = m_rwLock.write_guard();
  Diagnoser::instance().healOdTimeoutCount();
  const auto&   header       = msgPtr->header();
  const int64_t emoTimestamp = Vehicle::getVehicle().getEmoBuffer().back().timestampUs * 1000;
  const int64_t delayTime    = emoTimestamp - header.time_meas();

  if(delayTime < SYNC_EARLY_ARRIVAL_NS || delayTime > SYNC_WARNING_NS)
  {
    Diagnoser::instance().incrementOdOutOfRangeCount();
    UWARN << "od timestamp out of range, "
          << "meas ts: " << header.time_meas() << " ns, "
          << "emo ts: " << emoTimestamp << " ns, "
          << "pub - meas: " << header.time_pub() - header.time_meas() << " ns, "
          << "emo - meas: " << delayTime << " ns";
  }
  else
  {
    Diagnoser::instance().healOdOutOfRangeCount();
    const int64_t cycleTime = emoTimestamp - m_odLastSubTimestampNs;
    m_odLastSubTimestampNs  = emoTimestamp;
    LOG_WARN("[sub od timestamp: %ld][meas delay: %ld][cycle time: %ld]", header.time_meas(), delayTime, cycleTime);
  }

  m_odBuffer.pushBackForce(std::move(msgPtr));
}

void fs::FsNode::processVehiclePose(const uto::proto::VehiclePose& vehiclePose)
{
  auto guard = m_rwLock.write_guard();

  Diagnoser::instance().healEgoMotionTimeoutCount();
  // only INVALID needs to be reported
  if(uto::proto::VehiclePose_Status_INVALID == vehiclePose.status())
  {
    // we have to push invalid ego motion into the buffer, otherwise the synchronization for lidar fs will fail,
    // and we can't come into back up mode.
    UWARN_EVERY(10) << "vehicle pose invalid, timestamp: " << vehiclePose.header().time_meas();
    Diagnoser::instance().setPoseInvalid();
  }
  else
  {
    Diagnoser::instance().healPoseInvalid();
  }

  auto egoMotionPtr             = std::make_unique<EgoMotion>();
  egoMotionPtr->timestampUs     = vehiclePose.header().time_meas() / 1000;
  egoMotionPtr->translation.x() = vehiclePose.pos().x();
  egoMotionPtr->translation.y() = vehiclePose.pos().y();
  egoMotionPtr->translation.z() = vehiclePose.pos().z();

  egoMotionPtr->quaternion = Eigen::AngleAxisf(vehiclePose.attitude_rpy().z(), EVector3::UnitZ()) *
                             Eigen::AngleAxisf(vehiclePose.attitude_rpy().y(), EVector3::UnitY()) *
                             Eigen::AngleAxisf(vehiclePose.attitude_rpy().x(), EVector3::UnitX());

  egoMotionPtr->translationLtm.x() = vehiclePose.align_pos().x();
  egoMotionPtr->translationLtm.y() = vehiclePose.align_pos().y();
  egoMotionPtr->translationLtm.z() = vehiclePose.align_pos().z();

  egoMotionPtr->quaternionLtm = Eigen::AngleAxisf(vehiclePose.align_attitude_rpy().z(), EVector3::UnitZ()) *
                                Eigen::AngleAxisf(vehiclePose.align_attitude_rpy().y(), EVector3::UnitY()) *
                                Eigen::AngleAxisf(vehiclePose.align_attitude_rpy().x(), EVector3::UnitX());

  Vehicle& vehicle = Vehicle::instance();
  vehicle.pushEmoBuffer(egoMotionPtr);
}

void fs::FsNode::removeObsoleteInputs()
{
  // at least, sensors used for alignment should be cleared
  while(!m_fsCfBuffer.empty() && m_fsCfBuffer.front()->header().time_meas() <= m_fusionTimestampNs)
  {
    UWARN << "remove obsolete fc120 input, ts: " << m_fsCfBuffer.front()->header().time_meas() << " ns, "
          << "fusion ts: " << m_fusionTimestampNs << " ns";
    m_fsCfBuffer.popFront();
  }
  while(!m_lidarFsBuffer.empty() && m_lidarFsBuffer.front()->header().time_meas() <= m_fusionTimestampNs)
  {
    UWARN << "remove obsolete fc120 input, ts: " << m_lidarFsBuffer.front()->header().time_meas() << " ns, "
          << "fusion ts: " << m_fusionTimestampNs << " ns";
    m_lidarFsBuffer.popFront();
  }
}

bool fs::FsNode::getAlignedInputs(std::unique_ptr<uto::proto::PerceptionFreespace>&   fsLidarPtr,
                                  std::unique_ptr<uto::proto::CameraFreespace>&       fsCfPtr,
                                  std::unique_ptr<uto::proto::CameraFreespace>&       fsFisheyeFcfPtr,
                                  std::unique_ptr<uto::proto::CameraFreespace>&       fsFisheyeFllPtr,
                                  std::unique_ptr<uto::proto::CameraFreespace>&       fsFisheyeFrrPtr,
                                  std::unique_ptr<uto::proto::CameraFreespace>&       fsFisheyeBcbPtr,
                                  std::unique_ptr<uto::proto::CameraFreespace>&       fsFisheyeBllPtr,
                                  std::unique_ptr<uto::proto::CameraFreespace>&       fsFisheyeBrrPtr,
                                  std::unique_ptr<sensor_msgs::msg::CompressedImage>& imageCfPtr,
                                  std::unique_ptr<sensor_msgs::msg::CompressedImage>& imageFisheyeFcfPtr,
                                  std::unique_ptr<sensor_msgs::msg::CompressedImage>& imageFisheyeFllPtr,
                                  std::unique_ptr<sensor_msgs::msg::CompressedImage>& imageFisheyeFrrPtr,
                                  std::unique_ptr<sensor_msgs::msg::CompressedImage>& imageFisheyeBcbPtr,
                                  std::unique_ptr<sensor_msgs::msg::CompressedImage>& imageFisheyeBllPtr,
                                  std::unique_ptr<sensor_msgs::msg::CompressedImage>& imageFisheyeBrrPtr,
                                  std::unique_ptr<EgoMotion>&                         egoMotionPtr,
                                  std::unique_ptr<uto::proto::RoadModel>&             roadModelPtr)
{
  auto           guard     = m_rwLock.read_guard();
  const Vehicle& vehicle   = Vehicle::instance();
  const auto&    emoBuffer = vehicle.getEmoBuffer();

  const Diagnoser& diagnoser = Diagnoser::getDiagnoser();

  bool validAligned = false;

  if((diagnoser.isEgoMotionDefect() || diagnoser.hasSensorOutOfRangeError(SensorId::LIDAR)) && !m_lidarFsBuffer.empty())
  {
    validAligned = true;

    m_fusionTimestampNs = m_lidarFsBuffer.back()->header().time_meas();
    UWARN << "[ego motion defect][use lidar timestamp: " << m_fusionTimestampNs << "]";

    fsLidarPtr = std::move(m_lidarFsBuffer.back());
    m_lidarFsBuffer.clear();
  }
  else if(!emoBuffer.empty())
  {
    if(emoBuffer.back().timestampUs < emoBuffer.front().timestampUs)
    {
      // play reset
      UERROR << "The emo timestamp is fallback, front > back, reset all buffers";
      Vehicle::instance().resetEmoBuffer();
      resetBuffers();
    }
    else
    {
      const bool allInputsArrived =
#if FS_CHECK(CFG_USE_CF_FS)
        (diagnoser.isSensorDefect(SensorId::CAMERA_CENTER_FRONT) || !m_fsCfBuffer.empty()) &&
#endif
#if FS_CHECK(CFG_USE_FISHEYE_FS)
        (diagnoser.isSensorDefect(SensorId::FISHEYE_FCF) || !m_fsFisheyeFcfBuffer.empty()) &&
        (diagnoser.isSensorDefect(SensorId::FISHEYE_FLL) || !m_fsFisheyeFllBuffer.empty()) &&
        (diagnoser.isSensorDefect(SensorId::FISHEYE_FRR) || !m_fsFisheyeFrrBuffer.empty()) &&
        (diagnoser.isSensorDefect(SensorId::FISHEYE_BCB) || !m_fsFisheyeBcbBuffer.empty()) &&
        (diagnoser.isSensorDefect(SensorId::FISHEYE_BLL) || !m_fsFisheyeBllBuffer.empty()) &&
        (diagnoser.isSensorDefect(SensorId::FISHEYE_BRR) || !m_fsFisheyeBrrBuffer.empty()) &&
#endif
        !m_lidarFsBuffer.empty();

      const bool lidarOverdue           = !m_lidarFsBuffer.empty() && emoBuffer.back().timestampUs * 1e3 - m_lidarFsBuffer.front()->header().time_meas() > SYNC_MAX_WAIT_US * 1e3;
      const bool shouldTriggerAlignment = allInputsArrived || lidarOverdue;

      LOG_WARN("[should trigger alignment: %d][all inputs arrived: %d][lidar size: %zu]",
               shouldTriggerAlignment,
               allInputsArrived,
               m_lidarFsBuffer.size());

      if(shouldTriggerAlignment)
      {
        validAligned = true;

        m_fusionTimestampNs = m_lidarFsBuffer.front()->header().time_meas();
        UWARN << "[use lidar timestamp: " << m_fusionTimestampNs << "]"
              << "[meas time delay: " << emoBuffer.back().timestampUs * 1000 - m_fusionTimestampNs << " ns]";

        EgoMotion                egoMotion{};
        const Vehicle::EgoStatus emoStatus = vehicle.getEgoMotion(egoMotion, m_fusionTimestampNs / 1000);
        if(emoStatus == Vehicle::EgoStatus::NO_SMALLER_TIMESTAMP)
        {
          validAligned = false;
          // aligned timestamp is earlier than the earliest transform, means we can't have transform for this frame, and need reset sensor buffers
          // this should happen when the recording replays
          UWARN << "drop earlier fusion timestamp: " << m_fusionTimestampNs / 1000000 << " ms; "
                << "ego motion timestamp: front " << emoBuffer.front().timestampUs / 1000 << " ms | "
                << "back " << emoBuffer.back().timestampUs / 1000 << " ms";
          resetBuffers();
        }
        else if(emoStatus == Vehicle::EgoStatus::NO_GREATER_TIMESTAMP)
        {
          validAligned = false;
          // aligned timestamp is later than the latest transform , means we can't have transform for this frame, and need to wait for emo update
          // this should happen when the vehicle_pose message's disappearance or the vehicle_pose latency is too large
          UWARN << "drop later fusion timestamp: " << m_fusionTimestampNs / 1000000 << " ms; "
                << "ego motion timestamp: front " << emoBuffer.front().timestampUs / 1000 << " ms | "
                << "back " << emoBuffer.back().timestampUs / 1000 << " ms";
        }
        else
        {
          assert(emoStatus == Vehicle::EgoStatus::INTERPOLATED_TIMESTAMP);
          egoMotionPtr = std::make_unique<EgoMotion>(egoMotion);

          auto fsLidarIter = std::find_if(m_lidarFsBuffer.cbegin(), m_lidarFsBuffer.cend(), [this](const std::unique_ptr<uto::proto::PerceptionFreespace>& ptr) { return ptr->header().time_meas() > m_fusionTimestampNs - 50 * INT_1E6 && ptr->header().time_meas() < m_fusionTimestampNs + 50 * INT_1E6; });
#if FS_CHECK(CFG_USE_CF_FS)
          auto fsCfIter = std::find_if(m_fsCfBuffer.cbegin(), m_fsCfBuffer.cend(), [this](const std::unique_ptr<uto::proto::CameraFreespace>& ptr) { return ptr->header().time_meas() > m_fusionTimestampNs - 50 * INT_1E6 && ptr->header().time_meas() < m_fusionTimestampNs + 50 * INT_1E6; });
#endif
#if FS_CHECK(CFG_USE_FISHEYE_FS)
          auto fsFisheyeFcfIter = std::find_if(m_fsFisheyeFcfBuffer.cbegin(), m_fsFisheyeFcfBuffer.cend(), [this](const std::unique_ptr<uto::proto::CameraFreespace>& ptr) { return ptr->header().time_meas() > m_fusionTimestampNs - 50 * INT_1E6 && ptr->header().time_meas() < m_fusionTimestampNs + 50 * INT_1E6; });
          auto fsFisheyeFllIter = std::find_if(m_fsFisheyeFllBuffer.cbegin(), m_fsFisheyeFllBuffer.cend(), [this](const std::unique_ptr<uto::proto::CameraFreespace>& ptr) { return ptr->header().time_meas() > m_fusionTimestampNs - 50 * INT_1E6 && ptr->header().time_meas() < m_fusionTimestampNs + 50 * INT_1E6; });
          auto fsFisheyeFrrIter = std::find_if(m_fsFisheyeFrrBuffer.cbegin(), m_fsFisheyeFrrBuffer.cend(), [this](const std::unique_ptr<uto::proto::CameraFreespace>& ptr) { return ptr->header().time_meas() > m_fusionTimestampNs - 50 * INT_1E6 && ptr->header().time_meas() < m_fusionTimestampNs + 50 * INT_1E6; });
          auto fsFisheyeBllIter = std::find_if(m_fsFisheyeBllBuffer.cbegin(), m_fsFisheyeBllBuffer.cend(), [this](const std::unique_ptr<uto::proto::CameraFreespace>& ptr) { return ptr->header().time_meas() > m_fusionTimestampNs - 50 * INT_1E6 && ptr->header().time_meas() < m_fusionTimestampNs + 50 * INT_1E6; });
          auto fsFisheyeBrrIter = std::find_if(m_fsFisheyeBrrBuffer.cbegin(), m_fsFisheyeBrrBuffer.cend(), [this](const std::unique_ptr<uto::proto::CameraFreespace>& ptr) { return ptr->header().time_meas() > m_fusionTimestampNs - 50 * INT_1E6 && ptr->header().time_meas() < m_fusionTimestampNs + 50 * INT_1E6; });
          auto fsFisheyeBcbIter = std::find_if(m_fsFisheyeBcbBuffer.cbegin(), m_fsFisheyeBcbBuffer.cend(), [this](const std::unique_ptr<uto::proto::CameraFreespace>& ptr) { return ptr->header().time_meas() > m_fusionTimestampNs - 50 * INT_1E6 && ptr->header().time_meas() < m_fusionTimestampNs + 50 * INT_1E6; });
#endif
#if FS_CHECK(CFG_USE_ROAD_MODEL)
          auto roadModelIter = std::find_if(m_roadmodelBuffer.cbegin(), m_roadmodelBuffer.cend(), [this](const std::unique_ptr<uto::proto::RoadModel>& ptr) { return ptr->header().time_meas() > m_fusionTimestampNs - 50 * INT_1E6 && ptr->header().time_meas() < m_fusionTimestampNs + 50 * INT_1E6; });
#endif

          if(fsLidarIter != m_lidarFsBuffer.cend())
          {
            fsLidarPtr = std::move(*fsLidarIter);
            while(m_lidarFsBuffer.cbegin() < fsLidarIter + 1)
            {
              m_lidarFsBuffer.popFront();
            }
          }

#if FS_CHECK(CFG_USE_CF_FS)
          if(fsCfIter != m_fsCfBuffer.cend())
          {
            fsCfPtr = std::move(*fsCfIter);
            while(m_fsCfBuffer.cbegin() < fsCfIter + 1)
            {
              m_fsCfBuffer.popFront();
            }
          }
#endif

#if FS_CHECK(CFG_USE_FISHEYE_FS)
          if(fsFisheyeFcfIter != m_fsFisheyeFcfBuffer.cend())
          {
            fsFisheyeFcfPtr = std::move(*fsFisheyeFcfIter);
            while(m_fsFisheyeFcfBuffer.cbegin() < fsFisheyeFcfIter + 1)
            {
              m_fsFisheyeFcfBuffer.popFront();
            }
          }

          if(fsFisheyeFllIter != m_fsFisheyeFllBuffer.cend())
          {
            fsFisheyeFllPtr = std::move(*fsFisheyeFllIter);
            while(m_fsFisheyeFllBuffer.cbegin() < fsFisheyeFllIter + 1)
            {
              m_fsFisheyeFllBuffer.popFront();
            }
          }

          if(fsFisheyeFrrIter != m_fsFisheyeFrrBuffer.cend())
          {
            fsFisheyeFrrPtr = std::move(*fsFisheyeFrrIter);
            while(m_fsFisheyeFrrBuffer.cbegin() < fsFisheyeFrrIter + 1)
            {
              m_fsFisheyeFrrBuffer.popFront();
            }
          }

          if(fsFisheyeBllIter != m_fsFisheyeBllBuffer.cend())
          {
            fsFisheyeBllPtr = std::move(*fsFisheyeBllIter);
            while(m_fsFisheyeBllBuffer.cbegin() < fsFisheyeBllIter + 1)
            {
              m_fsFisheyeBllBuffer.popFront();
            }
          }

          if(fsFisheyeBrrIter != m_fsFisheyeBrrBuffer.cend())
          {
            fsFisheyeBrrPtr = std::move(*fsFisheyeBrrIter);
            while(m_fsFisheyeBrrBuffer.cbegin() < fsFisheyeBrrIter + 1)
            {
              m_fsFisheyeBrrBuffer.popFront();
            }
          }

          if(fsFisheyeBcbIter != m_fsFisheyeBcbBuffer.cend())
          {
            fsFisheyeBcbPtr = std::move(*fsFisheyeBcbIter);
            while(m_fsFisheyeBcbBuffer.cbegin() < fsFisheyeBcbIter + 1)
            {
              m_fsFisheyeBcbBuffer.popFront();
            }
          }
#endif

#if FS_CHECK(CFG_USE_ROAD_MODEL)
          // roadModelPtr = std::move(m_roadModelBuffer.front());
          // m_roadModelBuffer.popFront();
          if(roadModelIter != m_roadmodelBuffer.cend())
          {
            roadModelPtr = std::move(*roadModelIter);
            while(m_roadmodelBuffer.cbegin() < roadModelIter + 1)
            {
              m_roadmodelBuffer.popFront();
            }
          }
#endif

#if FS_CHECK(CFG_VIS_ENABLE)
          auto imageCfIter         = std::find_if(m_imageCfBuffer.cbegin(), m_imageCfBuffer.cend(), [&fsCfPtr](const std::unique_ptr<sensor_msgs::msg::CompressedImage>& ptr) { return fsCfPtr != nullptr && ptr->header.stamp == rclcpp::Time{fsCfPtr->header().time_meas()}; });
          auto imageFisheyeFcfIter = std::find_if(m_imageFisheyeFcfBuffer.cbegin(), m_imageFisheyeFcfBuffer.cend(), [&fsFisheyeFcfPtr](const std::unique_ptr<sensor_msgs::msg::CompressedImage>& ptr) { return fsFisheyeFcfPtr != nullptr && ptr->header.stamp == rclcpp::Time{fsFisheyeFcfPtr->header().time_meas()}; });
          auto imageFisheyeFllIter = std::find_if(m_imageFisheyeFllBuffer.cbegin(), m_imageFisheyeFllBuffer.cend(), [&fsFisheyeFllPtr](const std::unique_ptr<sensor_msgs::msg::CompressedImage>& ptr) { return fsFisheyeFllPtr != nullptr && ptr->header.stamp == rclcpp::Time{fsFisheyeFllPtr->header().time_meas()}; });
          auto imageFisheyeFrrIter = std::find_if(m_imageFisheyeFrrBuffer.cbegin(), m_imageFisheyeFrrBuffer.cend(), [&fsFisheyeFrrPtr](const std::unique_ptr<sensor_msgs::msg::CompressedImage>& ptr) { return fsFisheyeFrrPtr != nullptr && ptr->header.stamp == rclcpp::Time{fsFisheyeFrrPtr->header().time_meas()}; });
          auto imageFisheyeBllIter = std::find_if(m_imageFisheyeBllBuffer.cbegin(), m_imageFisheyeBllBuffer.cend(), [&fsFisheyeBllPtr](const std::unique_ptr<sensor_msgs::msg::CompressedImage>& ptr) { return fsFisheyeBllPtr != nullptr && ptr->header.stamp == rclcpp::Time{fsFisheyeBllPtr->header().time_meas()}; });
          auto imageFisheyeBrrIter = std::find_if(m_imageFisheyeBrrBuffer.cbegin(), m_imageFisheyeBrrBuffer.cend(), [&fsFisheyeBrrPtr](const std::unique_ptr<sensor_msgs::msg::CompressedImage>& ptr) { return fsFisheyeBrrPtr != nullptr && ptr->header.stamp == rclcpp::Time{fsFisheyeBrrPtr->header().time_meas()}; });
          auto imageFisheyeBcbIter = std::find_if(m_imageFisheyeBcbBuffer.cbegin(), m_imageFisheyeBcbBuffer.cend(), [&fsFisheyeBcbPtr](const std::unique_ptr<sensor_msgs::msg::CompressedImage>& ptr) { return fsFisheyeBcbPtr != nullptr && ptr->header.stamp == rclcpp::Time{fsFisheyeBcbPtr->header().time_meas()}; });

          if(imageCfIter != m_imageCfBuffer.cend())
          {
            imageCfPtr = std::move(*imageCfIter);
            while(m_imageCfBuffer.cbegin() < imageCfIter + 1)
            {
              m_imageCfBuffer.popFront();
            }
          }

          if(imageFisheyeFcfIter != m_imageFisheyeFcfBuffer.cend())
          {
            imageFisheyeFcfPtr = std::move(*imageFisheyeFcfIter);
            while(m_imageFisheyeFcfBuffer.cbegin() < imageFisheyeFcfIter + 1)
            {
              m_imageFisheyeFcfBuffer.popFront();
            }
          }

          if(imageFisheyeFllIter != m_imageFisheyeFllBuffer.cend())
          {
            imageFisheyeFllPtr = std::move(*imageFisheyeFllIter);
            while(m_imageFisheyeFllBuffer.cbegin() < imageFisheyeFllIter + 1)
            {
              m_imageFisheyeFllBuffer.popFront();
            }
          }

          if(imageFisheyeFrrIter != m_imageFisheyeFrrBuffer.cend())
          {
            imageFisheyeFrrPtr = std::move(*imageFisheyeFrrIter);
            while(m_imageFisheyeFrrBuffer.cbegin() < imageFisheyeFrrIter + 1)
            {
              m_imageFisheyeFrrBuffer.popFront();
            }
          }

          if(imageFisheyeBllIter != m_imageFisheyeBllBuffer.cend())
          {
            imageFisheyeBllPtr = std::move(*imageFisheyeBllIter);
            while(m_imageFisheyeBllBuffer.cbegin() < imageFisheyeBllIter + 1)
            {
              m_imageFisheyeBllBuffer.popFront();
            }
          }

          if(imageFisheyeBrrIter != m_imageFisheyeBrrBuffer.cend())
          {
            imageFisheyeBrrPtr = std::move(*imageFisheyeBrrIter);
            while(m_imageFisheyeBrrBuffer.cbegin() < imageFisheyeBrrIter + 1)
            {
              m_imageFisheyeBrrBuffer.popFront();
            }
          }

          if(imageFisheyeBcbIter != m_imageFisheyeBcbBuffer.cend())
          {
            imageFisheyeBcbPtr = std::move(*imageFisheyeBcbIter);
            while(m_imageFisheyeBcbBuffer.cbegin() < imageFisheyeBcbIter + 1)
            {
              m_imageFisheyeBcbBuffer.popFront();
            }
          }
#endif
        }
      }
    }
  }

  return validAligned;
}

void fs::FsNode::process()
{
#if FS_CHECK(CFG_LOAD_RECORDING)
  loadBag();
#endif

  Diagnoser& diagnoser = Diagnoser::instance();
  diagnoser.incrementTimeoutCount();

  removeObsoleteInputs();

  std::unique_ptr<uto::proto::PerceptionFreespace>   lidarFsPtr         = nullptr;
  std::unique_ptr<uto::proto::CameraFreespace>       cfFsPtr            = nullptr;
  std::unique_ptr<uto::proto::CameraFreespace>       fsFisheyeFcfPtr    = nullptr;
  std::unique_ptr<uto::proto::CameraFreespace>       fsFisheyeFllPtr    = nullptr;
  std::unique_ptr<uto::proto::CameraFreespace>       fsFisheyeFrrPtr    = nullptr;
  std::unique_ptr<uto::proto::CameraFreespace>       fsFisheyeBcbPtr    = nullptr;
  std::unique_ptr<uto::proto::CameraFreespace>       fsFisheyeBllPtr    = nullptr;
  std::unique_ptr<uto::proto::CameraFreespace>       fsFisheyeBrrPtr    = nullptr;
  std::unique_ptr<sensor_msgs::msg::CompressedImage> imageCfPtr         = nullptr;
  std::unique_ptr<sensor_msgs::msg::CompressedImage> imageFisheyeFcfPtr = nullptr;
  std::unique_ptr<sensor_msgs::msg::CompressedImage> imageFisheyeFllPtr = nullptr;
  std::unique_ptr<sensor_msgs::msg::CompressedImage> imageFisheyeFrrPtr = nullptr;
  std::unique_ptr<sensor_msgs::msg::CompressedImage> imageFisheyeBcbPtr = nullptr;
  std::unique_ptr<sensor_msgs::msg::CompressedImage> imageFisheyeBllPtr = nullptr;
  std::unique_ptr<sensor_msgs::msg::CompressedImage> imageFisheyeBrrPtr = nullptr;
  std::unique_ptr<EgoMotion>                         egoMotionPtr       = nullptr;
  std::unique_ptr<uto::proto::RoadModel>             roadModelPtr       = nullptr;

  const bool validAligned = getAlignedInputs(lidarFsPtr,
                                             cfFsPtr,
                                             fsFisheyeFcfPtr,
                                             fsFisheyeFllPtr,
                                             fsFisheyeFrrPtr,
                                             fsFisheyeBcbPtr,
                                             fsFisheyeBllPtr,
                                             fsFisheyeBrrPtr,
                                             imageCfPtr,
                                             imageFisheyeFcfPtr,
                                             imageFisheyeFllPtr,
                                             imageFisheyeFrrPtr,
                                             imageFisheyeBcbPtr,
                                             imageFisheyeBllPtr,
                                             imageFisheyeBrrPtr,
                                             egoMotionPtr,
                                             roadModelPtr);

  publishDiagnosisMsg();

  Vehicle&       vehicle       = Vehicle::instance();
  CameraManager& cameraManager = CameraManager::instance();

  const bool isAllCameraValid = cameraManager.isAllCameraValid();
  isAllCameraValid ? diagnoser.healCalibCount() : diagnoser.incrementCalibCount();

  if(nullptr != egoMotionPtr)
  {
    vehicle.updateVehicle(*egoMotionPtr);
  }

  if(validAligned && vehicle.isInitialized() && isAllCameraValid)
  {
#if FS_CHECK(CFG_VIS_ENABLE)
    broadcastTF();
    Vis::prepare(m_fusionTimestampNs,
                 imageCfPtr.get(),
                 imageFisheyeFcfPtr.get(),
                 imageFisheyeFllPtr.get(),
                 imageFisheyeFrrPtr.get(),
                 imageFisheyeBcbPtr.get(),
                 imageFisheyeBllPtr.get(),
                 imageFisheyeBrrPtr.get());
#endif

    assert(nullptr != lidarFsPtr);
    diagnoser.healProcessTimeoutCount();

    const auto odIter   = std::find_if(m_odBuffer.crbegin(), m_odBuffer.crend(), [this](const std::unique_ptr<uto::proto::PerceptionObstacles>& ptr) { return ptr->header().time_meas() >= m_fusionTimestampNs - 1 * INT_1E9 && ptr->header().time_meas() <= m_fusionTimestampNs; });
    const auto gateIter = std::find_if(m_gatesBuffer.crbegin(), m_gatesBuffer.crend(), [this](const std::unique_ptr<uto::proto::PerceptionGates>& ptr) { return ptr->header().time_meas() >= m_fusionTimestampNs - 1 * INT_1E9 && ptr->header().time_meas() <= m_fusionTimestampNs; });
    // const auto roadModelIter = std::find_if(m_roadModelBuffer.crbegin(), m_roadModelBuffer.crend(), [this](const std::unique_ptr<uto::proto::RoadModel>& ptr) { return ptr->header().time_meas() >= m_fusionTimestampNs - 1 * INT_1E9 && ptr->header().time_meas() <= m_fusionTimestampNs; });

    const uto::proto::PerceptionObstacles* const odPtr   = m_odBuffer.crend() == odIter ? nullptr : odIter->get();
    const uto::proto::PerceptionGates* const     gatePtr = m_gatesBuffer.crend() == gateIter ? nullptr : gateIter->get();
    // const uto::proto::RoadModel* const           roadModelPtr = roadModelIter.crend() == roadModelIter ? nullptr : roadModelIter->get();

    if(diagnoser.isEgoMotionDefect() || diagnoser.hasPoseInvalid() || diagnoser.hasSensorOutOfRangeError(SensorId::LIDAR))
    {
      m_abnormalCount = ABNORMAL_COUNT_MAX;
    }
    else
    {
      m_abnormalCount = std::max(0, m_abnormalCount - 1);
    }

    if(m_abnormalCount > ABNORMAL_COUNT_THRESH)
    {
      m_freespaceManager.resetGridMap();
      m_freespaceManager.backupMode(m_fusionTimestampNs, *lidarFsPtr);
    }
    else if(m_abnormalCount > 0)
    {
      assert(vehicle.getCurrentTimestamp() == m_fusionTimestampNs / 1000 && " timestamp error");

      m_freespaceManager.backupMode(m_fusionTimestampNs, *lidarFsPtr);
      m_freespaceManager.fuseFreespace(m_fusionTimestampNs,
                                       *lidarFsPtr,
                                       cfFsPtr.get(),
                                       fsFisheyeFcfPtr.get(),
                                       fsFisheyeFllPtr.get(),
                                       fsFisheyeFrrPtr.get(),
                                       fsFisheyeBcbPtr.get(),
                                       fsFisheyeBllPtr.get(),
                                       fsFisheyeBrrPtr.get(),
                                       roadModelPtr.get(),
                                       odPtr,
                                       gatePtr);
    }
    else
    {
      assert(0 == m_abnormalCount);
      assert(vehicle.getCurrentTimestamp() == m_fusionTimestampNs / 1000 && " timestamp error");

      m_freespaceManager.resetPassThroughPoints();
      m_freespaceManager.fuseFreespace(m_fusionTimestampNs,
                                       *lidarFsPtr,
                                       cfFsPtr.get(),
                                       fsFisheyeFcfPtr.get(),
                                       fsFisheyeFllPtr.get(),
                                       fsFisheyeFrrPtr.get(),
                                       fsFisheyeBcbPtr.get(),
                                       fsFisheyeBllPtr.get(),
                                       fsFisheyeBrrPtr.get(),
                                       roadModelPtr.get(),
                                       odPtr,
                                       gatePtr);
    }

#if FS_CHECK(CFG_VIS_ENABLE)
    Vis::drawLidarPointCloud(*lidarFsPtr);
    Vis::drawFusionPointCloud(m_freespaceManager.getGridMap(), m_freespaceManager.getPassThroughPoints());
    Vis::drawRoadModelPointCloud(roadModelPtr.get());
#endif

#if !FS_CHECK(CFG_VIS_DRAW_PERCEPTION_FREESPACE)
    publishFusionFreespace(*lidarFsPtr);
#endif
  }

#if FS_CHECK(CFG_VIS_ENABLE)
  publishVis();
#endif
}

void fs::FsNode::publishDiagnosisMsg()
{
  const Diagnoser& diagnoser = Diagnoser::getDiagnoser();

  auto monitorMsgPtr = std::make_unique<uto::proto::ErrorCode>();
  monitorMsgPtr->mutable_header()->set_time_meas(m_fusionTimestampNs);

  std::stringstream ss;

  // vehicle pose defect 5027
  if(diagnoser.isEgoMotionDefect())
  {
    ss << uto::proto::ErrorCode_Itc_FUSN_FS_VEH_POSE_COM_ERR << " ";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_VEH_POSE_COM_ERR);
  }

  // vehicle pose invalid 5075
  if(diagnoser.hasPoseInvalid())
  {
    ss << "5075: vehicle pose invalid. ";
    // TODO: add vehicle pose invalid error
  }

  // lidar fs defect 5004
  if(diagnoser.isSensorDefect(SensorId::LIDAR))
  {
    ss << uto::proto::ErrorCode_Itc_FUSN_FS_LDR_FS_COM_ERR << " ";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_LDR_FS_COM_ERR);
  }

  //lidar fs delay 5074
  if(diagnoser.hasSensorOutOfRangeError(SensorId::LIDAR))
  {
    // TODO: tangxu, add lidar out of range error
    ss << "5074: lidar fs out of range";
    // tmp: using 5027
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_VEH_POSE_COM_ERR);
  }

#if FS_CHECK(CFG_USE_CF_FS)
  // camera fs defect 5006
  if(diagnoser.isSensorDefect(SensorId::CAMERA_CENTER_FRONT))
  {
    ss << uto::proto::ErrorCode_Itc_FUSN_FS_VSN_FS_COM_ERR << " ";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_VSN_FS_COM_ERR);
  }

  //camera fs delay 5076
  if(diagnoser.hasSensorOutOfRangeError(SensorId::CAMERA_CENTER_FRONT))
  {
    // TODO: tangxu, add cam out of range error
    ss << "5076: camera fs out of range. ";
  }
#endif

#if FS_CHECK(CFG_USE_FISHEYE_FS)
  //fisheye fs defect 5007
  if(diagnoser.isFisheyeDefect())
  {
    ss << uto::proto::ErrorCode_Itc_FUSN_FS_SURRND_VSN_FS_COM_ERR << " ";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_SURRND_VSN_FS_COM_ERR);
  }

  //fisheye fs delay 5077
  if(diagnoser.hasFisheyeOutOfRangeError())
  {
    // TODO: add fisheye out of range error
    ss << "5077: fisheye fs out of range. ";
  }
#endif

  //calib error 5079
  if(diagnoser.hasCalibError())
  {
    // TODO: add calib error
    ss << "5079: calib error. ";
  }

  // map sdk error 5080
  if(diagnoser.hasMapError())
  {
    // TODO: add map error
    ss << "5080: map sdk error. ";
  }

  // mot od defect 5048
  if(diagnoser.isOdDefect())
  {
    ss << uto::proto::ErrorCode_Itc_FUSN_FS_FUSN_MOT_COM_ERR << " ";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_FUSN_MOT_COM_ERR);
  }

  // mot od delay 5078
  if(diagnoser.hasOdOutOfRangeError())
  {
    // TODO: tangxu, add od out of range error
    ss << "5078: od out of range. ";
  }

  //process error 5081
  if(diagnoser.hasProcessError())
  {
    // TODO: tangxu, add process error
    ss << "5081: process error. ";
  }

  if(!ss.str().empty())
  {
    UERROR << "failure detected, error code: " << ss.str();
  }

  m_diagPub.publish(std::move(monitorMsgPtr));
}

void fs::FsNode::resetBuffers()
{
  m_imageCfBuffer.clear();
  m_lidarFsBuffer.clear();
  m_gatesBuffer.clear();
  m_fsCfBuffer.clear();
}

#if FS_CHECK(CFG_VIS_ENABLE)
void fs::FsNode::broadcastTF()
{
  // 2023-01-01 00:00:00
  if(m_fusionTimestampNs > 1672502400000000000)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "world";
    tf.child_frame_id  = "ego";

    const Vehicle&  vehicle    = Vehicle::getVehicle();
    const EVector3& vecEgo2Wrd = vehicle.getVecEgo2Wrd();

    tf.transform.translation.x = vecEgo2Wrd.x();
    tf.transform.translation.y = vecEgo2Wrd.y();
    tf.transform.translation.z = vecEgo2Wrd.z();

    const Eigen::Quaternionf quaternion{vehicle.getRotEgo2Wrd()};
    tf.transform.rotation.x = quaternion.x();
    tf.transform.rotation.y = quaternion.y();
    tf.transform.rotation.z = quaternion.z();
    tf.transform.rotation.w = quaternion.w();

    m_rosWriterPtr->write(tf, "/tf", rclcpp::Time{m_fusionTimestampNs});
  }
}

void fs::FsNode::publishVis()
{
  // 2023-01-01 00:00:00
  if(m_fusionTimestampNs > 1672502400000000000)
  {
    auto header     = std_msgs::msg::Header();
    header.frame_id = "ego";
    header.stamp    = rclcpp::Time(m_fusionTimestampNs);

    sensor_msgs::msg::PointCloud2 lidarPointCloud2;
    sensor_msgs::msg::PointCloud2 fusionPointCloud2;
    sensor_msgs::msg::PointCloud2 passThroughPointCloud2;
    sensor_msgs::msg::PointCloud2 roadModelPointCloud2;
    sensor_msgs::convertPointCloudToPointCloud2(Vis::getLidarPointCloud(), lidarPointCloud2);
    sensor_msgs::convertPointCloudToPointCloud2(Vis::getFusionPointCloud(), fusionPointCloud2);
    sensor_msgs::convertPointCloudToPointCloud2(Vis::getPassThroughPointCloud(), passThroughPointCloud2);
    sensor_msgs::convertPointCloudToPointCloud2(Vis::getRoadModelPointCloud(), roadModelPointCloud2);

    m_rosWriterPtr->write(Vis::getMarkerArray(), VIS_MARKER_TOPIC, header.stamp);
    m_rosWriterPtr->write(lidarPointCloud2, VIS_LIDAR_POINT_CLOUD_TOPIC, header.stamp);
    m_rosWriterPtr->write(fusionPointCloud2, VIS_FUSION_POINT_CLOUD_TOPIC, header.stamp);
    m_rosWriterPtr->write(passThroughPointCloud2, VIS_PASS_THROUGH_POINT_CLOUD_TOPIC, header.stamp);
#if FS_CHECK(CFG_USE_CF_FS)
    m_rosWriterPtr->write(*cv_bridge::CvImage(header, "bgr8", Vis::getCenterFrontView()).toCompressedImageMsg(), VIS_FRONT_VIEW_TOPIC, header.stamp);
#endif
#if FS_CHECK(CFG_USE_ROAD_MODEL)
    m_rosWriterPtr->write(roadModelPointCloud2, VIS_ROAD_MODEL_TOPIC, header.stamp);
#endif
#if FS_CHECK(CFG_USE_FISHEYE_FS)
    m_rosWriterPtr->write(*cv_bridge::CvImage(header, "bgr8", Vis::getFisheyeFcfView()).toCompressedImageMsg(), VIS_FISHEYE_FCF_VIEW_TOPIC, header.stamp);
    m_rosWriterPtr->write(*cv_bridge::CvImage(header, "bgr8", Vis::getFisheyeFllView()).toCompressedImageMsg(), VIS_FISHEYE_FLL_VIEW_TOPIC, header.stamp);
    m_rosWriterPtr->write(*cv_bridge::CvImage(header, "bgr8", Vis::getFisheyeFrrView()).toCompressedImageMsg(), VIS_FISHEYE_FRR_VIEW_TOPIC, header.stamp);
    m_rosWriterPtr->write(*cv_bridge::CvImage(header, "bgr8", Vis::getFisheyeBcbView()).toCompressedImageMsg(), VIS_FISHEYE_BCB_VIEW_TOPIC, header.stamp);
    m_rosWriterPtr->write(*cv_bridge::CvImage(header, "bgr8", Vis::getFisheyeBllView()).toCompressedImageMsg(), VIS_FISHEYE_BLL_VIEW_TOPIC, header.stamp);
    m_rosWriterPtr->write(*cv_bridge::CvImage(header, "bgr8", Vis::getFisheyeBrrView()).toCompressedImageMsg(), VIS_FISHEYE_BRR_VIEW_TOPIC, header.stamp);
#endif
  }
}

void fs::FsNode::publishVisPerceptionFreespace(const uto::proto::PerceptionFreespace& perceptionFreespace)
{
  Vis::drawPerceptionFreespace(perceptionFreespace);
  sensor_msgs::msg::PointCloud2 perceptionPointCloud2;
  sensor_msgs::convertPointCloudToPointCloud2(Vis::getPerceptionFreespace(), perceptionPointCloud2);
  m_rosWriterPtr->write(perceptionPointCloud2, VIS_PERCEPTION_FREESPACE_TOPIC, rclcpp::Time{perceptionFreespace.header().time_meas()});
}
#endif

void fs::FsNode::publishFusionFreespace(const uto::proto::PerceptionFreespace& lidarFsPtr)
{
  auto start = std::chrono::steady_clock::now();

  auto fusionFsPtr = std::make_unique<uto::proto::PerceptionFreespace>();
  fusionFsPtr->mutable_header()->set_time_meas(m_fusionTimestampNs);
  fusionFsPtr->set_length((PUB_RANGE_FRONT + PUB_RANGE_REAR) * GRID_SCALE_INV);
  fusionFsPtr->set_width((2 * PUB_RANGE_LEFT_RIGHT) * GRID_SCALE_INV);
  fusionFsPtr->set_vehicle_origin_length(PUB_RANGE_REAR * GRID_SCALE_INV);
  fusionFsPtr->set_vehicle_origin_width(PUB_RANGE_LEFT_RIGHT * GRID_SCALE_INV);
  fusionFsPtr->set_resolution(GRID_SCALE);
  fusionFsPtr->mutable_origin_point()->set_x(0.0f);
  fusionFsPtr->mutable_origin_point()->set_y(0.0f);
  fusionFsPtr->set_origin_point_theta(0.0f);

  fusionFsPtr->mutable_perception_gridmap()->Reserve(20000);
  const GridMap& gridMap = m_freespaceManager.getGridMap();
  if(gridMap.isToPublish())
  {
    for(int j = PUB_MIN_ROW; j < PUB_MAX_ROW; ++j)
    {
      for(int i = PUB_MIN_COL; i < PUB_MAX_COL; ++i)
      {
        const Grid& grid = gridMap.at(j, i);
        if(grid.isBorder && grid.logit > LOGIT_OCCUPIED_THRESH)
        {
          auto& outputGrid = *fusionFsPtr->add_perception_gridmap();
          outputGrid.set_label(uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED);
          outputGrid.set_move_status(uto::proto::PerceptionGrid_MoveStatus::PerceptionGrid_MoveStatus_STATIONARY);
          auto& position = *outputGrid.mutable_position();
          position.set_x(grid.point.x());
          position.set_y(grid.point.y());
        }
      }
    }
  }

  for(const auto& point : m_freespaceManager.getPassThroughPoints())
  {
    auto& outputGrid = *fusionFsPtr->add_perception_gridmap();
    outputGrid.set_label(uto::proto::PerceptionGrid::Label ::PerceptionGrid_Label_OCCUPIED);
    outputGrid.set_move_status(uto::proto::PerceptionGrid_MoveStatus::PerceptionGrid_MoveStatus_STATIONARY);
    auto& position = *outputGrid.mutable_position();
    position.set_x(point.x());
    position.set_y(point.y());
  }

  fusionFsPtr->mutable_drivable_boundary()->CopyFrom(lidarFsPtr.drivable_boundary()); // db copy

  const size_t gridSize = fusionFsPtr->perception_gridmap_size();
  m_fusionFsPub.publish(std::move(fusionFsPtr));

  const int duration = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
  UWARN_IF(duration > 2) << "================= publish time exceeded limit: " << duration << " ms, grid size: " << gridSize << " =================";
}
