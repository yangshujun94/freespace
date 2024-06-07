#include "macro.h"
#include "switcher.h"
#include "core/expand/expand.h"
#include "peripheral/map_provider/map_provider.h"

#include <common/proto_utils.h>

#if FS_CHECK(CFG_MULTITHREAD)
#include "fs_node_multithread.h"
#else
#include "fs_node_singlethread.h"
#endif
#include "version.h"

#if FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_MCAP_RAW)
#include <filesystem>
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/int32.hpp"
#include "uto/idl/header.hpp"
#endif

extern const char *GIT_COMMIT_SHA1;
extern const char *GIT_COMMIT_DATE;
extern const char *GIT_COMMIT_SUBJECT;
extern const char *CICD_VEHICLE_TYPE;
extern const char *CICD_TARGET_BOARD;
extern const char *CICD_MIDDLEWARE;
extern const char *BUILD_PROJECT_FLAG;
extern const char *BUILD_CMAKE_TYPE;
extern const char *BUILD_VIS_ENABLE;
extern const char *BUILD_LOAD_ENABLE;

#if FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_MCAP_RAW)
// tf消息生成
using namespace std::chrono_literals;
geometry_msgs::msg::TransformStamped generate_transform()
{
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp            = rclcpp::Clock().now();
  transform.header.frame_id         = "ego";
  transform.child_frame_id          = "";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x    = 0.0;
  transform.transform.rotation.y    = 0.0;
  transform.transform.rotation.z    = 0.0;
  transform.transform.rotation.w    = 1.0;
  return transform;
}

rcl_interfaces::msg::Log generate_time_log(const rclcpp::Time &stamp)
{
  rcl_interfaces::msg::Log log_msg;
  log_msg.stamp = stamp;
  log_msg.level = rcl_interfaces::msg::Log::INFO;
  log_msg.name  = "-";

  std::stringstream ts;
  log_msg.msg      = std::to_string(stamp.nanoseconds() * 1e-6);
  log_msg.file     = __FILE__;
  log_msg.function = __FUNCTION__;
  log_msg.line     = __LINE__;
  return log_msg;
}

std::string getOutFilePath(std::string path_out, std::string bag_name)
{
  auto              now           = std::chrono::system_clock::now();
  auto              now_in_time_t = std::chrono::system_clock::to_time_t(now);
  auto              now_in_ms     = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_in_time_t), "%H-%M--%S-") << std::setw(3) << std::setfill('0') << now_in_ms.count();
  std::string time_stamp_str = path_out + bag_name + "-" + ss.str();
  return time_stamp_str;
}
#endif

// -- share function
void fs::FsNode::init()
{
  UERROR << "commit version: " << GIT_COMMIT_SHA1;
  UERROR << "commit date   : " << GIT_COMMIT_DATE;
  UERROR << "commit message: " << GIT_COMMIT_SUBJECT;
  UERROR << "cicd vehicle_type:" << CICD_VEHICLE_TYPE << ",target_board:" << CICD_TARGET_BOARD << ",middleware:" << CICD_MIDDLEWARE;
  UERROR << "build project_flag:" << BUILD_PROJECT_FLAG << ",cmake_type:" << BUILD_CMAKE_TYPE << ",vis:" << BUILD_VIS_ENABLE << ",load_log:" << BUILD_LOAD_ENABLE;
  UERROR << "fs software version: " << MAJOR_VERSION << "." << MINOR_VERSION << "." << PATCH_VERSION;

  std::string strProjectName;
#if FS_CHECK(CFG_ROS2)
  uto::proto::ProjectConfig cfg;
  if(uto::loadProtoTextFile(PROJECT_CONFIG_FILE, &cfg))
  {
    strProjectName = cfg.project_name();
    UERROR << "load parameter 'project_config' success, project_name is " << strProjectName.c_str() << ", license_plate is " << cfg.license_plate().c_str();
  }
  else
  {
    UERROR << "loadProtoBinaryFile failed, use ros args!";
    this->declare_parameter<std::string>("project_name", "QZ");
    this->get_parameter("project_name", strProjectName);
  }
#else
  this->declare_parameter<int8_t>("vehicle_num", -1);
  this->get_parameter("vehicle_num", vehicle_number_);
  UINFO << "the vehicle_num is " << vehicle_number_;

  this->declare_parameter<std::string>("project_name", "WS");
  this->get_parameter("project_name", strProjectName);
#endif

  const ProjectLocation projectLoc = str2ProjectLocation(strProjectName);
  fs::MapProvider::create(projectLoc);
  UINFO << "create map provider: " << PROJECT_LOCATION_TO_STRING.at(projectLoc);

#if FS_CHECK(CFG_ROS2)
#if FS_CHECK(CFG_USE_MCAP_FUSION)
#if FS_CHECK(CFG_LOAD_RECORDING)
  curr_db3_index_ = 0;
  curr_db3_index_ = 0;
  for(const auto &entry : std::filesystem::directory_iterator("../data_input"))
  {
    db3_files_.push_back(entry.path());
    db3_name_prefixes_.push_back(entry.path().filename().string());
  }
  m_bagReaderPtr->open(db3_files_[curr_db3_index_]);
  empty_ = false;
  rosbag2_storage::StorageOptions write_option;
  write_option.uri        = getOutFilePath("../data_output/", db3_name_prefixes_[curr_db3_index_]);
  write_option.storage_id = "mcap";
  m_bagWriterPtr->open(write_option);
#else
  this->declare_parameter<std::string>("path_out", "none");
  this->get_parameter("path_out", path_out_);

  std::string                     bag_name = "test";
  rosbag2_storage::StorageOptions write_option;
  write_option.uri        = getOutFilePath(path_out_, bag_name);
  write_option.storage_id = "mcap";
  this->m_bagWriterPtr->open(write_option);
#endif // -- CFG_LOAD_RECORDING
#endif // -- CFG_USE_MCAP_FUSION

#else
#if !FS_CHECK(CFG_LOAD_RECORDING)
  m_lcmHandler_DEFAULT.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_LIDAR_GRIDMAP.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_TRANSFORM.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_VCU_VEHICLE_INFO.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_LINE_CONTROL.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_VOT.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_BEV_FS.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_LANE.subscribe(".*", &FsNode::lcmReceiverCallback, this);
#if FS_CHECK(CFG_VIS_ENABLE)
  m_lcmHandler_Image_FW.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_Image_FN.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_Image_FL.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_Image_FR.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_Image_RW.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_Image_RN.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_Image_RL.subscribe(".*", &FsNode::lcmReceiverCallback, this);
  m_lcmHandler_Image_RR.subscribe(".*", &FsNode::lcmReceiverCallback, this);
#if FS_CHECK(CFG_DEBUG_PERCEPTION_FREESPACES)
  m_lcmHandler_Planner_Gridmap_AIV.subscribe(".*", &FsNode::lcmReceiverCallback, this);
#endif
#endif
#endif // -- CFG_LOAD_RECORDING
  initBlindArea();
#endif // -- CFG_ROS2
}

#if FS_CHECK(CFG_USE_MCAP_RAW)
void fs::FsNode::createMcap()
{
  this->declare_parameter<std::string>("recording_path", "none");
  this->get_parameter("recording_path", recording_path_);

  this->declare_parameter<std::string>("path_out", "none");
  this->get_parameter("path_out", path_out_);
  // 遍历输入文件夹里的所有文件
  for(const auto &entry : std::filesystem::directory_iterator(recording_path_))
  {
    std::cout << entry.path() << std::endl;
    rosbag2_storage::StorageOptions read_option;
    read_option.uri                        = entry.path();
    read_option.storage_id                 = "sqlite3";
    std::string                   bag_name = entry.path().filename().string();
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.output_serialization_format = "cdr";
    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(read_option, converter_options);
    rosbag2_storage::StorageOptions write_option;
    write_option.uri        = getOutFilePath(path_out_, bag_name);
    write_option.storage_id = "mcap";
    auto write_idl          = std::make_shared<rosbag2_cpp::Writer>();
    write_idl->open(write_option);
    std::cout << entry.path() << std::endl;
    while(reader.has_next())
    {
      auto curr_bag_message = reader.read_next();
      if(curr_bag_message == nullptr)
      {
        break;
      }
      if(curr_bag_message->topic_name == LIDAR_FS_TOPIC)
      {
        uto::idl::SerializedProto                        idl_msg;
        auto                                             serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<uto::idl::SerializedProto> idl_serialization;
        idl_serialization.deserialize_message(&serialized_bag_message, &idl_msg);
        auto msg_det_lidar = std::make_unique<uto::proto::PerceptionFreespace>();
        msg_det_lidar->ParseFromArray(idl_msg.data.data(), static_cast<int>(idl_msg.data.size()));

        rclcpp::Time curr_timestamp = rclcpp::Time(msg_det_lidar->header().time_meas());
        Vehicle     &vehicle        = Vehicle::instance();
        auto         lidar_fs_ptr   = parseLidarFSFromProto(msg_det_lidar.get(), vehicle);
        Vis::drawLidarPointCloud(lidar_fs_ptr.get());
        write_idl->write(Vis::getLidarPointCloud2(), "/fs/lidar_point_cloud", curr_timestamp);
        geometry_msgs::msg::TransformStamped tf_msg = generate_transform();
        write_idl->write(tf_msg, "/tf", curr_timestamp);
      }
#if FS_CHECK(CFG_USE_ROAD_MODEL)
      if(curr_bag_message->topic_name == ROAD_MODEL_TOPIC)
      {
        uto::idl::SerializedProto                        idl_msg;
        auto                                             serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<uto::idl::SerializedProto> idl_serialization;
        idl_serialization.deserialize_message(&serialized_bag_message, &idl_msg);
        auto msg_det_lidar = std::make_unique<uto::proto::RoadModel>();
        msg_det_lidar->ParseFromArray(idl_msg.data.data(), static_cast<int>(idl_msg.data.size()));

        rclcpp::Time curr_timestamp = rclcpp::Time(msg_det_lidar->header().time_meas());
        auto         lidar_fs_ptr   = parseRoadModelFromProto(msg_det_lidar.get());
        Vis::drawRoadModelPointCloud(lidar_fs_ptr.get());
        write_idl->write(Vis::getRoadPointCloud2(), "/fs/road_model_point_cloud", curr_timestamp);
        geometry_msgs::msg::TransformStamped tf_msg = generate_transform();
        write_idl->write(tf_msg, "/tf", curr_timestamp);
      }
#endif
      else if(curr_bag_message->topic_name == IMAGE_FW_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/FW_canvas", curr_timestamp);
      }
      else if(curr_bag_message->topic_name == IMAGE_FN_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/FN_canvas", curr_timestamp);
      }
      else if(curr_bag_message->topic_name == IMAGE_FL_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/FL_canvas", curr_timestamp);
      }

      else if(curr_bag_message->topic_name == IMAGE_FR_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/FR_canvas", curr_timestamp);
      }
      else if(curr_bag_message->topic_name == IMAGE_RW_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/RW_canvas", curr_timestamp);
      }
      else if(curr_bag_message->topic_name == IMAGE_RN_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/RN_canvas", curr_timestamp);
      }
      else if(curr_bag_message->topic_name == IMAGE_RL_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/RL_canvas", curr_timestamp);
      }
      else if(curr_bag_message->topic_name == IMAGE_RR_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/RR_canvas", curr_timestamp);
      }

#if FS_CHECK(CFG_USE_SVC)
      else if(curr_bag_message->topic_name == IMAGE_SFW_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/SFW_canvas", curr_timestamp);
      }
      else if(curr_bag_message->topic_name == IMAGE_SFL_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/SFL_canvas", curr_timestamp);
      }
      else if(curr_bag_message->topic_name == IMAGE_SFR_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/SFR_canvas", curr_timestamp);
      }

      else if(curr_bag_message->topic_name == IMAGE_SRL_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/SRL_canvas", curr_timestamp);
      }
      else if(curr_bag_message->topic_name == IMAGE_SRR_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/SRR_canvas", curr_timestamp);
      }

      else if(curr_bag_message->topic_name == IMAGE_SRW_TOPIC)
      {
        auto                                                     serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> idl_serialization;
        auto                                                     msg_copressed_img = std::make_unique<sensor_msgs::msg::CompressedImage>();
        idl_serialization.deserialize_message(&serialized_bag_message, msg_copressed_img.get());
        rclcpp::Time curr_timestamp_ori = rclcpp::Time(msg_copressed_img->header.stamp);
        int64_t      curr_int64         = curr_timestamp_ori.nanoseconds();
        rclcpp::Time curr_timestamp     = rclcpp::Time(curr_int64);
        write_idl->write(*msg_copressed_img, "/fs/SRW_canvas", curr_timestamp);
      }
#endif
      if(curr_bag_message->topic_name == VOT_OBJECTS_TOPIC)
      {
        Vis::clearMarkers();
        uto::idl::SerializedProto                        idl_msg;
        auto                                             serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<uto::idl::SerializedProto> idl_serialization;
        idl_serialization.deserialize_message(&serialized_bag_message, &idl_msg);
        auto msg_fusion_vot = std::make_unique<uto::proto::PerceptionObstacles>();
        msg_fusion_vot->ParseFromArray(idl_msg.data.data(), static_cast<int>(idl_msg.data.size()));

        rclcpp::Time curr_timestamp = rclcpp::Time(msg_fusion_vot->header().time_meas());

        auto vot_object_ptr = parseVotObjects(msg_fusion_vot.get());
        Vis::drawTrajectory();
        Vis::drawEgo();
        Vis::drawVotObject(vot_object_ptr.get(), "raw", 0);
        Vis::drawVotObject(vot_object_ptr.get(), "expand", 0);
        Vis::drawVotObject(vot_object_ptr.get(), "", 0);
        write_idl->write(Vis::getMarkerArray(), "/fs/marker_array_canvas", curr_timestamp);

        geometry_msgs::msg::TransformStamped tf_msg = generate_transform();
        write_idl->write(tf_msg, "/tf", curr_timestamp);
      }

      else if(curr_bag_message->topic_name == VEHICLE_POSE_TOPIC)
      {
        uto::idl::SerializedProto                        idl_msg;
        auto                                             serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<uto::idl::SerializedProto> idl_serialization;
        idl_serialization.deserialize_message(&serialized_bag_message, &idl_msg);
        auto msg_vehicle_pose = std::make_unique<uto::proto::VehiclePose>();
        subscribeVehiclePose(std::move(msg_vehicle_pose));
      }
      else if(curr_bag_message->topic_name == FS_TOPIC)
      {
        uto::idl::SerializedProto                        idl_msg;
        auto                                             serialized_bag_message = rclcpp::SerializedMessage(*curr_bag_message->serialized_data);
        rclcpp::Serialization<uto::idl::SerializedProto> idl_serialization;
        idl_serialization.deserialize_message(&serialized_bag_message, &idl_msg);
        auto msg_fusion_fs = std::make_unique<uto::proto::PerceptionFreespace>();
        msg_fusion_fs->ParseFromArray(idl_msg.data.data(), idl_msg.data.size());
        rclcpp::Time curr_timestamp = rclcpp::Time(msg_fusion_fs->header().time_meas());
        subscribeFsLidarFromBag(std::move(msg_fusion_fs));

        for(auto &pub : Vis::getSelfReceivePointCloud2())
        {
          if(pub.first == fs::GridLabel::OTHER)
          {
            write_idl->write(pub.second, "/fs/cld_self_receive_OTHER", curr_timestamp);
          }
          if(pub.first == fs::GridLabel::PEDESTRIAN)
          {
            write_idl->write(pub.second, "/fs/cld_self_receive_PEDESTRIAN", curr_timestamp);
          }
          if(pub.first == fs::GridLabel::CYCLIST)
          {
            write_idl->write(pub.second, "/fs/cld_self_receive_CYCLIST", curr_timestamp);
          }
          if(pub.first == fs::GridLabel::VEHICLE)
          {
            write_idl->write(pub.second, "/fs/cld_self_receive_VEHICLE", curr_timestamp);
          }
          if(pub.first == fs::GridLabel::RTG_QC)
          {
            write_idl->write(pub.second, "/fs/cld_self_receive_RTG_QC", curr_timestamp);
          }
          if(pub.first == fs::GridLabel::LOCK_BOX)
          {
            write_idl->write(pub.second, "/fs/cld_self_receive_LOCK_BOX", curr_timestamp);
          }
          if(pub.first == fs::GridLabel::CONE)
          {
            write_idl->write(pub.second, "/fs/cld_self_receive_CONE", curr_timestamp);
          }
          if(pub.first == fs::GridLabel::AIV)
          {
            write_idl->write(pub.second, "/fs/cld_self_receive_AIV", curr_timestamp);
          }
          if(pub.first == fs::GridLabel::TREE_BRANCH)
          {
            write_idl->write(pub.second, "/fs/cld_self_receive_TREE_BRANCH", curr_timestamp);
          }
        }
      }
    }
  }
}
#endif // -- CFG_USE_MCAP_RAW

fs::LocFlag fs::FsNode::getLocFlag()
{
  auto guard = rw_lock_.read_guard();
  return loc_flag_;
}

#if FS_CHECK(CFG_ROS2)
void fs::FsNode::process()
{
#if FS_CHECK(CFG_LOAD_RECORDING)
  loadBag();
  if(empty_)
  {
    m_bagReaderPtr->close();
    fusion_timestamp_     = 0;
    loc_valid_first_time_ = 0;
    lidar_count_          = 0;
    init_x_               = 0.;
    init_y_               = 0.;
    init_z_               = 0.;
    init_x_align_         = 0.;
    init_y_align_         = 0.;
    init_z_align_         = 0.;
    init_flag_            = false;
    init_flag_align_      = false;
    resetBuffers();
    Vehicle::instance().reset();
    freespace_manager_.resetTptsMap();
    freespace_manager_.resetFusionMap();
    freespace_manager_.setFusionMapResetFlag(true);
    curr_db3_index_++;
    if(curr_db3_index_ >= db3_files_.size())
    {
      UINFO << "All db3 files have been processed, exit!";
      rclcpp::shutdown();
    }
    m_bagReaderPtr->open(db3_files_[curr_db3_index_]);
    rosbag2_storage::StorageOptions option;
    option.uri        = getOutFilePath("../data_output/", db3_name_prefixes_[curr_db3_index_]);
    option.storage_id = "mcap";
    auto write_idl    = std::make_shared<rosbag2_cpp::Writer>();
    write_idl->open(option);
    m_bagWriterPtr = write_idl;
    empty_         = false;
  }
#endif

#if FS_CHECK(CFG_USE_MCAP_RAW)
  createMcap();
  rclcpp::shutdown();
#endif
  std::unique_ptr<uto::proto::PerceptionFreespace> fs_lidar_proto_ptr = nullptr;
  std::unique_ptr<uto::proto::PerceptionGates>     gates_proto_ptr    = nullptr;

  std::unique_ptr<EgoMotion>                       emo_pos_ptr          = nullptr;
  std::unique_ptr<uto::proto::PerceptionObstacles> vot_object_proto_ptr = nullptr;
#if FS_CHECK(CFG_USE_ROAD_MODEL)
  std::unique_ptr<uto::proto::RoadModel> road_model_proto_ptr = nullptr;
#endif

  std::array<CamFS, (uint8_t)SensorId::MAX_SENSOR_NUM> array_fs{};

#if FS_CHECK(CFG_VIS_ENABLE)
  std::array<std::unique_ptr<sensor_msgs::msg::CompressedImage>, (uint8_t)SensorId::MAX_SENSOR_NUM> array_img{};
  {
    // clang-format off
    auto guard = rw_lock_.write_guard();
    if(!img_fw_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_FW] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_fw_buffer_.back());}
    // if(!img_fn_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_FN] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_fn_buffer_.back());}
    // if(!img_fl_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_FL] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_fl_buffer_.back());}
    // if(!img_fr_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_FR] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_fr_buffer_.back());}
    // if(!img_rw_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_RW] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_rw_buffer_.back());}
    // if(!img_rn_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_RN] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_rn_buffer_.back());}
    // if(!img_rl_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_RL] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_rl_buffer_.back());}
    // if(!img_rr_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_RR] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_rr_buffer_.back());}
#if FS_CHECK(CFG_USE_SVC)
  if(!img_sfw_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_SFW] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_sfw_buffer_.back());}
  if(!img_sfl_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_SFL] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_sfl_buffer_.back());}
  if(!img_sfr_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_SFR] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_sfr_buffer_.back());}
  if(!img_srl_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_SRL] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_srl_buffer_.back());}
  if(!img_srr_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_SRR] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_srr_buffer_.back());}
  if(!img_srw_buffer_.empty()){array_img[(uint8_t)SensorId::CAMERA_SRW] = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_srw_buffer_.back());}
#endif
    // clang-format on
  }
  getAlignedInputs(fs_lidar_proto_ptr,
                   array_fs,
                   gates_proto_ptr,
                   array_img,
                   emo_pos_ptr,
#if FS_CHECK(CFG_USE_ROAD_MODEL)
                   road_model_proto_ptr,
#endif
                   vot_object_proto_ptr);

#else
  getAlignedInputs(fs_lidar_proto_ptr,
                   array_fs,
                   gates_proto_ptr,
                   emo_pos_ptr,
                   vot_object_proto_ptr);
#endif
  Vehicle       &vehicle       = Vehicle::instance();
  CameraManager &cameraManager = CameraManager::instance();
  if(nullptr != emo_pos_ptr)
  {
    vehicle.updateVehicle(emo_pos_ptr.get());
  }
  if(!fs::MapProvider::getMapProvider().isInit())
  {
    fs::MapProvider::getMapProvider().transformLLA2LocalXTM(init_x_, init_y_, init_z_, init_flag_);
  }
  //  if(validAligned && vehicle.isValid() && fs_lidar_proto_ptr != nullptr && cameraManager.isAllCameraValid())
  // UINFO  << "vehicle is " << vehicle.isValid() << " " << (fs_lidar_proto_ptr == nullptr) << " " << cameraManager.isAllCameraValid();
  if(vehicle.isValid() && fs_lidar_proto_ptr != nullptr && cameraManager.isAllCameraValid())
  {
    auto lidar_fs_ptr   = parseLidarFSFromProto(fs_lidar_proto_ptr.get(), vehicle);
    auto vot_object_ptr = parseVotObjects(vot_object_proto_ptr.get());
#if FS_CHECK(CFG_USE_ROAD_MODEL)
    auto road_model_ptr = parseRoadModelFromProto(road_model_proto_ptr.get());
    refineWithRoadModel(lidar_fs_ptr.get(), road_model_ptr);
#endif

#if FS_CHECK(CFG_AIV_ELECTRIC_FENCE) || FS_CHECK(CFG_AIV_LOCK_STATION_PUSH)
    auto coeff_lanes = getAllLaneCoeff();
#else
    std::vector<std::vector<fs::LineCoeff>> coeff_lanes{};
#endif // -- CFG_AIV_ELECTRIC_FENCE || CFG_AIV_LOCK_STATION_PUSH

    // -- emo of align_pos at fusion_timestamp_, used to publish enu
    std::unique_ptr<EgoMotion> emo_align_pos_ptr = nullptr;
    getAlignedEgoMotion(fusion_timestamp_, emo_align_pos_ptr, emo_align_pos_buffer_);

#if FS_CHECK(CFG_VIS_ENABLE)
    Vis::prepare(fusion_timestamp_,
                 array_img,
                 array_fs,
                 vot_object_ptr.get(),
                 lidar_fs_ptr.get(),
#if FS_CHECK(CFG_USE_ROAD_MODEL)
                 road_model_ptr.get(),
#endif
                 //                 parseCameraFreespace(fs_FW_proto_ptr.get()).get(),
                 //                 T_fw_2_lidar,
                 coeff_lanes);
#endif
    // UINFO << " array_fs.size() " << array_fs.size();
#if FS_CHECK(CFG_USE_CAM_FISH_FS) || FS_CHECK(CFG_USE_CAM_FW_FS)
    for(size_t i = 0; i < array_fs.size(); ++i)
    {
      removeNoise(std::move(array_fs[i].points_ptr), lidar_fs_ptr.get(), SensorId(i), array_fs[i].T_cam2lidar); // -- modify noise --> UNKNOWN
    }
#endif

    double curr_time = fusion_timestamp_ * 1e-6;
    removeGate(gates_proto_ptr.get(), lidar_fs_ptr.get(), emo_pos_ptr.get());
    // -- modify gates around fs-point --> UNKNOWN
    refineVotObject(lidar_fs_ptr.get(), vot_object_ptr.get());

    const auto loc_flag        = getLocFlag();
    const auto is_lock_station = isLockStation(vot_object_ptr);
    const auto in_ele_fence    = fs::MapProvider::getMapProvider().isInElectricFence(Vehicle::instance().getVecEgo2Wrd());

    std::unique_ptr<uto::proto::PerceptionFreespace> fusion_fs_ptr = std::make_unique<uto::proto::PerceptionFreespace>();
    if(loc_flag == LocFlag::VALID)
    {
      freespace_manager_.fuseFreespace(fusion_timestamp_, lidar_fs_ptr.get(), vot_object_ptr.get(), fusion_fs_ptr, emo_align_pos_ptr.get(), emo_pos_ptr.get(), fs_lidar_proto_ptr, coeff_lanes, loc_flag, is_lock_station, in_ele_fence);
      freespace_manager_.setFusionMapResetFlag(false);
    }
    else if(loc_flag == LocFlag::LOW_VALID)
    {
      freespace_manager_.fuseFreespace(fusion_timestamp_, lidar_fs_ptr.get(), vot_object_ptr.get(), fusion_fs_ptr, emo_align_pos_ptr.get(), emo_pos_ptr.get(), fs_lidar_proto_ptr, coeff_lanes, loc_flag, is_lock_station, in_ele_fence);
      freespace_manager_.setFusionMapResetFlag(false);
      freespace_manager_.tptsFreespace(fusion_timestamp_, vot_object_ptr.get(), fusion_fs_ptr, fs_lidar_proto_ptr);
      freespace_manager_.resetTptsMap();
    }
    else
    {
      freespace_manager_.tptsFreespace(fusion_timestamp_, vot_object_ptr.get(), fusion_fs_ptr, fs_lidar_proto_ptr);
      freespace_manager_.resetTptsMap();
      if(!freespace_manager_.getFusionMapResetFlag())
      {
        freespace_manager_.resetFusionMap();
        freespace_manager_.setFusionMapResetFlag(true);
      }
    }

#if FS_CHECK(!CFG_DEBUG_PERCEPTION_FREESPACES)
    auto pub = std::chrono::steady_clock::now();
    fs_enu_pub_ptr_.publish(std::move(fusion_fs_ptr));
    UINFO << "pub: " << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - pub).count();
#endif
  }

  publishMonitorMsg();

#if FS_CHECK(CFG_VIS_ENABLE)

  publishVis();

#endif
}

#if FS_CHECK(CFG_VIS_ENABLE)
void fs::FsNode::getAlignedInputs(std::unique_ptr<uto::proto::PerceptionFreespace> &fs_lidar_proto_ptr,
                                  //                                  std::unique_ptr<uto::proto::CameraFreespace>       &fs_FW_proto_ptr,
                                  std::array<CamFS, (uint8_t)SensorId::MAX_SENSOR_NUM> &array_fs,
                                  std::unique_ptr<uto::proto::PerceptionGates>         &gates_proto_ptr,
                                  //                                  std::unique_ptr<sensor_msgs::msg::CompressedImage> &image_FW_ros_ptr,
                                  std::array<std::unique_ptr<sensor_msgs::msg::CompressedImage>, (uint8_t)SensorId::MAX_SENSOR_NUM> &array_img,
                                  std::unique_ptr<EgoMotion>                                                                        &emo_pos_ptr,
#if FS_CHECK(CFG_USE_ROAD_MODEL)
                                  std::unique_ptr<uto::proto::RoadModel> &road_model_proto_ptr,
#endif
                                  std::unique_ptr<uto::proto::PerceptionObstacles> &vot_object_proto_ptr)
#else
void fs::FsNode::getAlignedInputs(std::unique_ptr<uto::proto::PerceptionFreespace> &fs_lidar_proto_ptr,
                                  std::array<CamFS, (uint8_t)SensorId::MAX_SENSOR_NUM> &array_fs,
                                  std::unique_ptr<uto::proto::PerceptionGates> &gates_proto_ptr,
                                  std::unique_ptr<EgoMotion> &emo_pos_ptr,
#if FS_CHECK(CFG_USE_ROAD_MODEL)
                                  std::unique_ptr<uto::proto::RoadModel> &road_model_proto_ptr,
#endif
                                  std::unique_ptr<uto::proto::PerceptionObstacles> &vot_object_proto_ptr)
#endif
{
  auto guard = rw_lock_.write_guard();
  Diagnoser::instance().updateCount();

  // -- loop reset buffer
  if(!emo_pos_buffer_.empty())
  {
    if(emo_pos_buffer_.back().timestamp < emo_pos_buffer_.front().timestamp)
    {
      resetBuffers();
    }
  }

  if(fs_lidar_buffer_.size() > SYNC_MAX_LIDAR_BUFFER_SIZE)
  {
    fusion_timestamp_  = fs_lidar_buffer_.front()->header().time_meas();
    fs_lidar_proto_ptr = std::move(fs_lidar_buffer_.front());
    fs_lidar_buffer_.popFront();

#if FS_CHECK(CFG_USE_ROAD_MODEL)
    road_model_proto_ptr = std::move(road_model_buffer_.front());
    road_model_buffer_.popFront();
#endif

    // -- sync emo
    const auto emoRhsIter = std::find_if(emo_pos_buffer_.begin(), emo_pos_buffer_.end(), [this](const EgoMotion &emo) { return static_cast<int64_t>(emo.timestamp) >= fusion_timestamp_; });
    const auto emoLhsIter = std::find_if(emo_pos_buffer_.rbegin(), emo_pos_buffer_.rend(), [this](const EgoMotion &emo) { return static_cast<int64_t>(emo.timestamp) <= fusion_timestamp_; });
    if(emoLhsIter == emo_pos_buffer_.rend() || emoRhsIter == emo_pos_buffer_.end())
    {
      loc_flag_ = LocFlag::INVALID;
      //        UERROR << "herin";
    }
    else
    {
      if(emoLhsIter->is_valid && emoRhsIter->is_valid)
      {
        emo_pos_ptr = std::make_unique<EgoMotion>(interpolateEgoMotion(fusion_timestamp_, *emoLhsIter, *emoRhsIter));
      }
    }

    // -- sync gate
    gates_proto_ptr = cloneGates(gates_buffer_.front());

    // -- sync vot
    const auto votRhsIter = std::find_if(vot_object_buffer_.begin(), vot_object_buffer_.end(), [this](const std::unique_ptr<uto::proto::PerceptionObstacles> &vot) { return static_cast<int64_t>(vot->header().time_meas()) >= fusion_timestamp_; });
    const auto votLhsIter = std::find_if(vot_object_buffer_.rbegin(), vot_object_buffer_.rend(), [this](const std::unique_ptr<uto::proto::PerceptionObstacles> &vot) { return static_cast<int64_t>(vot->header().time_meas()) <= fusion_timestamp_; });
    if(votLhsIter == vot_object_buffer_.rend() || votRhsIter == vot_object_buffer_.end())
    {
      if(!vot_object_buffer_.empty() && std::abs(fusion_timestamp_ - vot_object_buffer_.back()->header().time_meas()) < DELAY_MS * TIME_SCALE_MS)
      {
        vot_object_proto_ptr = cloneVotObjects(vot_object_buffer_.back());
      }
    } // -- can't find
    else
    {
      if(std::abs(votRhsIter->get()->header().time_meas() - fusion_timestamp_) > std::abs(votLhsIter->get()->header().time_meas() - fusion_timestamp_))
      {
        vot_object_proto_ptr = cloneVotObjects(*votLhsIter);
      }
      else
      {
        vot_object_proto_ptr = cloneVotObjects(*votRhsIter);
      }
    } // select min err
//#if FS_CHECK(CFG_USE_CAM_FW_FS)
//    syncFsTF(array_fs[(uint8_t)SensorId::CAMERA_FW], fs_fw_buffer_, emo_pos_buffer_, 50, 50);
//#endif
#if FS_CHECK(CFG_USE_CAM_FISH_FS)
    syncFsTF(array_fs[(uint8_t)SensorId::CAMERA_SFW], fs_sfw_buffer_, emo_pos_buffer_, 150, 150);
    syncFsTF(array_fs[(uint8_t)SensorId::CAMERA_SFL], fs_sfl_buffer_, emo_pos_buffer_, 150, 150);
    syncFsTF(array_fs[(uint8_t)SensorId::CAMERA_SFR], fs_sfr_buffer_, emo_pos_buffer_, 150, 150);
    syncFsTF(array_fs[(uint8_t)SensorId::CAMERA_SRL], fs_srl_buffer_, emo_pos_buffer_, 150, 150);
    syncFsTF(array_fs[(uint8_t)SensorId::CAMERA_SRR], fs_srr_buffer_, emo_pos_buffer_, 150, 150);
    syncFsTF(array_fs[(uint8_t)SensorId::CAMERA_SRW], fs_srw_buffer_, emo_pos_buffer_, 150, 150);
#endif

#if FS_CHECK(CFG_VIS_ENABLE)
    //    syncImg(array_img[(uint8_t)SensorId::CAMERA_FW], array_fs[(uint8_t)SensorId::CAMERA_FW], img_fw_buffer_);

#if FS_CHECK(CFG_USE_SVC)
     syncImg(array_img[(uint8_t)SensorId::CAMERA_SFW], array_fs[(uint8_t)SensorId::CAMERA_SFW], img_sfw_buffer_);
     syncImg(array_img[(uint8_t)SensorId::CAMERA_SFL], array_fs[(uint8_t)SensorId::CAMERA_SFL], img_sfl_buffer_);
     syncImg(array_img[(uint8_t)SensorId::CAMERA_SFR], array_fs[(uint8_t)SensorId::CAMERA_SFR], img_sfr_buffer_);
     syncImg(array_img[(uint8_t)SensorId::CAMERA_SRL], array_fs[(uint8_t)SensorId::CAMERA_SRL], img_srl_buffer_);
     syncImg(array_img[(uint8_t)SensorId::CAMERA_SRR], array_fs[(uint8_t)SensorId::CAMERA_SRR], img_srr_buffer_);
     syncImg(array_img[(uint8_t)SensorId::CAMERA_SRW], array_fs[(uint8_t)SensorId::CAMERA_SRW], img_srw_buffer_);
#endif
#endif
  }
}

void fs::FsNode::resetBuffers()
{
#if FS_CHECK(CFG_VIS_ENABLE)
  img_fw_buffer_.clear();
  img_fn_buffer_.clear();
  img_fl_buffer_.clear();
  img_fr_buffer_.clear();
  img_rw_buffer_.clear();
  img_rn_buffer_.clear();
  img_rl_buffer_.clear();
  img_rr_buffer_.clear();
#if FS_CHECK(CFG_USE_SVC)
  img_sfw_buffer_.clear();
  img_sfl_buffer_.clear();
  img_sfr_buffer_.clear();
  img_srl_buffer_.clear();
  img_srr_buffer_.clear();
  img_srw_buffer_.clear();
#endif

#endif
  emo_pos_buffer_.clear();
  emo_align_pos_buffer_.clear();
  fs_lidar_buffer_.clear();
  gates_buffer_.clear();
  fs_fw_buffer_.clear();
  fs_fn_buffer_.clear();
  vot_object_buffer_.clear();
}

std::unique_ptr<uto::proto::PerceptionObstacles> fs::FsNode::cloneVotObjects(std::unique_ptr<uto::proto::PerceptionObstacles> &vot_object_ptr)
{
  if(vot_object_ptr == nullptr)
  {
    return nullptr;
  }
  uto::proto::PerceptionObstacles vot_obj{};
  vot_obj.CopyFrom(*vot_object_ptr.get());
  return std::make_unique<uto::proto::PerceptionObstacles>(vot_obj);
}

std::unique_ptr<uto::proto::PerceptionGates> fs::FsNode::cloneGates(std::unique_ptr<uto::proto::PerceptionGates> &gates_ptr)
{
  if(gates_ptr == nullptr)
  {
    return nullptr;
  }
  uto::proto::PerceptionGates gates_obj{};
  gates_obj.CopyFrom(*gates_ptr.get());
  return std::make_unique<uto::proto::PerceptionGates>(gates_obj);
}

void fs::FsNode::processVehiclePose(const uto::proto::VehiclePose &vehicle_pose_proto)
{
  auto guard = rw_lock_.write_guard();

  if(vehicle_pose_proto.status() == uto::proto::VehiclePose::INVALID)
  {
    UERROR << "loc invalid";
    loc_flag_             = LocFlag::INVALID;
    loc_valid_first_time_ = 0;
    lidar_count_          = 0; // -- clear lidar count， prevent overflow
  }
  else
  {
    // -- set loc status
    if(loc_valid_first_time_ == 0)
    {
      loc_valid_first_time_ = vehicle_pose_proto.header().time_meas();
    }
    else
    {
      // -- 里程计连续有效300ms，且lidar在这段时间内有3帧结果，认为可以发
      loc_flag_ = (vehicle_pose_proto.header().time_meas() - loc_valid_first_time_ > 1000 * TIME_SCALE_MS) && lidar_count_ > 2 ? LocFlag::VALID : LocFlag::LOW_VALID;
    }
  }

  // -- save valid odo data
  {
    if(!init_flag_)
    {
      init_x_    = vehicle_pose_proto.pos().x();
      init_y_    = vehicle_pose_proto.pos().y();
      init_z_    = vehicle_pose_proto.pos().z();
      init_flag_ = true;
    }

    EgoMotion &egoMotion      = *emo_pos_buffer_.pushBackForce();
    egoMotion.timestamp       = vehicle_pose_proto.header().time_meas();
    egoMotion.init_x          = init_x_;
    egoMotion.init_y          = init_y_;
    egoMotion.init_z          = init_z_;
    egoMotion.translation.x() = vehicle_pose_proto.pos().x() - init_x_;
    egoMotion.translation.y() = vehicle_pose_proto.pos().y() - init_y_;
    egoMotion.translation.z() = vehicle_pose_proto.pos().z() - init_z_;
    egoMotion.yaw             = vehicle_pose_proto.attitude_rpy().z();
    egoMotion.pitch           = vehicle_pose_proto.attitude_rpy().y();
    egoMotion.roll            = vehicle_pose_proto.attitude_rpy().x();
    egoMotion.is_valid        = vehicle_pose_proto.status() == uto::proto::VehiclePose::INVALID ? false : true;
  }
  {
    if(!init_flag_align_)
    {
      init_x_align_    = vehicle_pose_proto.align_pos().x();
      init_y_align_    = vehicle_pose_proto.align_pos().y();
      init_z_align_    = vehicle_pose_proto.align_pos().z();
      init_flag_align_ = true;
    }

    EgoMotion &egoMotion      = *emo_align_pos_buffer_.pushBackForce();
    egoMotion.timestamp       = vehicle_pose_proto.header().time_meas();
    egoMotion.init_x          = init_x_align_;
    egoMotion.init_y          = init_y_align_;
    egoMotion.init_z          = init_z_align_;
    egoMotion.translation.x() = vehicle_pose_proto.align_pos().x() - init_x_align_;
    egoMotion.translation.y() = vehicle_pose_proto.align_pos().y() - init_y_align_;
    egoMotion.translation.z() = vehicle_pose_proto.align_pos().z() - init_z_align_;
    egoMotion.yaw             = vehicle_pose_proto.align_attitude_rpy().z();
    egoMotion.pitch           = vehicle_pose_proto.align_attitude_rpy().y();
    egoMotion.roll            = vehicle_pose_proto.align_attitude_rpy().x();
    egoMotion.is_valid        = vehicle_pose_proto.align_status() == uto::proto::VehiclePose::INVALID ? false : true;
  }
}

#if FS_CHECK(CFG_USE_CAM_FISH_FS) || FS_CHECK(CFG_USE_CAM_FW_FS)
void fs::FsNode::removeNoise(const std::unique_ptr<std::vector<fs::CamFSPoint>> cam_fs, LidarFS *lidar_fs_ptr, const SensorId &sensor_id, const FSMat4x4 &Tcl)
{
  if(cam_fs == nullptr || lidar_fs_ptr == nullptr)
  {
    return;
  }
  int idx = static_cast<int>(sensor_id);
  getRemoveNoiseFreespace3(lidar_fs_ptr, cam_fs.get(), idx, CameraManager::instance().getCamera(sensor_id).get(), Tcl);
}
#endif

void fs::FsNode::removeGate(const uto::proto::PerceptionGates *gates_proto_ptr, LidarFS *lidar_fs_ptr, const EgoMotion *ego_Twb_ptr)
{
  if(gates_proto_ptr == nullptr || ego_Twb_ptr == nullptr)
  {
    return;
  }
  GatesInfoVec gates_ego_vec;
  for(int i = 0; i < gates_proto_ptr->gates_size(); ++i)
  {
    if(gates_proto_ptr->gates(i).gate_status() != 3)
    {
      auto gate_ego_info_ptr           = std::make_shared<GateInfo>();
      gate_ego_info_ptr->timestamp     = gates_proto_ptr->header().time_meas();
      gate_ego_info_ptr->request_id    = gates_proto_ptr->gates(i).request_id();
      gate_ego_info_ptr->gate_state    = gates_proto_ptr->gates(i).gate_status();
      gate_ego_info_ptr->gate_cx       = gates_proto_ptr->gates(i).position().x();
      gate_ego_info_ptr->gate_cy       = gates_proto_ptr->gates(i).position().y();
      gate_ego_info_ptr->gate_cz       = gates_proto_ptr->gates(i).position().z();
      gate_ego_info_ptr->stop_distance = gates_proto_ptr->gates(i).stop_distance();
      gates_ego_vec.emplace_back(gate_ego_info_ptr);
    }
  }

  GatesInfoVec gates_info_vec{};
  const auto   Tbw = Utils::inverse(Utils::makeTFrom6Dof(ego_Twb_ptr->yaw, 0., 0., ego_Twb_ptr->translation.x(), ego_Twb_ptr->translation.y(), 0.));

  for(const auto &gate_ego_info_ptr : gates_ego_vec)
  {
    std::unique_ptr<EgoMotion> egoMotionPtr = nullptr;
    if(getAlignedEgoMotion(gate_ego_info_ptr->timestamp, egoMotionPtr, emo_pos_buffer_) == 1)
    {
      const auto Twg = Utils::makeTFrom6Dof(egoMotionPtr->yaw, 0., 0., egoMotionPtr->translation.x(), egoMotionPtr->translation.y(), 0.);
      FSVec3f    base_gate{gate_ego_info_ptr->gate_cx, gate_ego_info_ptr->gate_cy, 0.f};
      FSVec3f    base = (Tbw * Twg * HOMO4(base_gate)).topRows(3);

      GateInfoPtr gate_aligned_ego_ptr    = std::make_shared<GateInfo>();
      gate_aligned_ego_ptr->timestamp     = gate_ego_info_ptr->timestamp;
      gate_aligned_ego_ptr->request_id    = gate_ego_info_ptr->request_id;
      gate_aligned_ego_ptr->gate_state    = gate_ego_info_ptr->gate_state;
      gate_aligned_ego_ptr->stop_distance = gate_ego_info_ptr->stop_distance;
      gate_aligned_ego_ptr->gate_cx       = base[0];
      gate_aligned_ego_ptr->gate_cy       = base[1];
      gates_info_vec.emplace_back(gate_aligned_ego_ptr);
    }
  }
  std::vector<fs::Rect> gates_bbox{};
  updateGatePoints(gates_info_vec, gates_bbox);
  modifyErrorGates(gates_bbox, lidar_fs_ptr);
#if FS_CHECK(CFG_VIS_ENABLE)
  Vis::drawGates(gates_bbox);
#endif
}

std::unique_ptr<std::vector<fs::CamFSPoint>> fs::FsNode::parseCameraFreespace(const uto::proto::CameraFreespace *camera_fs_proto)
{
  if(camera_fs_proto == nullptr)
  {
    return nullptr;
  }
  std::vector<CamFSPoint> camera_freespace(camera_fs_proto->image_width());
  for(int j = 0; j < camera_fs_proto->camera_freespace_points().size(); ++j)
  {
    auto point = camera_fs_proto->camera_freespace_points()[j].point();
    if(point.x() < 0 || point.x() >= (camera_fs_proto->image_width() - 10) || point.y() < 0 || point.y() >= (camera_fs_proto->image_height() - 10))
    {
      continue;
    }

    camera_freespace[static_cast<int>(point.x())] = CamFSPoint(point.y(), [&](int value) -> CameraFSType {
      if(value >= 0 && value <= 9)
      {
        return static_cast<CameraFSType>(value);
      }
      return CameraFSType::UNINIT;
    }(camera_fs_proto->camera_freespace_points()[j].type()));
  };

  int offset = 30;
  int new_h  = camera_fs_proto->image_height() - offset;
  for(size_t j = 1; j < camera_freespace.size(); ++j)
  {
    if(camera_freespace[j].y == 0)
    {
      for(int k = 0; k < 3; ++k)
      {
        if(camera_freespace[j - k].y != 0)
        {
          camera_freespace[j].y = camera_freespace[j - k].y;
          if(camera_freespace[j].y > new_h)
          {
            camera_freespace[j].type = CameraFSType::INVALID;
          }
          else
          {
            camera_freespace[j].type = camera_freespace[j - k].type;
          }
          break;
        }
      } // -- left
    }
    if(camera_freespace[j].y > new_h)
    {
      camera_freespace[j].type = CameraFSType::INVALID;
    }
  } // -- freespace 1280 --> 3840
  return std::make_unique<std::vector<CamFSPoint>>(camera_freespace);
}

fs::LidarFSPtr fs::FsNode::parseLidarFSFromProto(const uto::proto::PerceptionFreespace *lidar_fs_proto_ptr, const Vehicle &vehicle)
{
#if FS_CHECK(CFG_USE_LIGHT_COMMON)
  LidarFSPtr lidar_fs_ptr = std::make_shared<LidarFS>();
  lidar_fs_ptr->timestamp = lidar_fs_proto_ptr->header().time_meas();
  lidar_fs_ptr->points.resize(lidar_fs_proto_ptr->grid_positions().size());
  for(int i = 0; i < lidar_fs_proto_ptr->grid_positions().size(); i += 2)
  {
    lidar_fs_ptr->points[i] = FSPoint(lidar_fs_proto_ptr->grid_positions(i),
                                      lidar_fs_proto_ptr->grid_positions(i + 1),
                                      lidar_fs_proto_ptr->altitudes(i) * 0.001f,
                                      lidar_fs_proto_ptr->altitudes(i + 1) * 0.001f,
                                      FS_PROTO_CLASS_TO_CLASS.at(lidar_fs_proto_ptr->labels(i / 2)),
                                      HDT2_LIDAR_CLASS_TO_MOTION_STATE.at(lidar_fs_proto_ptr->labels(i / 2)));
  }
  //  return lidar_fs_ptr;
#else
  LidarFSPtr lidar_fs_ptr = std::make_shared<LidarFS>();
  lidar_fs_ptr->timestamp = lidar_fs_proto_ptr->header().time_meas();
  lidar_fs_ptr->points.resize(lidar_fs_proto_ptr->perception_gridmap().size());
  /*
    +----------------+
    | 2  |  1   | 3  |
    |    |      |    |
    +----------------+
    |    |      |    |
    |    |      |    |
    |    |      |    |
    |    |      |    |
    | 7  |   0  | 8  |
    |    |      |    |
    |    |      |    |
    |    |      |    |
    +----|------|----+
    | 5  |  4   |  6 |
    |    |      |    |
    +----------------+
   * */
  for(int i = 0; i < lidar_fs_proto_ptr->perception_gridmap().size(); ++i)
  {
    // UINFO << " lidar_fs_proto_ptr->perception_gridmap().size() " << lidar_fs_proto_ptr->perception_gridmap().size();
    //  if(lidar_fs_proto_ptr->perception_gridmap().data()[i]->label() == uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN) {
    lidar_fs_ptr->points[i] = FSPoint(lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().x(),
                                      lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().y(),
                                      lidar_fs_proto_ptr->perception_gridmap().data()[i]->top(),
                                      lidar_fs_proto_ptr->perception_gridmap().data()[i]->bottom(),
                                      FS_PROTO_CLASS_TO_CLASS.at(lidar_fs_proto_ptr->perception_gridmap().data()[i]->label()),
                                      HDT2_MOVESTATUS_TO_MOVESTATUS.at(lidar_fs_proto_ptr->perception_gridmap().data()[i]->move_status()));

    // UERROR << " lidar----------------: " << lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().x() << " " << lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().y();
    if(lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().x() > vehicle.instance().getHalfLength() + 20.0f)
    {
      lidar_fs_ptr->points[i].area = 1;
      //-- area: 1
      if(lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().y() > vehicle.instance().getHalfWidth() + 10.0f)
      {
        lidar_fs_ptr->points[i].area = 2;
        //-- area: 2
      }
      else if(lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().y() < -(vehicle.instance().getHalfWidth() + 10.0f))
      {
        lidar_fs_ptr->points[i].area = 3;
        //-- area: 3
      }
    }
    else if(lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().x() < -(vehicle.instance().getHalfLength() + 20.0f))
    {
      lidar_fs_ptr->points[i].area = 4;
      //-- area: 4
      if(lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().y() > vehicle.instance().getHalfWidth() + 10.0f)
      {
        lidar_fs_ptr->points[i].area = 5;
        //-- area: 5
      }
      else if(lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().y() < -(vehicle.instance().getHalfWidth() + 10.0f))
      {
        lidar_fs_ptr->points[i].area = 6;
        //-- area: 6
      }
    }
    else
    {
      if(lidar_fs_proto_ptr->perception_gridmap().data()[i]->position().y() > 0.0f)
      {
        lidar_fs_ptr->points[i].area = 7;
        //-- area: 7
      }
      else
      {
        lidar_fs_ptr->points[i].area = 8;
        //-- area: 8
      }
    }
    // }
  }
#endif
  return lidar_fs_ptr;
}

#if FS_CHECK(CFG_USE_ROAD_MODEL)
fs::RoadModelFSPtr fs::FsNode::parseRoadModelFromProto(const uto::proto::RoadModel *road_model_proto_ptr)
{
  if(road_model_proto_ptr == nullptr)
  {
    return nullptr;
  }
  RoadModelFSPtr road_model_ptr = std::make_shared<RoadModelFS>();
  road_model_ptr->timestamp     = road_model_proto_ptr->header().time_meas();
  int count                     = road_model_proto_ptr->grids().size();
  road_model_ptr->points.reserve(count);
  for(size_t i = 0; i < count;)
  {
    int row                                          = (road_model_proto_ptr->grids()[i] - road_model_proto_ptr->ego_x_index()) * road_model_proto_ptr->grid_resolution();
    int col                                          = (-road_model_proto_ptr->grids()[i + 1] + road_model_proto_ptr->ego_y_index()) * road_model_proto_ptr->grid_resolution();
    road_model_ptr->points[std::make_pair(row, col)] = road_model_proto_ptr->grids()[i + 2] * road_model_proto_ptr->height_resolution();
    i += 3;
  }
  return road_model_ptr;
}

void fs::FsNode::refineWithRoadModel(fs::LidarFS *lidar_fs_ptr, std::shared_ptr<RoadModelFS> road_model_proto_ptr)
{
  if(lidar_fs_ptr == nullptr || road_model_proto_ptr == nullptr)
  {
    return;
  }
  std::vector<FSVec2> res;
  auto                searchInNeighbor = [&road_model_proto_ptr, &res](const FSVec2 &point) {
    //    std::vector<FSVec2> foundPoints;
    for(int i = -1; i <= 1; ++i)
    {
      for(int j = -1; j <= 1; ++j)
      {
        if(road_model_proto_ptr->points.find(std::pair(point.x() + i, point.y() + j)) != road_model_proto_ptr->points.end())
        {
          res.emplace_back(FSVec2{point.x() + i, point.y() + j});
        }
      }
    }
    return res.empty();
  };

  auto searchInGround = [&road_model_proto_ptr, &res](const FSVec2 point, const float &searchRadius) {
    //    std::vector<FSVec2> foundPoints;
    for(const auto &road_point : road_model_proto_ptr->points)
    {
      float distance = std::hypot(point.x() - road_point.first.first, point.y() - road_point.first.second);
      if(distance <= searchRadius)
      {
        res.emplace_back(FSVec2{road_point.first.first, road_point.first.second});
      }
    }
    //    return foundPoints;
  };

  for(int i = 0; i < lidar_fs_ptr->points.size(); ++i)
  {
    int row = static_cast<int>(lidar_fs_ptr->points[i].x);
    int col = static_cast<int>(lidar_fs_ptr->points[i].y);

    res.clear();
    if(searchInNeighbor(FSVec2{row, col}))
    {
      searchInGround(FSVec2{row, col}, RADIUS);
    } // -- search neighbor, 2 stage

    if(!res.empty())
    {
      float sum                      = std::accumulate(res.begin(), res.end(), 0.0f, [&road_model_proto_ptr](float acc, const FSVec2 &p) {
        return acc + road_model_proto_ptr->points.at(std::make_pair(p.x(), p.y()));
      });
      lidar_fs_ptr->points[i].ground = sum / static_cast<float>(res.size());
    }
    else
    {
      lidar_fs_ptr->points[i].ground = std::min(lidar_fs_ptr->points[i].bottom, 0.0f);
    }
  }
}
#endif

fs::ObjectStatesPtr fs::FsNode::parseVotObjects(const uto::proto::PerceptionObstacles *vot_objects_proto_ptr)
{
  if(vot_objects_proto_ptr == nullptr)
  {
    return nullptr;
  }
  ObjectStatesPtr object_states_ptr = std::make_unique<ObjectStates>();
  object_states_ptr->timestamp      = vot_objects_proto_ptr->header().time_meas();
  std::unordered_map<GridLabel, std::vector<ObjectState>> vot_map;
  for(const auto &gstObject : vot_objects_proto_ptr->perception_obstacles())
  {
    const MotionState motion_state = gstObject.move_status() == uto::proto::PerceptionObstacle_ObstacleMoveStatus_MOVING ? MotionState::MOVING : MotionState::STOPPED;
    for(auto &&box : gstObject.sub_obstacles())
    {
      const auto grid_label = perceptionClass2ObjectClass(box.obstacle_type());
      if(grid_label == GridLabel::CONE)
      {
        continue;
      }
      switch(grid_label)
      {
      case GridLabel::PEDESTRIAN:
        vot_map[GridLabel::PEDESTRIAN].emplace_back(box.position().x(), box.position().y(), box.velocity().x(), box.velocity().y(), box.velocity().x(), box.velocity().y(), box.heading_angle(), box.length(), box.width(), gstObject.id(), grid_label, motion_state);
        break;
      case GridLabel::CYCLIST:
        vot_map[GridLabel::CYCLIST].emplace_back(box.position().x(), box.position().y(), box.velocity().x(), box.velocity().y(), box.velocity().x(), box.velocity().y(), box.heading_angle(), box.length(), box.width(), gstObject.id(), grid_label, motion_state);
        break;
        //      case GridLabel::CONE:
        //        vot_map[GridLabel::CONE].emplace_back(box.position().x(), box.position().y(), box.velocity().x(), box.velocity().y(), box.velocity().x(), box.velocity().y(), box.heading_angle(), box.length(), box.width(), gstObject.id(), grid_label, motion_state);
        //        break;
      case GridLabel::RTG_QC:
        vot_map[GridLabel::RTG_QC].emplace_back(box.position().x(), box.position().y(), box.velocity().x(), box.velocity().y(), box.velocity().x(), box.velocity().y(), box.heading_angle(), box.length(), box.width(), gstObject.id(), grid_label, motion_state);
        break;
      default:
        vot_map[GridLabel::OTHER].emplace_back(box.position().x(), box.position().y(), box.velocity().x(), box.velocity().y(), box.velocity().x(), box.velocity().y(), box.heading_angle(), box.length(), box.width(), gstObject.id(), grid_label, motion_state);
        break;
      }
    } // -- double box
  }   // -- each object
  for(auto &order : vot_type_order_)
  {
    for(auto &&elm : vot_map)
    {
      if(order == elm.first)
      {
        object_states_ptr->objects.insert(object_states_ptr->objects.end(), elm.second.begin(), elm.second.end());
      }
    }
  } // -- order
  return object_states_ptr;
}

#if FS_CHECK(CFG_USE_CAM_FISH_FS)
void fs::FsNode::getRemoveNoiseFreespace3(LidarFS *lidar_fs_ptr, const std::vector<CamFSPoint> *camera_fs_ptr, const int &idx, const Camera *const camera, const FSMat4x4 &Tcl)
{
  if(camera == nullptr)
    return;
  int                      cols = camera->getImageWidth();
  int                      rows = camera->getImageHeight();
  std::vector<cv::Point3f> objectPoints(1);
  for(auto &&gstPoint : lidar_fs_ptr->points)
  {
    if(gstPoint.x > FW_THRE)
    {
      continue;
    }

    if(std::find(SENSOR_ID_TO_AREA.at(SensorId(idx)).begin(), SENSOR_ID_TO_AREA.at(SensorId(idx)).end(), gstPoint.area) == SENSOR_ID_TO_AREA.at(SensorId(idx)).end())
    {
      continue;
    }

    const float     sync_x    = Tcl(0, 0) * gstPoint.x + Tcl(0, 1) * gstPoint.y + Tcl(0, 3);
    const float     sync_y    = Tcl(1, 0) * gstPoint.x + Tcl(1, 1) * gstPoint.y + Tcl(1, 3);
    Eigen::Vector3f pc_ground = camera->transformEgo2Cam(FSVec3f(sync_x, sync_y, 0));
    if(pc_ground[2] < 0)
    {
      continue;
    }

    auto roi_fish = static_cast<Fisheye *>(const_cast<Camera *>(camera))->fov2Pixel(90.0 * M_PI / 180, 60.0 * M_PI / 180);

    // 激光相机系下3D转为柱面坐标
    Eigen::Vector3f ray   = camera->getRotMat().inverse() * pc_ground;
    cv::Point2f     pt_cy = static_cast<Fisheye *>(const_cast<Camera *>(camera))->projectToCylindrical(ray, cols, rows);

    // 将激光检测转为鱼眼2d
    auto px_ground = camera->transformCam2Img(pc_ground); //使用opencv自带库函数代替

    objectPoints[0] = cv::Point3f(pc_ground[0], pc_ground[1], pc_ground[2]);
    std::vector<cv::Point2f> imagePoints;
    cv::fisheye::projectPoints(objectPoints, imagePoints, cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), camera->toK(), camera->toD());

    cv::Point2f pt_fish = imagePoints[0];

    // 找出激光检测对应的视觉的检测点
    if((*camera_fs_ptr)[std::round(pt_fish.x)].y > camera->getImageHeight())
    {
      continue;
    }

    cv::Point   pt_fish_det(pt_fish.x, (*camera_fs_ptr)[std::round(pt_fish.x)].y);
    cv::Point2f pt_cy_vis = static_cast<Fisheye *>(const_cast<Camera *>(camera))->unprojectFisheye(pt_fish_det, camera, cols, rows);
//    if(sync_y < 0 && sync_y > -2 ){
//      UERROR <<"输入 idx " << idx << " sync_x " << sync_x << " " << gstPoint.x << " y is " << sync_y << " " << gstPoint.y << " pt_cy " << pt_cy << " pt_cy_vis " << pt_cy_vis << " lable " << static_cast<int>(gstPoint.lidar_label);
//    }
    if(pt_fish.x >= roi_fish[0] && pt_fish.x <= roi_fish[2] && pt_fish.y >= roi_fish[1] && pt_fish.y <= roi_fish[3])
    {                                                                                 // -- pixel of lidar-fs ground point out of 2160
      if((*camera_fs_ptr)[static_cast<int>(pt_fish.x)].type == CameraFSType::INVALID) // -- INVALID == wipers
      {
      }
      else if((*camera_fs_ptr)[static_cast<int>(pt_fish.x)].type == CameraFSType::RAIN_FOG)
      {
        // gstPoint.is_noise = true;
      }
      else if(pt_cy_vis.y + OFFSET_FISHFS > pt_cy.y)
      {
      } // --extrinsic parameters fluctuation
      else
      {
        if(gstPoint.lidar_label == GridLabel::TREE_BRANCH)
        {
          gstPoint.is_noise = true;
//           UERROR << " pt_cy " << pt_cy << " pt_cy_vis " << pt_cy_vis;
//          UERROR <<"被视觉判定 idx " << idx << " sync_x " << sync_x << " " << gstPoint.x << " y is " << sync_y << " " << gstPoint.y << " pt_cy " << pt_cy << " pt_cy_vis " << pt_cy_vis << " lable " << static_cast<int>(gstPoint.lidar_label);

        }
      } // -- noise

    } // -- 在图像范围内
  }   // -- iteration for each freespace point
}
#endif

#if FS_CHECK(CFG_USE_CAM_FW_FS)
void fs::FsNode::getRemoveNoiseFreespace3(LidarFS *lidar_fs_ptr, const std::vector<CamFSPoint> *camera_fs_ptr, const int &idx, const Camera *const camera, const FSMat4x4 &Tcl)
{
  for(auto &&gstPoint : lidar_fs_ptr->points)
  {
    if(gstPoint.x > FW_THRE)
    {
      continue;
    }
    const float sync_x    = Tcl(0, 0) * gstPoint.x + Tcl(0, 1) * gstPoint.y + Tcl(0, 3);
    const float sync_y    = Tcl(1, 0) * gstPoint.x + Tcl(1, 1) * gstPoint.y + Tcl(1, 3);
    auto        pc_ground = camera->transformEgo2Cam(FSVec3f(sync_x, sync_y, 0));
    if(pc_ground(2, 0) < 0)
    {
      continue;
    }
    auto px_ground = camera->transformCam2Img(pc_ground);

    if(px_ground.x() >= 0 && px_ground.x() < camera->getImageWidth())
    {
      if(px_ground.y() > camera->getImageHeight())
      {
      } // -- pixel of lidar-fs ground point out of 2160
      else if((*camera_fs_ptr)[static_cast<int>(px_ground.x())].type == CameraFSType::INVALID)
      {
      } // -- INVALID == wipers
      else if((*camera_fs_ptr)[static_cast<int>(px_ground.x())].y + OFFSET_FS > px_ground.y())
      {
      } // --extrinsic parameters fluctuation
      else
      {
        gstPoint.is_noise = true;
      } // -- noise

    } // -- 在图像范围内
  }   // -- iteration for each freespace point
}
#endif

void fs::FsNode::modifyErrorGates(const std::vector<fs::Rect> &gates_bbox, LidarFS *lidar_fs_ptr)
{
  for(size_t i = 0; i < lidar_fs_ptr->points.size(); ++i)
  {
    for(auto &&bbox : gates_bbox)
    {
      if(Utils::isInGrid(FSVec2f(lidar_fs_ptr->points[i].x, lidar_fs_ptr->points[i].y), bbox.min_x, bbox.max_x, bbox.min_y, bbox.max_y))
      {
        lidar_fs_ptr->points[i].lidar_label = GridLabel::UNOCCUPIED;
        break;
      }
    } // -- for each bbox
  }   // --for each UNKNOWN point
}

void fs::FsNode::updateGatePoints(const GatesInfoVec &gates_info_vec, std::vector<fs::Rect> &gates_bbox)
{
  if(gates_info_vec.empty())
  {
    return;
  }
  constexpr float x_up   = (GRID_MAP_SIZE - HALF_GRID_MAP_SIZE) * GRID_SCALE;
  constexpr float x_down = (0 - HALF_GRID_MAP_SIZE) * GRID_SCALE;
  constexpr float y_down = -(GRID_MAP_SIZE - HALF_GRID_MAP_SIZE) * GRID_SCALE;
  constexpr float y_up   = -(0 - HALF_GRID_MAP_SIZE) * GRID_SCALE;
  for(const auto &gate_info_ptr : gates_info_vec)
  {
    float max_y = gate_info_ptr->gate_cy + GATES_LEN * 2.5f;
    float min_y = gate_info_ptr->gate_cy - GATES_LEN * 2.0f;
    float max_x = gate_info_ptr->gate_cx + GATES_RANGE;
    float min_x = gate_info_ptr->gate_cx - GATES_RANGE;

    if(min_x >= x_up || max_x < x_down || min_y >= y_up || max_y < y_down)
    {
      continue;
    } // -- 不在栅格内
    min_x = min_x < x_down ? x_down : min_x;
    max_x = max_x >= x_up ? x_up : max_x;
    min_y = min_y < y_down ? y_down : min_y;
    max_y = max_y >= y_up ? y_up : max_y;
    gates_bbox.emplace_back(gate_info_ptr->gate_state, min_x, min_y, max_x, max_y);
  }
  return;
}

std::vector<std::vector<fs::LineCoeff>> fs::FsNode::getAllLaneCoeff()
{
  std::vector<std::vector<fs::LineCoeff>> lanes_coeff{};
#if FS_CHECK(!CFG_HDT_DT_BT_REMOVE_GRASS)
  auto fc_lanes_ptr = getLatestLanes(fc_lanes_buffer_);
  if(fc_lanes_ptr)
  {
    lanes_coeff.push_back(getLanesCoeff(fc_lanes_ptr.get()));
  }
  auto rc_lanes_ptr = getLatestLanes(rc_lanes_buffer_);
  if(rc_lanes_ptr)
  {
    lanes_coeff.push_back(getLanesCoeff(rc_lanes_ptr.get()));
  }
#else
  auto lanes_ptr = getLatestLanes(lanes_buffer_);
  if(lanes_ptr)
  {
    lanes_coeff.push_back(getLanesCoeff(lanes_ptr.get()));
  }
#endif
  return lanes_coeff;
}

std::vector<fs::LineCoeff> fs::FsNode::getLanesCoeff(const uto::proto::PerceptionLanes *perception_lines)
{
  if(perception_lines == nullptr)
  {
    return {};
  }
  std::vector<fs::LineCoeff> res;

  for(int i = 0; i < perception_lines->lanes().size(); ++i)
  {
    if(perception_lines->lanes(i).host_type() == uto::proto::PerceptionLane_HostType_LEFT)
    {
      LineCoeff line_coeff{};
      line_coeff.lines_pos  = LinesPos::LEFT;
      line_coeff.timeastamp = perception_lines->header().time_meas();
      for(int j = 0; j < perception_lines->lanes(i).coefficients().size(); ++j)
      {
        line_coeff.coeff.emplace_back(perception_lines->lanes(i).coefficients(j));
      }
      std::unique_ptr<EgoMotion> egoMotionPtr;
      getAlignedEgoMotion(line_coeff.timeastamp, egoMotionPtr, emo_pos_buffer_);
      if(egoMotionPtr)
      {
        line_coeff.emo = *egoMotionPtr;
        res.emplace_back(line_coeff);
      }
    }
    else if(perception_lines->lanes(i).host_type() == uto::proto::PerceptionLane_HostType_RIGHT)
    {
      LineCoeff line_coeff{};
      line_coeff.lines_pos  = LinesPos::RIGHT;
      line_coeff.timeastamp = perception_lines->header().time_meas();

      for(int j = 0; j < perception_lines->lanes(i).coefficients().size(); ++j)
      {
        line_coeff.coeff.emplace_back(perception_lines->lanes(i).coefficients(j));
      }
      std::unique_ptr<EgoMotion> egoMotionPtr;
      getAlignedEgoMotion(line_coeff.timeastamp, egoMotionPtr, emo_pos_buffer_);
      if(egoMotionPtr)
      {
        line_coeff.emo = *egoMotionPtr;
        res.emplace_back(line_coeff);
      }
    }
    else {}
  }

  return res;
}

std::unique_ptr<uto::proto::PerceptionLanes> fs::FsNode::getLatestLanes(const fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionLanes>, MSG_BUFFER_SIZE> &buffer)
{
  auto guard = rw_lock_.read_guard();
  if(!buffer.empty())
  {
    if(abs(fusion_timestamp_ - buffer.back()->header().time_meas()) < DELAY_MS * TIME_SCALE_MS)
    {
      uto::proto::PerceptionLanes lane = *buffer.back();
      return std::make_unique<uto::proto::PerceptionLanes>(lane);
    }
  }
  return nullptr;
}

void fs::FsNode::publishMonitorMsg()
{
  const Diagnoser &diagnoser     = Diagnoser::getDiagnoser();
  auto             monitorMsgPtr = std::make_unique<uto::proto::ErrorCode>();
  monitorMsgPtr->mutable_header()->set_time_meas(fusion_timestamp_);
  monitorMsgPtr->mutable_header()->set_time_pub(this->get_clock()->now().nanoseconds());
  // lidar fs 5004
  if(diagnoser.isLidarInvalid())
  {
    UERROR << " Not receive lidar fs, error code is 5004";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_LDR_FS_COM_ERR);
  }
#if FS_CHECK(CFG_USE_CAM_FW_FS)
  // camera fw fs 5006
  if(diagnoser.isCameraFSInvalid())
  {
    UERROR << " Not receive camera fs, error code is 5006";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_VSN_FS_COM_ERR);
  }
#endif

#if FS_CHECK(CFG_USE_CAM_FISH_FS)
  // camera fish fs 5007
  if(diagnoser.isCameraFishInvalid())
  {
    UERROR << " Not receive camera fish fs, error code is 5007";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_SURRND_VSN_FS_COM_ERR);
  }
#endif
  // vehicle pose 5027
  if(diagnoser.isEgoMotionInvalid())
  {
    UERROR << " Not receive vehicle pose, error code is 5027";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_VEH_POSE_COM_ERR);
  }
#if FS_CHECK(CFG_AIV5_BEV_FS)
  // bev fs 5065
  if(diagnoser.isBEVFSInvalid())
  {
    UERROR << " Not receive bev fs, error code is 5065";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_BEV_FS_COM_ERR);
  }
#endif
  // vot 5048
  if(diagnoser.isVOTInvalid())
  {
    UERROR << " Not receive vot, error code is 5048";
    monitorMsgPtr->add_itcs(uto::proto::ErrorCode_Itc_FUSN_FS_FUSN_MOT_COM_ERR);
  }
  fs_monitor_pub_.publish(std::move(monitorMsgPtr));
}
#if FS_CHECK(CFG_LOAD_RECORDING)
void fs::FsNode::loadBag()
{
  while(true)
  {
    if(m_bagReaderPtr->has_next())
    {
      const auto message = m_bagReaderPtr->read_next();

#if FS_CHECK(CFG_VIS_ENABLE)
      std::list<std::string> img_topic_list = {"/hal/sensor/cam_bcb_jpeg",
                                               "/hal/sensor/cam_blf_jpeg",
                                               "/hal/sensor/cam_brf_jpeg",
                                               "/hal/sensor/cam_fcf_jpeg",
                                               "/hal/sensor/cam_flb_jpeg",
                                               "/hal/sensor/cam_frb_jpeg"};
      for(auto &topic : img_topic_list)
      {
        if(message->topic_name == topic)
        {
          auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);
          if(nullptr != msgPtr)
          {
            auto guard = rw_lock_.write_guard();
            this->m_bagWriterPtr->write(*msgPtr, topic, rclcpp::Time(msgPtr->header.stamp));
          }
        }
      }
      if(message->topic_name == IMAGE_FW_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_fw_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_FN_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_fn_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_FL_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_fl_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_FR_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_fr_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_RW_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_rw_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_RN_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_rn_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_RL_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_rl_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_RR_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_rr_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
#if FS_CHECK(CFG_USE_SVC)
      if(message->topic_name == IMAGE_SFW_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_sfw_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_SFL_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_sfl_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_SFR_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_sfr_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_SRL_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_srl_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_SRR_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_srr_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == IMAGE_SRW_TOPIC)
      {
        auto msgPtr = Utils::deserializeRosMessage<sensor_msgs::msg::CompressedImage>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          img_srw_buffer_.pushFrontForce(std::move(msgPtr));
        }
      }
#endif

#endif // -- CFG_VIS_ENABLE

      if(message->topic_name == CALIB_FW_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_FW);
        }
      }
      if(message->topic_name == CALIB_FN_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_FN);
        }
      }
      if(message->topic_name == CALIB_FL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_FL);
        }
      }
      if(message->topic_name == CALIB_FR_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_FR);
        }
      }
      if(message->topic_name == CALIB_RW_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_RW);
        }
      }
      if(message->topic_name == CALIB_RN_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_RN);
        }
      }
      if(message->topic_name == CALIB_RL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_RL);
        }
      }
      if(message->topic_name == CALIB_RR_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_RR);
        }
      }

      if(message->topic_name == CALIB_SFW_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_SFW);
        }
      }
      if(message->topic_name == CALIB_SFL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_SFL);
        }
      }
      if(message->topic_name == CALIB_SFR_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_SFR);
        }
      }
      if(message->topic_name == CALIB_SRL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_SRL);
        }
      }
      if(message->topic_name == CALIB_SRR_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_SRR);
        }
      }
      if(message->topic_name == CALIB_SRW_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraCalib>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateCalibInfo(*msgPtr, SensorId::CAMERA_SRW);
        }
      }

      if(message->topic_name == FS_FW_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          fs_fw_buffer_.pushBackForce(std::move(msgPtr));
          Diagnoser::instance().updateCameraFS();
        }
      }
      if(message->topic_name == FS_FN_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          fs_fn_buffer_.pushBackForce(std::move(msgPtr));
        }
      }
#if FS_CHECK(CFG_USE_CAM_FISH_FS)
      if(message->topic_name == FS_SFW_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          fs_sfw_buffer_.pushBackForce(std::move(msgPtr));
          Diagnoser::instance().updateCameraFishSFW();
        }
      }
      if(message->topic_name == FS_SFL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          fs_sfl_buffer_.pushBackForce(std::move(msgPtr));
          Diagnoser::instance().updateCameraFishSFL();
        }
      }
      if(message->topic_name == FS_SFR_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          fs_sfr_buffer_.pushBackForce(std::move(msgPtr));
          Diagnoser::instance().updateCameraFishSFR();
        }
      }
      if(message->topic_name == FS_SRL_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          fs_srl_buffer_.pushBackForce(std::move(msgPtr));
          Diagnoser::instance().updateCameraFishSRL();
        }
      }
      if(message->topic_name == FS_SRR_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          fs_srr_buffer_.pushBackForce(std::move(msgPtr));
          Diagnoser::instance().updateCameraFishSRR();
        }
      }
      if(message->topic_name == FS_SRW_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::CameraFreespace>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          fs_srw_buffer_.pushBackForce(std::move(msgPtr));
          Diagnoser::instance().updateCameraFishSRW();
        }
      }
#endif
      if(message->topic_name == LIDAR_FS_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::PerceptionFreespace>(*message);

        if(nullptr != msgPtr)
        {
          //可视化输入FS
          rclcpp::Time                  curr_timestamp = rclcpp::Time(msgPtr->header().time_meas());
          sensor_msgs::msg::PointCloud2 cloud;
          cloud.header.stamp    = curr_timestamp;
          cloud.header.frame_id = "ego";
          cloud.height          = 1;
          cloud.width           = msgPtr->perception_gridmap().size();
          cloud.is_bigendian    = false;
          cloud.is_dense        = true;
          sensor_msgs::PointCloud2Modifier modifier(cloud);
          modifier.setPointCloud2Fields(4,
                                        "x",
                                        1,
                                        sensor_msgs::msg::PointField::FLOAT32,
                                        "y",
                                        1,
                                        sensor_msgs::msg::PointField::FLOAT32,
                                        "z",
                                        1,
                                        sensor_msgs::msg::PointField::FLOAT32,
                                        "type",
                                        1,
                                        sensor_msgs::msg::PointField::FLOAT32);
          sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
          sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
          sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
          sensor_msgs::PointCloud2Iterator<float> iter_type(cloud, "type");
          for(const auto &point : msgPtr->perception_gridmap())
          {
            *iter_x    = point.position().x();
            *iter_y    = point.position().y();
            *iter_z    = point.top();
            *iter_type = 0.;
            if(point.label() == uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN)
            {
              *iter_type = 3.;
            }
            else if(point.label() == uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_ROAD_EDGE)
            {
              *iter_type = 2.;
            }
            else if(point.label() == uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNOCCUPIED)
            {
              *iter_type = 1.;
            }
            else
            {
              *iter_type = 0.;
            }
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_type;
          }
          this->m_bagWriterPtr->write(cloud, "/fs/lidar_input", curr_timestamp);
          auto guard = rw_lock_.write_guard();
          fs_lidar_buffer_.pushBackForce(std::move(msgPtr));
          Diagnoser::instance().updateLidar();
          ++lidar_count_;
        }
      }
      if(message->topic_name == FS_TOPIC)
      {
        uto::idl::SerializedProto                        idl_msg;
        auto                                             serialized_bag_message = rclcpp::SerializedMessage(*message->serialized_data);
        rclcpp::Serialization<uto::idl::SerializedProto> idl_serialization;
        idl_serialization.deserialize_message(&serialized_bag_message, &idl_msg);
        auto msg_fusion_fs = std::make_unique<uto::proto::PerceptionFreespace>();
        msg_fusion_fs->ParseFromArray(idl_msg.data.data(), idl_msg.data.size());
        rclcpp::Time curr_timestamp = rclcpp::Time(msg_fusion_fs->header().time_meas());
        // 初始化PointCloud2消息
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp    = curr_timestamp;
        cloud.header.frame_id = "ego";
        cloud.height          = 1;                                          // 通常用于无组织点云
        cloud.width           = msg_fusion_fs->perception_gridmap().size(); // 点的数量
        cloud.is_bigendian    = false;                                      // 根据你的系统设置
        cloud.is_dense        = true;                                       // 假设没有无效点
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(4,
                                      "x",
                                      1,
                                      sensor_msgs::msg::PointField::FLOAT32,
                                      "y",
                                      1,
                                      sensor_msgs::msg::PointField::FLOAT32,
                                      "z",
                                      1,
                                      sensor_msgs::msg::PointField::FLOAT32,
                                      "motion",
                                      1,
                                      sensor_msgs::msg::PointField::FLOAT32);
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_motion(cloud, "motion");
        for(const auto &point : msg_fusion_fs->perception_gridmap())
        {
          *iter_x      = point.position().x();
          *iter_y      = point.position().y();
          *iter_z      = point.top();
          *iter_motion = float(point.move_status());
          ++iter_x;
          ++iter_y;
          ++iter_z;
          ++iter_motion;
        }
        this->m_bagWriterPtr->write(cloud, "/fs/raw", curr_timestamp);
      }
      if(message->topic_name == GATES_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::PerceptionGates>(*message);
        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          gates_buffer_.pushBackForce(std::move(msgPtr));
        }
      }
      if(message->topic_name == VEHICLE_POSE_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::VehiclePose>(*message);

        if(nullptr != msgPtr)
        {
          processVehiclePose(*msgPtr);
          Diagnoser::instance().updateEgoMotion();

          if(++vehicle_frame_count >= 5)
          {
            vehicle_frame_count = 0;
            break;
          }
        }
      }
      if(message->topic_name == VOT_OBJECTS_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::PerceptionObstacles>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          vot_object_buffer_.pushBackForce(std::move(msgPtr));
          Diagnoser::instance().updateVOTCount();
        }
      }
      if(message->topic_name == SENSOR_TABLE_TOPIC)
      {
        auto msgPtr = Utils::deserializeIdlMessage<uto::proto::SensorTable>(*message);

        if(nullptr != msgPtr)
        {
          auto guard = rw_lock_.write_guard();
          CameraManager::instance().updateConfigInfo(*msgPtr);
        }
      }
    }
    else
    {
      empty_ = true;
      return;
      //      exit(0);
    }
  }
}
#else
void fs::FsNode::subscribeSensorTable(const std::unique_ptr<uto::proto::SensorTable> &message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();

    CameraManager::instance().updateConfigInfo(*message);
  }
}
void fs::FsNode::subscribeCalibFW(const std::unique_ptr<uto::proto::CameraCalib> &message)
{
  auto guard = rw_lock_.write_guard();
  CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_FW);
}
void fs::FsNode::subscribeCalibFN(const std::unique_ptr<uto::proto::CameraCalib> &message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_FN);
  }
}
void fs::FsNode::subscribeCalibFL(const std::unique_ptr<uto::proto::CameraCalib> &message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_FL);
  }
}
void fs::FsNode::subscribeCalibFR(const std::unique_ptr<uto::proto::CameraCalib> &message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_FR);
  }
}
void fs::FsNode::subscribeCalibRW(const std::unique_ptr<uto::proto::CameraCalib> &message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_RW);
  }
}
void fs::FsNode::subscribeCalibRN(const std::unique_ptr<uto::proto::CameraCalib> &message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_RN);
  }
}
void fs::FsNode::subscribeCalibRL(const std::unique_ptr<uto::proto::CameraCalib> &message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_RL);
  }
}
void fs::FsNode::subscribeCalibRR(const std::unique_ptr<uto::proto::CameraCalib> &message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_RR);
  }
}
void fs::FsNode::subscribeFsFW(std::unique_ptr<uto::proto::CameraFreespace> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    Diagnoser::instance().updateCameraFS();
    fs_fw_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeFsFN(std::unique_ptr<uto::proto::CameraFreespace> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    fs_fn_buffer_.pushBackForce(std::move(message));
  }
}
// #if FS_CHECK(CFG_USE_CAM_FISH_FS)
void fs::FsNode::subscribeCalibSFW(std::unique_ptr<uto::proto::CameraCalib> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_SFW);
  }
}
void fs::FsNode::subscribeCalibSFL(std::unique_ptr<uto::proto::CameraCalib> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_SFL);
  }
}
void fs::FsNode::subscribeCalibSFR(std::unique_ptr<uto::proto::CameraCalib> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_SFR);
  }
}
void fs::FsNode::subscribeCalibSRL(std::unique_ptr<uto::proto::CameraCalib> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_SRL);
  }
}
void fs::FsNode::subscribeCalibSRR(std::unique_ptr<uto::proto::CameraCalib> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_SRR);
  }
}
void fs::FsNode::subscribeCalibSRW(std::unique_ptr<uto::proto::CameraCalib> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    CameraManager::instance().updateCalibInfo(*message, SensorId::CAMERA_SRW);
  }
}
// #endif
#if FS_CHECK(CFG_USE_CAM_FISH_FS)
void fs::FsNode::subscribeFsSFW(std::unique_ptr<uto::proto::CameraFreespace> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    Diagnoser::instance().updateCameraFishSFW();
    fs_sfw_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeFsSFL(std::unique_ptr<uto::proto::CameraFreespace> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    Diagnoser::instance().updateCameraFishSFL();
    fs_sfl_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeFsSFR(std::unique_ptr<uto::proto::CameraFreespace> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    Diagnoser::instance().updateCameraFishSFR();
    fs_sfr_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeFsSRL(std::unique_ptr<uto::proto::CameraFreespace> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    Diagnoser::instance().updateCameraFishSRL();
    fs_srl_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeFsSRR(std::unique_ptr<uto::proto::CameraFreespace> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    Diagnoser::instance().updateCameraFishSRR();
    fs_srr_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeFsSRW(std::unique_ptr<uto::proto::CameraFreespace> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    Diagnoser::instance().updateCameraFishSRW();
    fs_srw_buffer_.pushBackForce(std::move(message));
  }
}
#endif // -- CFG_USE_CAM_FISH_FS
void fs::FsNode::subscribeFsLidar(std::unique_ptr<uto::proto::PerceptionFreespace> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    Diagnoser::instance().updateLidar();
    fs_lidar_buffer_.pushBackForce(std::move(message));
    ++lidar_count_;
  }
}
#if FS_CHECK(CFG_USE_MCAP_RAW)
void fs::FsNode::subscribeFsLidarFromBag(std::unique_ptr<uto::proto::PerceptionFreespace> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
#if FS_CHECK(CFG_PUB_ENU_FS)
    freespace_manager_.drawBag(message, getLatestEgoMotion(emo_align_pos_buffer_).get());
#else
    freespace_manager_.drawBag(message);
#endif
  }
}
#endif

void fs::FsNode::subscribeVehicleConfig(const std::unique_ptr<uto::proto::MechanicalInfo> &message) //TODO: waitting for common update
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    Vehicle::instance().updateMechanicInfo(*message);
  }
}

void fs::FsNode::subscribeVehiclePose(const std::unique_ptr<uto::proto::VehiclePose> &message)
{
  if(nullptr != message)
  {
    Diagnoser::instance().updateEgoMotion();
    processVehiclePose(*message);
  }
}
void fs::FsNode::subscribeGates(std::unique_ptr<uto::proto::PerceptionGates> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    gates_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeVotObject(std::unique_ptr<uto::proto::PerceptionObstacles> message)
{
  if(nullptr != message)
  {
    Diagnoser::instance().updateVOTCount();
    auto guard = rw_lock_.write_guard();
    vot_object_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeFCLanes(std::unique_ptr<uto::proto::PerceptionLanes> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    fc_lanes_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeRCLanes(std::unique_ptr<uto::proto::PerceptionLanes> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    rc_lanes_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeLanes(std::unique_ptr<uto::proto::PerceptionLanes> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    lanes_buffer_.pushBackForce(std::move(message));
  }
}

#if FS_CHECK(CFG_USE_ROAD_MODEL)
void fs::FsNode::subscribeRoadModel(std::unique_ptr<uto::proto::RoadModel> message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    road_model_buffer_.pushBackForce(std::move(message));
  }
}
#endif

#if FS_CHECK(CFG_VIS_ENABLE)

void fs::FsNode::subscribeImageFW(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_fw_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageFN(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_fn_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageFL(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_fl_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageFR(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_fr_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageRW(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_rw_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageRN(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_rn_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageRL(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_rl_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageRR(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_rr_buffer_.pushBackForce(std::move(message));
  }
}
#if FS_CHECK(CFG_USE_SVC)
void fs::FsNode::subscribeImageSFW(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_sfw_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageSFL(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_sfl_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageSFR(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_sfr_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageSRL(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_srl_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageSRR(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_srr_buffer_.pushBackForce(std::move(message));
  }
}
void fs::FsNode::subscribeImageSRW(sensor_msgs::msg::CompressedImage::UniquePtr message)
{
  if(nullptr != message)
  {
    auto guard = rw_lock_.write_guard();
    img_srw_buffer_.pushBackForce(std::move(message));
  }
}
#endif // -- CFG_USE_SVC
#endif // -- CFG_VIS_ENABLE
#endif // -- CFG_LOAD_RECORDING
#else
#if FS_CHECK(CFG_LOAD_RECORDING)
void fs::FsNode::loadLcm()
{
  static lcm::LogFile lcmLog{recording_path_, "r"};
  assert(lcmLog.good());

  while(true)
  {
    const lcm::LogEvent *event = lcmLog.readNextEvent();
    if(nullptr != event)
    {
#if FS_CHECK(CFG_VIS_ENABLE)
      if(event->channel == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_FW))
      {
        processImage(img_fw_buffer_, event->data, event->datalen);
      }
      if(event->channel == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_FN))
      {
        processImage(img_fn_buffer_, event->data, event->datalen);
      }
      if(event->channel == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_FL))
      {
        processImage(img_fl_buffer_, event->data, event->datalen);
      }
      if(event->channel == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_FR))
      {
        processImage(img_fr_buffer_, event->data, event->datalen);
      }
      if(event->channel == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_RW))
      {
        processImage(img_rw_buffer_, event->data, event->datalen);
      }
      if(event->channel == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_RN))
      {
        processImage(img_rn_buffer_, event->data, event->datalen);
      }
      if(event->channel == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_RL))
      {
        processImage(img_rl_buffer_, event->data, event->datalen);
      }
      if(event->channel == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_RR))
      {
        processImage(img_rr_buffer_, event->data, event->datalen);
      }
#endif
      if(event->channel == TOPIC_TRANSFORM)
      {
        processVehiclePose(event->data, event->datalen);
      }
      if(event->channel == TOPIC_LIDAR_GRIDMAP)
      {
        processLidarFreeSpace(event->data, event->datalen);
        break;
      }
      if(event->channel == TOPIC_NORMAL_CAMERA_FS)
      {
        processFWFreeSpace(event->data, event->datalen);
      }
      if(event->channel == TOPIC_CAM_FS_BEV)
      {
        processBEVFreeSpace(event->data, event->datalen);
      }
      if(event->channel == TOPIC_VCU_VEHICLE_INFO)
      {
        processVcuInfo(event->data, event->datalen);
      }
      if(event->channel == TOPIC_LANE_CONTROL)
      {
        processLaneControlInfo(event->data, event->datalen);
      }
      if(event->channel == TOPIC_VOT)
      {
        processVotObscales(event->data, event->datalen);
      }
      if(event->channel == TOPIC_LANE)
      {
        processLane(event->data, event->datalen);
      }
      if(event->channel == TOPIC_INTEGRATEDPOSITION)
      {
        processLocalization(event->data, event->datalen);
      }
    }
    else
    {
      exit(0);
    }
  }
}
#else
void fs::FsNode::lcmReceiverCallback(const lcm::ReceiveBuffer *recv_buf, const std::string &channel_name)
{
#if FS_CHECK(CFG_VIS_ENABLE)
  // clang-format off
  if(channel_name == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_FW)){processImage(img_fw_buffer_, recv_buf->data, recv_buf->data_size);}
  if(channel_name == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_FN)){processImage(img_fn_buffer_, recv_buf->data, recv_buf->data_size);}
  if(channel_name == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_RW)){processImage(img_rw_buffer_, recv_buf->data, recv_buf->data_size);}
  if(channel_name == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_RN)){processImage(img_rn_buffer_, recv_buf->data, recv_buf->data_size);}
  if(channel_name == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_FL)){processImage(img_fl_buffer_, recv_buf->data, recv_buf->data_size);}
  if(channel_name == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_FR)){processImage(img_fr_buffer_, recv_buf->data, recv_buf->data_size);}
  if(channel_name == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_RL)){processImage(img_rl_buffer_, recv_buf->data, recv_buf->data_size);}
  if(channel_name == CAMERA_IMAGE_TO_LCM_NAME.at(SensorId::CAMERA_RR)){processImage(img_rr_buffer_, recv_buf->data, recv_buf->data_size);}
  // clang-format on
#if FS_CHECK(CFG_DEBUG_PERCEPTION_FREESPACES)
  if(channel_name == TOPIC_PLANNER_GRIDMAP_AIV)
  {
    subscribeFsLidarFromBag(recv_buf->data, recv_buf->data_size);
  }
#endif
#endif
  if(channel_name == TOPIC_INTEGRATEDPOSITION)
  {
    processLocalization(recv_buf->data, recv_buf->data_size);
  }
  if(channel_name == TOPIC_TRANSFORM)
  {
    processVehiclePose(recv_buf->data, recv_buf->data_size);
  }
  if(channel_name == TOPIC_LIDAR_GRIDMAP)
  {
    processLidarFreeSpace(recv_buf->data, recv_buf->data_size);
  }
  if(channel_name == TOPIC_NORMAL_CAMERA_FS)
  {
    processFWFreeSpace(recv_buf->data, recv_buf->data_size);
  }
  if(channel_name == TOPIC_CAM_FS_BEV)
  {
    processBEVFreeSpace(recv_buf->data, recv_buf->data_size);
  }
  if(channel_name == TOPIC_VCU_VEHICLE_INFO)
  {
    processVcuInfo(recv_buf->data, recv_buf->data_size);
  }
  if(channel_name == TOPIC_LANE_CONTROL)
  {
    processLaneControlInfo(recv_buf->data, recv_buf->data_size);
  }
  if(channel_name == TOPIC_VOT)
  {
    processVotObscales(recv_buf->data, recv_buf->data_size);
  }
  if(channel_name == TOPIC_LANE)
  {
    processLane(recv_buf->data, recv_buf->data_size);
  }
  if(channel_name == TOPIC_LOC_MONITOR)
  {
    processLocMonitor(recv_buf->data, recv_buf->data_size);
  }
}
#endif // -- CFG_LOAD_RECORDING
fs::ObjectStatesPtr fs::FsNode::parseVotObjects(const PERCEPTION_OBSTACLES *vot_objects_lcm_ptr)
{
  if(vot_objects_lcm_ptr == nullptr)
  {
    return nullptr;
  }
  ObjectStatesPtr object_states_ptr = std::make_unique<ObjectStates>();
  object_states_ptr->timestamp = vot_objects_lcm_ptr->stHeader.nTimeStamp;
  std::unordered_map<GridLabel, std::vector<ObjectState>> vot_map;
  int flag = 1;
  float flag_angle = 0;
  if(getReverse())
  {
    flag = -1;
    flag_angle = M_PI;
  }

  for(const auto &gstObject : vot_objects_lcm_ptr->gstObstacles)
  {
    if(!VOT_LCM_CLASS_TO_CLASS.count(gstObject.nType))
    {
      continue;
    }
    const auto grid_label = VOT_LCM_CLASS_TO_CLASS.at(gstObject.nType);
    if(grid_label == GridLabel::CONE)
    {
      continue;
    }
    const MotionState motion_state = gstObject.nMoveStatus == 1 ? MotionState::MOVING : MotionState::STOPPED;
    switch(grid_label)
    {
    case GridLabel::PEDESTRIAN:
      vot_map[GridLabel::PEDESTRIAN].emplace_back(gstObject.fRelX * flag,
                                                  gstObject.fRelY * flag,
                                                  gstObject.fAbsVelX * flag,
                                                  gstObject.fAbsVelY * flag,
                                                  gstObject.fAbsVelX * flag,
                                                  gstObject.fAbsVelY * flag,
                                                  gstObject.fHeading + flag_angle,
                                                  gstObject.fLength,
                                                  gstObject.fWidth,
                                                  gstObject.nObjectID,
                                                  grid_label,
                                                  motion_state);
      break;
    case GridLabel::CYCLIST:
      vot_map[GridLabel::CYCLIST].emplace_back(gstObject.fRelX * flag,
                                               gstObject.fRelY * flag,
                                               gstObject.fAbsVelX * flag,
                                               gstObject.fAbsVelY * flag,
                                               gstObject.fAbsVelX * flag,
                                               gstObject.fAbsVelY * flag,
                                               gstObject.fHeading + flag_angle,
                                               gstObject.fLength,
                                               gstObject.fWidth,
                                               gstObject.nObjectID,
                                               grid_label,
                                               motion_state);
      break;
    case GridLabel::CONE:
      vot_map[GridLabel::CONE].emplace_back(gstObject.fRelX * flag,
                                            gstObject.fRelY * flag,
                                            gstObject.fAbsVelX * flag,
                                            gstObject.fAbsVelY * flag,
                                            gstObject.fAbsVelX * flag,
                                            gstObject.fAbsVelY * flag,
                                            gstObject.fHeading + flag_angle,
                                            gstObject.fLength,
                                            gstObject.fWidth,
                                            gstObject.nObjectID,
                                            grid_label,
                                            motion_state);
      break;
    case GridLabel::RTG_QC:
      vot_map[GridLabel::RTG_QC].emplace_back(gstObject.fRelX * flag,
                                              gstObject.fRelY * flag,
                                              gstObject.fAbsVelX * flag,
                                              gstObject.fAbsVelY * flag,
                                              gstObject.fAbsVelX * flag,
                                              gstObject.fAbsVelY * flag,
                                              gstObject.fHeading + flag_angle,
                                              gstObject.fLength,
                                              gstObject.fWidth,
                                              gstObject.nObjectID,
                                              grid_label,
                                              motion_state);
      break;
    default:
      vot_map[GridLabel::OTHER].emplace_back(gstObject.fRelX * flag,
                                             gstObject.fRelY * flag,
                                             gstObject.fAbsVelX * flag,
                                             gstObject.fAbsVelY * flag,
                                             gstObject.fAbsVelX * flag,
                                             gstObject.fAbsVelY * flag,
                                             gstObject.fHeading + flag_angle,
                                             gstObject.fLength,
                                             gstObject.fWidth,
                                             gstObject.nObjectID,
                                             grid_label,
                                             motion_state);
      break;
    }
  }
  for(auto &order : vot_type_order_)
  {
    for(auto &&elm : vot_map)
    {
      if(order == elm.first)
      {
        object_states_ptr->objects.insert(object_states_ptr->objects.end(), elm.second.begin(), elm.second.end());
      }
    }
  } // -- order
  return object_states_ptr;
}

fs::BevSegPtr fs::FsNode::parseBevSeg(const FS_BEV_SEG::CameraFreespaceSeg *bev_seg_lcm_ptr)
{
  if(bev_seg_lcm_ptr == nullptr)
  {
    return nullptr;
  }
  BevSegPtr bev_seg_ptr = std::make_unique<BevSeg>();
  const int width = bev_seg_lcm_ptr->bev_fs_seg_image_width;
  const int height = bev_seg_lcm_ptr->bev_fs_seg_image_height;
  const float width_s = bev_seg_lcm_ptr->bev_fs_seg_width_resolution * 0.01;
  const float height_s = bev_seg_lcm_ptr->bev_fs_seg_height_resolution * 0.01;
  bev_seg_ptr->height_m = bev_seg_lcm_ptr->bev_fs_seg_image_height * bev_seg_lcm_ptr->bev_fs_seg_height_resolution;
  bev_seg_ptr->width_m = bev_seg_lcm_ptr->bev_fs_seg_image_width * bev_seg_lcm_ptr->bev_fs_seg_width_resolution;
  bev_seg_ptr->timestamp = bev_seg_lcm_ptr->measure_timestamp;

  auto &points = bev_seg_ptr->points;
  const int car_col = width / 2;
  const int car_row = height / 2;
  for(int i = 0; i < bev_seg_lcm_ptr->bev_fs_seg_image_data_label.size(); ++i)
  {
    int row = i / width;
    int col = i % width;
    points.emplace_back(-(row - car_row) * height_s,
                        -(col - car_col) * width_s,
                        bev_seg_lcm_ptr->bev_fs_seg_image_data_score[i] * 0.01f,
                        bev_seg_lcm_ptr->bev_fs_seg_image_data_label[i]);
    //    UERROR << "pt x is " << points[points.size()-1].x << " y is " << points[points.size() - 1].y;
  }
  return bev_seg_ptr;
}

std::unique_ptr<PERCEPTION_OBSTACLES> fs::FsNode::cloneVotObjects(std::unique_ptr<PERCEPTION_OBSTACLES> &vot_object_ptr)
{
  if(vot_object_ptr == nullptr)
  {
    return nullptr;
  }
  PERCEPTION_OBSTACLES vot_obj{};
  vot_obj.stHeader = vot_object_ptr->stHeader;
  vot_obj.gstObstacles = vot_object_ptr->gstObstacles;
  vot_obj.obj_num = vot_object_ptr->obj_num;
  return std::make_unique<PERCEPTION_OBSTACLES>(vot_obj);
}

void fs::FsNode::processVotObscales(const void *data, uint32_t size)
{
  auto msg_ptr = std::make_unique<PERCEPTION_OBSTACLES>();
  msg_ptr->decode(data, 0, size);

  if(nullptr != msg_ptr)
  {
    Diagnoser::instance().updateVOTCount();
    auto guard = rw_lock_.write_guard();
    vot_objects_buffer_.pushBackForce(std::move(msg_ptr));
  }
}

bool fs::FsNode::getAlignedInputs(std::unique_ptr<Lidar_FreeSpace_v2> &fs_lidar_lcm_ptr,
                                  std::unique_ptr<FS_BEV_SEG::CameraFreespaceSeg> &fs_bev_lcm_ptr,
                                  std::unique_ptr<DMAIV::DmAivBatch> &image_bev_lcm_ptr,
                                  std::unique_ptr<EgoMotion> &emo_pos_ptr,
                                  std::unique_ptr<PERCEPTION_OBSTACLES> &vot_objects_lcm_ptr)
{
  auto guard = rw_lock_.write_guard();
  Diagnoser::instance().updateCount();

  bool valid_aligned = false;

  if(!emo_pos_buffer_.empty())
  {
    if(emo_pos_buffer_.back().timestamp < emo_pos_buffer_.front().timestamp)
    {
      // play reset
      resetBuffers();
    }
    else
    {
      const bool all_inputs_arrived = !fs_bev_buffer_.empty() && !fs_lidar_buffer_.empty();

      const bool too_many_lidar_arrived = fs_lidar_buffer_.size() > SYNC_MAX_LIDAR_BUFFER_SIZE;

      const bool should_trigger_alignment = all_inputs_arrived || too_many_lidar_arrived;

      UINFO << "should trigger alignment: " << should_trigger_alignment
            << ", lidar fs size: " << fs_lidar_buffer_.size()
            << ", all inputs arrived: " << all_inputs_arrived;

      if(should_trigger_alignment)
      {
        valid_aligned = true;

        if(fs_lidar_buffer_.empty())
        {
          return false;
        }
        fusion_timestamp_ = fs_lidar_buffer_.front()->stHeader.nTimeStamp;
        UINFO << "[use lidar timestamp: " << fusion_timestamp_ << "]";

        const auto emoRhsIter = std::find_if(emo_pos_buffer_.begin(), emo_pos_buffer_.end(), [this](const EgoMotion &pose) { return static_cast<int64_t>(pose.timestamp) >= fusion_timestamp_; });
        const auto emoLhsIter = std::find_if(emo_pos_buffer_.rbegin(), emo_pos_buffer_.rend(), [this](const EgoMotion &pose) { return static_cast<int64_t>(pose.timestamp) <= fusion_timestamp_; });
        if(emoLhsIter == emo_pos_buffer_.rend() || emoRhsIter == emo_pos_buffer_.end())
        {
          // aligned timestamp is earlier than the earliest transform or later than the latest transform , means we can't have transform for this frame.
          // this should happen when the recording replays or the vehicle_pose message's disappearance
          UWARN << "drop fusion timestamp: " << fusion_timestamp_ / TIME_SCALE_MS << " ms; "
                << "ego motion timestamp: front " << emo_pos_buffer_.front().timestamp / TIME_SCALE_MS << " ms | "
                << "back " << emo_pos_buffer_.back().timestamp / TIME_SCALE_MS << " ms";
          //          resetBuffers();
          valid_aligned = false;
        }
        else
        {
          if(!fs_bev_buffer_.empty())
          {
            if(std::abs(fusion_timestamp_ - fs_bev_buffer_.back()->measure_timestamp) < DELAY_MS * TIME_SCALE_MS)
            {
              fs_bev_lcm_ptr = cloneBevFs(fs_bev_buffer_.back());
            }
          } // -- bev-fs

          if(!vot_objects_buffer_.empty())
          {
            const auto votRhsIter = std::find_if(vot_objects_buffer_.begin(),
                                                 vot_objects_buffer_.end(),
                                                 [this](const std::unique_ptr<PERCEPTION_OBSTACLES> &vot) { return static_cast<int64_t>(vot->stHeader.nTimeStamp) >= fusion_timestamp_; });
            const auto votLhsIter = std::find_if(vot_objects_buffer_.rbegin(),
                                                 vot_objects_buffer_.rend(),
                                                 [this](const std::unique_ptr<PERCEPTION_OBSTACLES> &vot) { return static_cast<int64_t>(vot->stHeader.nTimeStamp) <= fusion_timestamp_; });
            if(votLhsIter == vot_objects_buffer_.rend() || votRhsIter == vot_objects_buffer_.end())
            {
              if(std::abs(fusion_timestamp_ - vot_objects_buffer_.back()->stHeader.nTimeStamp) < DELAY_MS * TIME_SCALE_MS)
              {
                vot_objects_lcm_ptr = cloneVotObjects(vot_objects_buffer_.back());
              }
            } // -- can't find
            else
            {
              if(std::abs(votRhsIter->get()->stHeader.nTimeStamp - fusion_timestamp_) > std::abs(votLhsIter->get()->stHeader.nTimeStamp - fusion_timestamp_))
              {
                vot_objects_lcm_ptr = cloneVotObjects(*votLhsIter);
              }
              else
              {
                vot_objects_lcm_ptr = cloneVotObjects(*votRhsIter);
              }
            } // select min err
          }   // -- vot-obj
          emo_pos_ptr = std::make_unique<EgoMotion>(interpolateEgoMotion(fusion_timestamp_, *emoLhsIter, *emoRhsIter));
        }
        fs_lidar_lcm_ptr = std::move(fs_lidar_buffer_.front());
        fs_lidar_buffer_.popFront();
      }
    }
  }

  return valid_aligned;
}

#if FS_CHECK(CFG_DEBUG_PERCEPTION_FREESPACES)
void fs::FsNode::subscribeFsLidarFromBag(const void *data, uint32_t size)
{
  auto msg_ptr = std::make_unique<PerceptionFreeSpace>();
  msg_ptr->decode(data, 0, static_cast<int>(size));
  if(nullptr != msg_ptr)
  {
    freespace_manager_.drawBag(*msg_ptr.get(), nullptr);
#if FS_CHECK(CFG_USE_FOXGLOV_VIS)
    for(auto &pub : map_pub_self_receive2_)
    {
      pub.second->publish(Vis::getSelfReceivePointCloud2().at(pub.first));
    }
#else
    for(auto &pub : map_pub_self_receive_)
    {
      pub.second->publish(Vis::getSelfReceivePointCloud().at(pub.first));
    }
#endif
  }
}
#endif // -- CFG_DEBUG_PERCEPTION_FREESPACES

void fs::FsNode::process()
{
#if FS_CHECK(CFG_LOAD_RECORDING)
  loadLcm();
#endif // -- CFG_LOAD_RECORDING

  std::unique_ptr<Lidar_FreeSpace_v2> fs_lidar_lcm_ptr = nullptr;
  std::unique_ptr<FS_BEV::CameraFreespace> fs_fw_lcm_ptr = nullptr;
  std::unique_ptr<FS_BEV::CameraFreespace> fs_rw_lcm_ptr = nullptr;
  std::unique_ptr<LOGITECH_IMAGE> image_fw_lcm_ptr = nullptr;
  std::unique_ptr<LOGITECH_IMAGE> image_fn_lcm_ptr = nullptr;
  std::unique_ptr<LOGITECH_IMAGE> image_rw_lcm_ptr = nullptr;
  std::unique_ptr<LOGITECH_IMAGE> image_rn_lcm_ptr = nullptr;
  std::unique_ptr<LOGITECH_IMAGE> image_fl_lcm_ptr = nullptr;
  std::unique_ptr<LOGITECH_IMAGE> image_fr_lcm_ptr = nullptr;
  std::unique_ptr<LOGITECH_IMAGE> image_rl_lcm_ptr = nullptr;
  std::unique_ptr<LOGITECH_IMAGE> image_rr_lcm_ptr = nullptr;
  std::unique_ptr<DMAIV::DmAivBatch> image_bev_lcm_ptr = nullptr;
  std::unique_ptr<EgoMotion> emo_pos_ptr = nullptr;
  std::unique_ptr<FS_BEV_SEG::CameraFreespaceSeg> fs_bev_lcm_ptr = nullptr;

  std::unique_ptr<PERCEPTION_LINES> lane_ptr = nullptr;
  {
    auto guard = rw_lock_.read_guard();
    if(!lanes_buffer_.empty())
    {
      if(abs(fusion_timestamp_ - lanes_buffer_.back()->stHeader.nTimeStamp) < DELAY_MS * TIME_SCALE_MS)
      {
        PERCEPTION_LINES lane = *lanes_buffer_.back();
        lane_ptr = std::make_unique<PERCEPTION_LINES>(lane);
      }
    }
  }

#if FS_CHECK(CFG_VIS_ENABLE)
  {
    auto guard = rw_lock_.write_guard();
    if(!img_fw_buffer_.empty())
    {
      LOGITECH_IMAGE image_fw_lcm = *img_fw_buffer_.back();
      image_fw_lcm_ptr = std::make_unique<LOGITECH_IMAGE>(image_fw_lcm);
    }
    if(!img_fn_buffer_.empty())
    {
      LOGITECH_IMAGE image_fn_lcm = *img_fn_buffer_.back();
      image_fn_lcm_ptr = std::make_unique<LOGITECH_IMAGE>(image_fn_lcm);
    }
    if(!img_rw_buffer_.empty())
    {
      LOGITECH_IMAGE image_rw_lcm = *img_rw_buffer_.back();
      image_rw_lcm_ptr = std::make_unique<LOGITECH_IMAGE>(image_rw_lcm);
    }
    if(!img_rn_buffer_.empty())
    {
      LOGITECH_IMAGE image_rn_lcm = *img_rn_buffer_.back();
      image_rn_lcm_ptr = std::make_unique<LOGITECH_IMAGE>(image_rn_lcm);
    }
    if(!img_fl_buffer_.empty())
    {
      LOGITECH_IMAGE image_fl_lcm = *img_fl_buffer_.back();
      image_fl_lcm_ptr = std::make_unique<LOGITECH_IMAGE>(image_fl_lcm);
    }
    if(!img_fr_buffer_.empty())
    {
      LOGITECH_IMAGE image_fr_lcm = *img_fr_buffer_.back();
      image_fr_lcm_ptr = std::make_unique<LOGITECH_IMAGE>(image_fr_lcm);
    }
    if(!img_rl_buffer_.empty())
    {
      LOGITECH_IMAGE image_rl_lcm = *img_rl_buffer_.back();
      image_rl_lcm_ptr = std::make_unique<LOGITECH_IMAGE>(image_rl_lcm);
    }
    if(!img_rr_buffer_.empty())
    {
      LOGITECH_IMAGE image_rr_lcm = *img_rr_buffer_.back();
      image_rr_lcm_ptr = std::make_unique<LOGITECH_IMAGE>(image_rr_lcm);
    }
  }
#endif // -- CFG_VIS_ENABLE
  std::unique_ptr<PERCEPTION_OBSTACLES> vot_objects_lcm_ptr = nullptr;

  const bool valid_aligned = getAlignedInputs(fs_lidar_lcm_ptr,
                                              fs_bev_lcm_ptr,
                                              image_bev_lcm_ptr,
                                              emo_pos_ptr,
                                              vot_objects_lcm_ptr);

  Vehicle &vehicle = Vehicle::instance();
  if(nullptr != emo_pos_ptr)
  {
    vehicle.updateVehicle(emo_pos_ptr.get());
  }

  CameraManager &camera_manager = CameraManager::instance();
  if(!camera_manager.isAllCameraValid())
  {
    UINFO << "init camera ";
    camera_manager.readCalibInfo(vehicle_number_);
  }

  if(!fs::MapProvider::getMapProvider().isInit())
  {
    fs::MapProvider::getMapProvider().transformLLA2LocalXTM(init_x_, init_y_, init_z_, init_flag_);
  }

  if(valid_aligned && vehicle.isValid() && fs_lidar_lcm_ptr != nullptr)
  {
#if FS_CHECK(CFG_VIS_ENABLE)
    Vis::clearMarkers();
#endif // -- CFG_VIS_ENABLE
    auto lidar_fs_ptr = parseLidarFreespace(fs_lidar_lcm_ptr.get());
    auto vot_object_ptr = parseVotObjects(vot_objects_lcm_ptr.get());
    auto bev_seg_ptr = parseBevSeg(fs_bev_lcm_ptr.get());
    const auto loc_flag = getLocFlag();
    const auto is_lock_station = isLockStation(vot_object_ptr);
    const auto in_ele_fence = fs::MapProvider::getMapProvider().isInElectricFence(Vehicle::instance().getVecEgo2Wrd());
    std::vector<cv::Point2f> polygon_lane = getLanePolygon(lidar_fs_ptr.get(), lane_ptr.get());

    {
      if(fs_bev_lcm_ptr != nullptr)
      {
        refineBevFsSeg(lidar_fs_ptr->timestamp, bev_seg_ptr.get()); // -- modify bev_fs_ptr-- tf to lidar
        if(last_sync_bev_lidar_timestamp_ != fs_bev_lcm_ptr->measure_timestamp)
        {
          //          addBevFreespace(lidar_fs_ptr.get(), fs_bev_lcm_ptr.get()); // -- don't modify  bev_fs_ptr
          last_sync_bev_lidar_timestamp_ = fs_bev_lcm_ptr->measure_timestamp;
        } // -- new data
        else
        {
          UINFO << "can't find latest BEV at: " << lidar_fs_ptr->timestamp;
        } // -- old data
      }   // -- delay < 1s
      else
      {
        //UWARN << " !!! BEV delay > 1s ";
      } // -- delay > 1s
    }   // -- process bev fs:  时间补偿
    {
      if(vot_object_ptr != nullptr)
      {
        refineVotObject(lidar_fs_ptr.get(), vot_object_ptr.get());
        if(last_sync_vot_lidar_timestamp_ != vot_object_ptr->timestamp)
        {
          last_sync_vot_lidar_timestamp_ = vot_object_ptr->timestamp;
        } // -- new data
        else
        {
          UINFO << "can't find latest VOT at: " << lidar_fs_ptr->timestamp;
        } // -- old data
      }   // -- delay < 1s
      else
      {
        //UWARN << " !!! VOT delay > 1s ";
      } // -- delay > 1s
    }   // -- process vot obj: 预测+时间补偿

    std::unique_ptr<PerceptionFreeSpace> fusion_fs_ptr = std::make_unique<PerceptionFreeSpace>();
    if(loc_flag == LocFlag::VALID)
    {
      freespace_manager_.setFusionMapResetFlag(false);
      freespace_manager_.fuseFreespace(fusion_timestamp_, bev_seg_ptr.get(), lidar_fs_ptr.get(), vot_object_ptr.get(), fusion_fs_ptr, getLatestEgoMotion(emo_pos_buffer_).get(), emo_pos_ptr.get(), fs_lidar_lcm_ptr->fResolution, getReverse(), polygon_lane, in_ele_fence, is_lock_station, fs_lidar_lcm_ptr.get());
    }
    else if(loc_flag == LocFlag::LOW_VALID)
    {
      freespace_manager_.fuseFreespace(fusion_timestamp_, bev_seg_ptr.get(), lidar_fs_ptr.get(), vot_object_ptr.get(), fusion_fs_ptr, getLatestEgoMotion(emo_pos_buffer_).get(), emo_pos_ptr.get(), fs_lidar_lcm_ptr->fResolution, getReverse(), polygon_lane, in_ele_fence, is_lock_station, fs_lidar_lcm_ptr.get());
      freespace_manager_.tptsFreespace(fusion_timestamp_, vot_object_ptr.get(), fusion_fs_ptr, fs_lidar_lcm_ptr->fResolution, getReverse(), fs_lidar_lcm_ptr.get());
    }
    else
    {
      if(!freespace_manager_.getFusionMapResetFlag())
      {
        freespace_manager_.resetFusionMap();
        freespace_manager_.setFusionMapResetFlag(true);
      }
      freespace_manager_.tptsFreespace(fusion_timestamp_, vot_object_ptr.get(), fusion_fs_ptr, fs_lidar_lcm_ptr->fResolution, getReverse(), fs_lidar_lcm_ptr.get());
      freespace_manager_.resetTptsMap();
    }

#if FS_CHECK(!CFG_DEBUG_PERCEPTION_FREESPACES)
    auto pub = std::chrono::steady_clock::now();
    m_lcmHandler_Planner_Gridmap_AIV.publish("Planner_Gridmap_AIV", fusion_fs_ptr.get());
    UINFO << "pub: " << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - pub).count();
#endif // -- !CFG_DEBUG_PERCEPTION_FREESPACES

#if FS_CHECK(CFG_VIS_ENABLE)
    auto vot_object_raw_ptr = parseVotObjects(vot_objects_lcm_ptr.get());
    if(!(is_lock_station || in_ele_fence))
    {
      polygon_lane.clear();
    }
    Vis::prepare(fusion_timestamp_,
                 lidar_fs_ptr.get(),
                 polygon_lane,
                 image_bev_lcm_ptr.get(),
                 bev_seg_ptr.get(),
                 vot_object_raw_ptr.get(),
                 vot_object_ptr.get(),
                 image_fw_lcm_ptr.get(),
                 image_fn_lcm_ptr.get(),
                 image_rw_lcm_ptr.get(),
                 image_rn_lcm_ptr.get(),
                 image_fl_lcm_ptr.get(),
                 image_fr_lcm_ptr.get(),
                 image_rl_lcm_ptr.get(),
                 image_rr_lcm_ptr.get());
#endif // -- CFG_VIS_ENABLE
  }
  publishMonitorMsg();

#if FS_CHECK(CFG_VIS_ENABLE)
  publishVis();
#endif // -- CFG_VIS_ENABLE
}

void fs::FsNode::refineBevFsSeg(int64_t timestamp, BevSeg *bev_fs_ptr)
{
  if(bev_fs_ptr == nullptr)
  {
    return;
  }
  std::unique_ptr<EgoMotion> emo_pos_Twb_lidar = nullptr;
  std::unique_ptr<EgoMotion> emo_pos_Twb_bev = nullptr;
  if(getAlignedEgoMotion(timestamp, emo_pos_Twb_lidar, emo_pos_buffer_) == 1 && getAlignedEgoMotion(bev_fs_ptr->timestamp, emo_pos_Twb_bev, emo_pos_buffer_) == 1)
  {
    const auto Twb_lidar = Utils::makeTFrom6Dof(emo_pos_Twb_lidar->yaw, 0., 0., emo_pos_Twb_lidar->translation.x(), emo_pos_Twb_lidar->translation.y(), 0.);
    const auto Twb_bev = Utils::makeTFrom6Dof(emo_pos_Twb_bev->yaw, 0., 0., emo_pos_Twb_bev->translation.x(), emo_pos_Twb_bev->translation.y(), 0.);
    FSMat4x4 Tcl = Utils::inverse(Twb_lidar) * Twb_bev;
    for(auto &&p : bev_fs_ptr->points)
    {
      //      UERROR << "pt x is " << p.x << " y is " << p.y;
      float x = Tcl(0, 0) * p.x + Tcl(0, 1) * p.y + Tcl(0, 3);
      float y = Tcl(1, 0) * p.x + Tcl(1, 1) * p.y + Tcl(1, 3);
      p.x = x;
      p.y = y;
    }
  }
}

void fs::FsNode::initBlindArea()
{
  //转换到图像坐标系的形式下
  //3为A点与自车点C的横向距离
  // clang-format off
  cv::Point2f L1{+7.5 * 10 + HALF_GRID_MAP_SIZE, +0. * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f L2{-7.5 * 10 + HALF_GRID_MAP_SIZE, +0. * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f L3{-7.5 * 10 + HALF_GRID_MAP_SIZE, -(1.35 + BLIND_EXPAND) * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f L4{-10.176 * 10 + HALF_GRID_MAP_SIZE, -3.045 * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f L5{-10.176 * 10 + HALF_GRID_MAP_SIZE, +3.045 * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f L6{-10.176 * 10 + HALF_GRID_MAP_SIZE, +(3.045 + BLIND_EXPAND) * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f L7{+7.5 * 10 + HALF_GRID_MAP_SIZE, +(1.35 + BLIND_EXPAND) * 10 + HALF_GRID_MAP_SIZE};
  //  cv::Point B = Utils::findB(C, D, 3 * 10);
  //构造两个多边形，一左一右
  lidar_blind_l_poly_ = {L1, L2, L3, L4, L5, L6, L7};

  cv::Point2f R1{-7.5 * 10 + HALF_GRID_MAP_SIZE, -0. * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f R2{+7.5 * 10 + HALF_GRID_MAP_SIZE, -0. * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f R3{+7.5 * 10 + HALF_GRID_MAP_SIZE, +(1.35 + BLIND_EXPAND) * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f R4{+10.176 * 10 + HALF_GRID_MAP_SIZE, +(3.045) * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f R5{+10.176 * 10 + HALF_GRID_MAP_SIZE, -(3.045) * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f R6{+10.176 * 10 + HALF_GRID_MAP_SIZE, -(3.045 + BLIND_EXPAND) * 10 + HALF_GRID_MAP_SIZE};
  cv::Point2f R7{-7.5 * 10 + HALF_GRID_MAP_SIZE, -(1.35 + BLIND_EXPAND) * 10 + HALF_GRID_MAP_SIZE};
  //  cv::Point H         = Utils::findB(E, F, -3 * 10);
  lidar_blind_r_poly_ = {R1, R2, R3, R4, R5, R6, R7};
  // clang-format on

  //遍历整个栅格图，把所有盲区内的点找到，并赋值为1，不在盲区内的点赋值0
  for(int row = 0; row < GRID_MAP_SIZE; ++row)
  {
    for(int col = 0; col < GRID_MAP_SIZE; ++col)
    {
      lidar_blind_area_[row][col] = (Utils::pointInPoly(row, col, lidar_blind_l_poly_) || Utils::pointInPolygonQuick(row, col, lidar_blind_r_poly_));
    }
  }
}

void fs::FsNode::resetBuffers()
{
#if FS_CHECK(CFG_VIS_ENABLE)
  img_fw_buffer_.clear();
  img_fn_buffer_.clear();
  img_rw_buffer_.clear();
  img_rn_buffer_.clear();
  img_rl_buffer_.clear();
  img_rr_buffer_.clear();
  img_fl_buffer_.clear();
  img_fr_buffer_.clear();
#endif
#if FS_CHECK(CFG_USE_SVC)
  img_sfw_buffer_.clear();
  img_sfl_buffer_.clear();
  img_sfr_buffer_.clear();
  img_srl_buffer_.clear();
  img_srr_buffer_.clear();
  img_srw_buffer_.clear();
#endif
#if FS_CHECK(CFG_USE_CAM_FISH_FS)
  fs_sfw_buffer_.clear();
  fs_sfl_buffer_.clear();
  fs_sfr_buffer_.clear();
  fs_srl_buffer_.clear();
  fs_srr_buffer_.clear();
  fs_srw_buffer_.clear();
#endif
  emo_pos_buffer_.clear();
  fs_lidar_buffer_.clear();
  fs_fw_buffer_.clear();
  vot_objects_buffer_.clear();
}

fs::LidarFSPtr fs::FsNode::parseLidarFreespace(const Lidar_FreeSpace_v2 *lidar_fs_lcm_ptr)
{
  LidarFSPtr lidar_fs_ptr = std::make_shared<LidarFS>();
  lidar_fs_ptr->timestamp = lidar_fs_lcm_ptr->stHeader.nTimeStamp;
  lidar_fs_ptr->points.resize(lidar_fs_lcm_ptr->gstPoints.size());
  for(size_t i = 0; i < lidar_fs_lcm_ptr->gstPoints.size(); ++i)
  {
    lidar_fs_ptr->points[i] = FSPoint((lidar_fs_lcm_ptr->gstPoints[i].nRowIndex - lidar_fs_lcm_ptr->nVehicleOriginRows) * lidar_fs_lcm_ptr->fResolution,
                                      (-lidar_fs_lcm_ptr->gstPoints[i].nColIndex + lidar_fs_lcm_ptr->nVehicleOriginCols) * lidar_fs_lcm_ptr->fResolution,
                                      lidar_fs_lcm_ptr->gstPoints[i].nHight * 0.1f,
                                      0.,
                                      FS_LCM_LIDAR_CLASS_TO_BUILT_IN_CLASS.at(static_cast<int>(lidar_fs_lcm_ptr->gstPoints[i].nType)),
                                      lidar_fs_lcm_ptr->gstPoints[i].nType == 2 ? MotionState::MOVING : MotionState::STOPPED);
  }
  return lidar_fs_ptr;
}

void fs::FsNode::refineBevFs(int64_t timestamp, FS_BEV::CameraFreespace *bev_fs_lcm_ptr)
{
  if(bev_fs_lcm_ptr == nullptr)
  {
    return;
  }
  std::unique_ptr<EgoMotion> motion_Twb_lidar = nullptr;
  std::unique_ptr<EgoMotion> motion_Twb_bev = nullptr;
  if(getAlignedEgoMotion(timestamp, motion_Twb_lidar, emo_pos_buffer_) == 1 && getAlignedEgoMotion(bev_fs_lcm_ptr->measure_timestamp, motion_Twb_bev, emo_pos_buffer_) == 1)
  {
    const auto Twb_lidar = Utils::makeTFrom6Dof(motion_Twb_lidar->yaw, 0., 0., motion_Twb_lidar->translation.x(), motion_Twb_lidar->translation.y(), 0.);
    const auto Twb_bev = Utils::makeTFrom6Dof(motion_Twb_bev->yaw, 0., 0., motion_Twb_bev->translation.x(), motion_Twb_bev->translation.y(), 0.);
    FSMat4x4 Tcl = Utils::inverse(Twb_lidar) * Twb_bev;
    for(auto &&p : bev_fs_lcm_ptr->camera_freespace_points)
    {
      float x = Tcl(0, 0) * p.x + Tcl(0, 1) * p.y + Tcl(0, 3);
      float y = Tcl(1, 0) * p.x + Tcl(1, 1) * p.y + Tcl(1, 3);
      p.x = x;
      p.y = y;
    }
  }
}

void fs::FsNode::addBevFreespace(LidarFS *lidar_fs_ptr, const FS_BEV::CameraFreespace *bev_fs_lcm_ptr)
{
  if(bev_fs_lcm_ptr == nullptr || lidar_fs_ptr == nullptr)
  {
    return;
  }
  //如果valid为0则为无效，直接跳过
  if(bev_fs_lcm_ptr->valid == 0)
  {
    UWARN << "bev fs is invalid";
    return;
  }
  else
  {
    for(const auto &p : bev_fs_lcm_ptr->camera_freespace_points)
    {
      if(BEVCLASS_TO_CLASS.at(p.type) != GridLabel::UNOCCUPIED && lidar_blind_area_[(int)(p.x * GRID_SCALE_INV + HALF_GRID_MAP_SIZE)][(int)(p.y * GRID_SCALE_INV + HALF_GRID_MAP_SIZE)] == 1)
      {
        lidar_fs_ptr->points.emplace_back(p.x, p.y, 0, 0, BEVCLASS_TO_CLASS.at(p.type), MotionState::STOPPED);
      }
    }
  }
}

fs::ObjectState fs::FsNode::interpolateVcu(const int64_t timestamp, const VCU_VEHILCE_INFO *lhs_vcu, const VCU_VEHILCE_INFO *rhs_vcu)
{
  const float lhs_ratio =
    lhs_vcu->stHeader.nTimeStamp == rhs_vcu->stHeader.nTimeStamp ? 0.f : static_cast<float>(rhs_vcu->stHeader.nTimeStamp - timestamp) / (rhs_vcu->stHeader.nTimeStamp - lhs_vcu->stHeader.nTimeStamp);
  const float rhs_ratio = 1.f - lhs_ratio;
  auto interpolate = [lhs_ratio, rhs_ratio](const auto &lhsValue, const auto &rhsValue) { return lhsValue * lhs_ratio + rhsValue * rhs_ratio; };
  ObjectState ego_state{};
  ego_state.abs_vx = interpolate(lhs_vcu->fSpeed, rhs_vcu->fSpeed);
  ego_state.type = GridLabel::VEHICLE;
  return ego_state;
}

bool fs::FsNode::getAlignedVcu(int64_t timestamp, ObjectState &ego_state)
{
  auto guard = rw_lock_.read_guard();
  const auto vcu_rhs_iter =
    std::find_if(vcu_buffer_.begin(), vcu_buffer_.end(), [timestamp](const std::unique_ptr<VCU_VEHILCE_INFO> &pose) { return static_cast<int64_t>(pose->stHeader.nTimeStamp) >= timestamp; });
  const auto vcu_lhs_iter =
    std::find_if(vcu_buffer_.rbegin(), vcu_buffer_.rend(), [timestamp](const std::unique_ptr<VCU_VEHILCE_INFO> &pose) { return static_cast<int64_t>(pose->stHeader.nTimeStamp) <= timestamp; });
  if(vcu_lhs_iter == vcu_buffer_.rend() || vcu_rhs_iter == vcu_buffer_.end())
  {
    return false;
  }
  else
  {
    ego_state = interpolateVcu(timestamp, vcu_lhs_iter->get(), vcu_rhs_iter->get());
    return true;
  }
}

std::unique_ptr<FS_BEV_SEG::CameraFreespaceSeg> fs::FsNode::cloneBevFs(std::unique_ptr<FS_BEV_SEG::CameraFreespaceSeg> &bev_fs_ptr)
{
  if(bev_fs_ptr == nullptr)
  {
    return nullptr;
  }
  FS_BEV_SEG::CameraFreespaceSeg camera_fs{};
  camera_fs.measure_timestamp = bev_fs_ptr.get()->measure_timestamp;
  camera_fs.publish_timestamp = bev_fs_ptr.get()->publish_timestamp;
  camera_fs.bev_fs_seg_image_height = bev_fs_ptr.get()->bev_fs_seg_image_height;
  camera_fs.bev_fs_seg_image_width = bev_fs_ptr.get()->bev_fs_seg_image_width;
  camera_fs.bev_fs_seg_width_resolution = bev_fs_ptr.get()->bev_fs_seg_width_resolution;
  camera_fs.bev_fs_seg_height_resolution = bev_fs_ptr.get()->bev_fs_seg_height_resolution;
  camera_fs.bev_fs_seg_image_size = bev_fs_ptr.get()->bev_fs_seg_image_size;
  camera_fs.bev_fs_seg_image_data_label = bev_fs_ptr.get()->bev_fs_seg_image_data_label;
  camera_fs.bev_fs_seg_image_data_score = bev_fs_ptr.get()->bev_fs_seg_image_data_score;
  return std::make_unique<FS_BEV_SEG::CameraFreespaceSeg>(camera_fs);
}

bool fs::FsNode::getReverse()
{
  auto guard = rw_lock_.read_guard();
  if(!line_control_buffer_.empty() && line_control_buffer_.back()->nLaneDetectFront == 4)
  {
    UDEBUG << "----------------------is_reverse------------------- ";
    return true;
  }
  else
  {
    return false;
  }
}

void fs::FsNode::processVcuInfo(const void *data, uint32_t size)
{
  auto msg_ptr = std::make_unique<VCU_VEHILCE_INFO>();
  msg_ptr->decode(data, 0, size);

  if(nullptr != msg_ptr)
  {
    auto guard = rw_lock_.write_guard();
    // TODO: Process the error of yawRate from vcu
    msg_ptr->fYawRate += 0.01;
    vcu_buffer_.pushBackForce(std::move(msg_ptr));
  }
}

void fs::FsNode::processLaneControlInfo(const void *data, uint32_t size)
{
  auto msg_ptr = std::make_unique<NORAML_CAMERA_LANE_CONTROL>();
  msg_ptr->decode(data, 0, size);

  if(nullptr != msg_ptr)
  {
    auto guard = rw_lock_.write_guard();
    line_control_buffer_.pushBackForce(std::move(msg_ptr));
  }
}

template<class T>
void fs::FsNode::processImage(fs::RingBuffer<std::unique_ptr<T>, MSG_BUFFER_SIZE> &image_buffer, const void *data, uint32_t size)
{
  auto msg_ptr = std::make_unique<T>();
  msg_ptr->decode(data, 0, static_cast<int>(size));
  if(nullptr != msg_ptr)
  {
    auto guard = rw_lock_.write_guard();
    image_buffer.pushBackForce(std::move(msg_ptr));
  }
}

void fs::FsNode::processVehiclePose(const void *data, uint32_t size)
{
  static TRANSFORM transform;
  transform.decode(data, 0, static_cast<int>(size));
  {
    auto guard = rw_lock_.write_guard();
    {
      if(!init_flag_align_)
      {
        init_x_align_ = transform.gfVehiclePoseInWorld[0];
        init_y_align_ = transform.gfVehiclePoseInWorld[1];
        init_z_align_ = transform.gfVehiclePoseInWorld[2];
        init_flag_align_ = true;
      }

      EgoMotion &ego_motion = *emo_align_pos_buffer_.pushBackForce();
      ego_motion.timestamp = transform.stHeader.nTimeStamp;
      ego_motion.init_x = init_x_align_;
      ego_motion.init_y = init_y_align_;
      ego_motion.init_z = init_z_align_;
      ego_motion.translation.x() = transform.gfVehiclePoseInWorld[0] - init_x_align_; // -- loc
      ego_motion.translation.y() = transform.gfVehiclePoseInWorld[1] - init_y_align_;
      ego_motion.translation.z() = transform.gfVehiclePoseInWorld[2] - init_z_align_;
      ego_motion.yaw = transform.gfVehiclePoseInWorld[3];
      ego_motion.pitch = transform.gfVehiclePoseInWorld[4];
      ego_motion.roll = transform.gfVehiclePoseInWorld[5];
    }
    {
      if(!init_flag_)
      {
        init_x_ = transform.gfVehiclePoseInWorldVCUDR[0];
        init_y_ = transform.gfVehiclePoseInWorldVCUDR[1];
        init_z_ = transform.gfVehiclePoseInWorldVCUDR[2];
        init_flag_ = true;
      }

      EgoMotion &ego_motion = *emo_pos_buffer_.pushBackForce();
      ego_motion.timestamp = transform.stHeader.nTimeStamp;
      ego_motion.init_x = init_x_;
      ego_motion.init_y = init_y_;
      ego_motion.init_z = init_z_;
      ego_motion.translation.x() = transform.gfVehiclePoseInWorldVCUDR[0] - init_x_; // -- loc
      ego_motion.translation.y() = transform.gfVehiclePoseInWorldVCUDR[1] - init_y_;
      ego_motion.translation.z() = transform.gfVehiclePoseInWorldVCUDR[2] - init_z_;
      ego_motion.yaw = transform.gfVehiclePoseInWorldVCUDR[3];
      ego_motion.pitch = transform.gfVehiclePoseInWorldVCUDR[4];
      ego_motion.roll = transform.gfVehiclePoseInWorldVCUDR[5];
    }
  }
}

void fs::FsNode::processLocalization(const void *data, const uint32_t size)
{
  static IntegratedPosition integratedPosition;
  integratedPosition.decode(data, 0, size);
  {
    auto guard = rw_lock_.write_guard();
    if(integratedPosition.timestamp_micro_sec == emo_pos_buffer_.back().timestamp)
    {
      return;
    }
    Eigen::Vector2d pointInUtm = Utils::convertLLA2UTM(integratedPosition.longitude, integratedPosition.latitude);
    if(!init_flag_)
    {
      init_x_ = pointInUtm.x();
      init_y_ = pointInUtm.y();
      init_z_ = 0.;
      init_flag_ = true;
    }
    EgoMotion &ego_motion = *emo_pos_buffer_.pushBackForce();
    ego_motion.timestamp = integratedPosition.timestamp_micro_sec;
    ego_motion.init_x = init_x_;
    ego_motion.init_y = init_y_;
    ego_motion.init_z = init_z_;
    ego_motion.translation.x() = pointInUtm.x() - init_x_;
    ego_motion.translation.y() = pointInUtm.y() - init_y_;
    ego_motion.translation.z() = 0.0;
    ego_motion.roll = 0.0;
    ego_motion.pitch = 0.0;
    ego_motion.yaw = Utils::convertHeadingLLA2UTM(integratedPosition.longitude, integratedPosition.latitude, integratedPosition.yaw);
  }
}

void fs::FsNode::processLidarFreeSpace(const void *data, uint32_t size)
{
  auto msg_ptr = std::make_unique<Lidar_FreeSpace_v2>();
  msg_ptr->decode(data, 0, static_cast<int>(size));

  if(nullptr != msg_ptr)
  {
    auto guard = rw_lock_.write_guard();
    Diagnoser::instance().updateLidar();
    fs_lidar_buffer_.pushBackForce(std::move(msg_ptr));
    ++lidar_count_;
  }
}

void fs::FsNode::processBEVFreeSpace(const void *data, uint32_t size)
{
  auto msg_ptr = std::make_unique<FS_BEV_SEG::CameraFreespaceSeg>();
  msg_ptr->decode(data, 0, static_cast<int>(size));

  if(nullptr != msg_ptr)
  {
    auto guard = rw_lock_.write_guard();
    fs_bev_buffer_.pushBackForce(std::move(msg_ptr));
  }
}

void fs::FsNode::processFWFreeSpace(const void *data, uint32_t size)
{
  auto msg_ptr = std::make_unique<FS_BEV::CameraFreespace>();
  msg_ptr->decode(data, 0, static_cast<int>(size));

  if(nullptr != msg_ptr)
  {
    auto guard = rw_lock_.write_guard();
    fs_fw_buffer_.pushBackForce(std::move(msg_ptr));
  }
}

void fs::FsNode::processRWFreeSpace(const void *data, uint32_t size)
{
  auto msg_ptr = std::make_unique<FS_BEV::CameraFreespace>();
  msg_ptr->decode(data, 0, static_cast<int>(size));

  if(nullptr != msg_ptr)
  {
    auto guard = rw_lock_.write_guard();
    fs_rw_buffer_.pushBackForce(std::move(msg_ptr));
  }
}

void fs::FsNode::processLocMonitor(const void *data, uint32_t size)
{
  atd::monitor::ErrorCode2Monitor protoMsg;
  protoMsg.ParseFromArray(data, static_cast<int>(size));
  bool find_7003 = false;
  for(auto &e : protoMsg.errorcode())
  {
    if(e == 7003)
    {
      find_7003 = true;
      break;
    }
  }
  if(find_7003)
  {
    UERROR << "loc invalid";
    loc_flag_ = LocFlag::INVALID;
    loc_valid_first_time_ = 0;
    lidar_count_ = 0; // -- clear lidar count， prevent overflow
  }
  else
  {
    if(loc_valid_first_time_ == 0)
    {
      loc_valid_first_time_ = protoMsg.timestamp();
    }
    else
    {
      loc_flag_ = (protoMsg.timestamp() - loc_valid_first_time_) > 300 * TIME_SCALE_MS && lidar_count_ > 2 ? LocFlag::VALID : LocFlag::LOW_VALID;
    }
  }
}

void fs::FsNode::processLane(const void *data, uint32_t size)
{
  auto msg_ptr = std::make_unique<PERCEPTION_LINES>();
  msg_ptr->decode(data, 0, static_cast<int>(size));

  if(nullptr != msg_ptr)
  {
    auto guard = rw_lock_.write_guard();
    lanes_buffer_.pushBackForce(std::move(msg_ptr));
  }
}

void fs::FsNode::publishMonitorMsg()
{
  const Diagnoser &diagnoser = Diagnoser::getDiagnoser();
  atd::monitor::ErrorCode2Monitor SensorErrorCode{};

  if(diagnoser.isLidarInvalid())
  {
    SensorErrorCode.add_errorcode(5004);
  }

  // vot lost
  if(diagnoser.isVOTInvalid())
  {
    SensorErrorCode.add_errorcode(5048);
  }

  const auto getTime = this->get_clock()->now().nanoseconds() / 1000;
  SensorErrorCode.set_timestamp(getTime);
  std::string SensorErrorCodeData;
  SensorErrorCode.SerializeToString(&SensorErrorCodeData);
  m_lcmHandler_Error_Code.publish("freespace2Monitor", SensorErrorCodeData.data(), SensorErrorCodeData.size());
}

std::vector<cv::Point2f> fs::FsNode::getLanePolygon(LidarFS *lidar_fs_ptr, const PERCEPTION_LINES *perception_lines)
{
  if(lidar_fs_ptr == nullptr || perception_lines == nullptr)
  {
    return {};
  }

  // -- 1. tf latest ego
  std::unique_ptr<EgoMotion> emo_pos_Twb_lidar = nullptr;
  std::unique_ptr<EgoMotion> emo_pos_Twb_lines = nullptr;
  if(getAlignedEgoMotion(lidar_fs_ptr->timestamp, emo_pos_Twb_lidar, emo_pos_buffer_) == 1 && getAlignedEgoMotion(perception_lines->stHeader.nTimeStamp, emo_pos_Twb_lines, emo_pos_buffer_) == 1)
  {
    const auto Twb_lidar = Utils::makeTFrom6Dof(emo_pos_Twb_lidar->yaw, 0., 0., emo_pos_Twb_lidar->translation.x(), emo_pos_Twb_lidar->translation.y(), 0.);
    const auto Twb_lines = Utils::makeTFrom6Dof(emo_pos_Twb_lines->yaw, 0., 0., emo_pos_Twb_lines->translation.x(), emo_pos_Twb_lines->translation.y(), 0.);
    FSMat4x4 Tcl = Utils::inverse(Twb_lidar) * Twb_lines;

    //     1     2
    //     |     |
    //     |     |
    //     0     3
    std::vector<cv::Point2f> line{cv::Point2f(std::numeric_limits<float>::max(), 0.f),
                                  -cv::Point2f(std::numeric_limits<float>::max(), 0.f),
                                  -cv::Point2f(std::numeric_limits<float>::max(), 0.f),
                                  cv::Point2f(std::numeric_limits<float>::max(), 0.f)};
    if(!perception_lines->bLeftLine || !perception_lines->bRightLine)
    {
      return {};
    }

    if(perception_lines->bLeftLine)
    {
      for(int i = 0; i < 300; ++i)
      {
        if(perception_lines->gfLeftLineX[i] > REMOVE_REGION_BACK && perception_lines->gfLeftLineX[i] < REMOVE_REGION_FRONT)
        {
          float x = Tcl(0, 0) * perception_lines->gfLeftLineX[i] + Tcl(0, 1) * perception_lines->gfLeftLineY[i] + Tcl(0, 3);
          float y = Tcl(1, 0) * perception_lines->gfLeftLineX[i] + Tcl(1, 1) * perception_lines->gfLeftLineY[i] + Tcl(1, 3);
          if(x < line[0].x)
          {
            line[0].x = x;
            line[0].y = y;
          }
          if(x > line[1].x)
          {
            line[1].x = x;
            line[1].y = y;
          }
        }
      }
    }

    if(perception_lines->bRightLine)
    {
      for(int i = 0; i < 300; ++i)
      {
        if(perception_lines->gfRightLineX[i] > REMOVE_REGION_BACK && perception_lines->gfRightLineX[i] < REMOVE_REGION_FRONT)
        {
          float x = Tcl(0, 0) * perception_lines->gfRightLineX[i] + Tcl(0, 1) * perception_lines->gfRightLineY[i] + Tcl(0, 3);
          float y = Tcl(1, 0) * perception_lines->gfRightLineX[i] + Tcl(1, 1) * perception_lines->gfRightLineY[i] + Tcl(1, 3);
          if(x > line[2].x)
          {
            line[2].x = x;
            line[2].y = y;
          }
          if(x < line[3].x)
          {
            line[3].x = x;
            line[3].y = y;
          }
        }
      }
    }
    line[0].y += 0.075; // -- 7.5cm
    line[1].y += 0.075;
    line[2].y -= 0.075;
    line[3].y -= 0.075;
    return line;
  }
  return {};
}
#endif // -- CFG_ROS2

fs::EgoMotion fs::FsNode::interpolateEgoMotion(const int64_t timestamp, const fs::EgoMotion &lhs_ego_motion, const fs::EgoMotion &rhs_ego_motion)
{
  const float lhs_ratio   = lhs_ego_motion.timestamp == rhs_ego_motion.timestamp ? 0.f : static_cast<float>(rhs_ego_motion.timestamp - timestamp) / (rhs_ego_motion.timestamp - lhs_ego_motion.timestamp);
  const float rhs_ratio   = 1.f - lhs_ratio;
  auto        interpolate = [lhs_ratio, rhs_ratio](const auto &lhs_value, const auto &rhs_value) { return lhs_value * lhs_ratio + rhs_value * rhs_ratio; };

  std::array<float, 2> yaw_array{lhs_ego_motion.yaw, rhs_ego_motion.yaw};
  if(std::abs(lhs_ego_motion.yaw - rhs_ego_motion.yaw) > M_PIf32)
  {
    *std::min_element(yaw_array.begin(), yaw_array.end()) += 2.f * M_PIf32;
  }

  EgoMotion output_ego_motion   = lhs_ego_motion;
  output_ego_motion.timestamp   = timestamp;
  output_ego_motion.translation = interpolate(lhs_ego_motion.translation, rhs_ego_motion.translation);
  output_ego_motion.yaw         = interpolate(yaw_array[0], yaw_array[1]); // be careful, the lhs yaw must in yaw_array[0]
  output_ego_motion.pitch       = interpolate(lhs_ego_motion.pitch, rhs_ego_motion.pitch);
  output_ego_motion.roll        = interpolate(lhs_ego_motion.roll, rhs_ego_motion.roll);

  return output_ego_motion;
}

bool fs::FsNode::getAlignedEgoMotion(int64_t timestamp, std::unique_ptr<EgoMotion> &ego_motion_ptr, const fs::RingBuffer<EgoMotion, EMO_BUFFER_SIZE> &emo_buffer)
{
  auto guard = rw_lock_.read_guard();
  return getAlignedEgoMotionNoLock(timestamp, ego_motion_ptr, emo_buffer);
}

bool fs::FsNode::getAlignedEgoMotionNoLock(int64_t timestamp, std::unique_ptr<EgoMotion> &ego_motion_ptr, const fs::RingBuffer<EgoMotion, EMO_BUFFER_SIZE> &emo_buffer)
{
  const auto emo_rhs_iter = std::find_if(emo_buffer.begin(), emo_buffer.end(), [timestamp](const EgoMotion &emo) { return static_cast<int64_t>(emo.timestamp) >= timestamp; });
  const auto emo_lhs_iter = std::find_if(emo_buffer.rbegin(), emo_buffer.rend(), [timestamp](const EgoMotion &emo) { return static_cast<int64_t>(emo.timestamp) <= timestamp; });
  if(emo_lhs_iter == emo_buffer.rend() || emo_rhs_iter == emo_buffer.end())
  {
    return false;
  }
  else
  {
    if(emo_lhs_iter->is_valid && emo_rhs_iter->is_valid)
    {
      ego_motion_ptr = std::make_unique<EgoMotion>(interpolateEgoMotion(timestamp, *emo_lhs_iter, *emo_rhs_iter));
      return true;
    }
    else
    {
      return false;
    }
  }
  return false;
}

std::unique_ptr<fs::EgoMotion> fs::FsNode::getLatestEgoMotion(const fs::RingBuffer<EgoMotion, EMO_BUFFER_SIZE> &emo_buffer)
{
  auto      guard = rw_lock_.read_guard();
  EgoMotion output_ego_motion{};
  if(!emo_buffer.empty())
  {
    const auto &lpose             = emo_buffer.back();
    output_ego_motion.timestamp   = lpose.timestamp;
    output_ego_motion.translation = lpose.translation;
    output_ego_motion.yaw         = lpose.yaw; // be careful, the lhs yaw must in yaw_array[0]
    output_ego_motion.pitch       = lpose.pitch;
    output_ego_motion.roll        = lpose.roll;
  }
  return std::make_unique<EgoMotion>(output_ego_motion);
}

void fs::FsNode::refineVotObject(const LidarFS *lidar_fs_ptr, ObjectStates *vot_object_ptr)
{
  if(lidar_fs_ptr == nullptr || vot_object_ptr == nullptr)
  {
    return;
  }
  std::unique_ptr<EgoMotion> emo_pos_Twb_lidar = nullptr;
  std::unique_ptr<EgoMotion> emo_pos_Twb_vot   = nullptr;
  if(getAlignedEgoMotion(lidar_fs_ptr->timestamp, emo_pos_Twb_lidar, emo_pos_buffer_) == 1 && getAlignedEgoMotion(vot_object_ptr->timestamp, emo_pos_Twb_vot, emo_pos_buffer_) == 1)
  {
    const auto Twb_lidar = Utils::makeTFrom6Dof(emo_pos_Twb_lidar->yaw, 0., 0., emo_pos_Twb_lidar->translation.x(), emo_pos_Twb_lidar->translation.y(), 0.);
    const auto Twb_vot   = Utils::makeTFrom6Dof(emo_pos_Twb_vot->yaw, 0., 0., emo_pos_Twb_vot->translation.x(), emo_pos_Twb_vot->translation.y(), 0.);
    // -- 1. tf vot to utm
    for(auto &&ob : vot_object_ptr->objects)
    {
      float x    = ob.x * Twb_vot(0, 0) + ob.y * Twb_vot(0, 1) + Twb_vot(0, 3);
      float y    = ob.x * Twb_vot(1, 0) + ob.y * Twb_vot(1, 1) + Twb_vot(1, 3);
      ob.real_vx = ob.abs_vx * Twb_vot(0, 0) + ob.abs_vy * Twb_vot(0, 1);
      ob.real_vy = ob.abs_vx * Twb_vot(1, 0) + ob.abs_vy * Twb_vot(1, 1);
      ob.x       = x;
      ob.y       = y;
    }
    // -- 2. predict
    const float dt = static_cast<float>(lidar_fs_ptr->timestamp - vot_object_ptr->timestamp) / (TIME_SCALE_MS * 1000);
    for(auto &&ob : vot_object_ptr->objects)
    {
      ob.x += ob.real_vx * dt;
      ob.y += ob.real_vy * dt;
    }
    // -- 3. tf vot from utm to lidar
    FSMat4x4 Tbw_lidar = Utils::inverse(Twb_lidar);
    for(auto &&ob : vot_object_ptr->objects)
    {
      float x = ob.x * Tbw_lidar(0, 0) + ob.y * Tbw_lidar(0, 1) + Tbw_lidar(0, 3);
      float y = ob.x * Tbw_lidar(1, 0) + ob.y * Tbw_lidar(1, 1) + Tbw_lidar(1, 3);
      ob.x    = x;
      ob.y    = y;
    }
  }
}

bool fs::FsNode::isLockStation(const fs::ObjectStatesPtr &vot_ptr)
{
  if(vot_ptr == nullptr)
  {
    return false;
  }
  int                      Y_thre = 6;
  int                      X_thre = 30;
  std::vector<ObjectState> lock_left{};
  std::vector<ObjectState> lock_right{};
  for(const auto &gstObject : vot_ptr->objects)
  {
    if(gstObject.type == GridLabel::LOCK_STATION)
    {
      if(gstObject.y > 0 && gstObject.y < Y_thre && fabs(gstObject.x) < X_thre)
      {
        lock_left.push_back(gstObject);
      }
      if(gstObject.y < 0 && gstObject.y > -Y_thre && fabs(gstObject.x) < X_thre)
      {
        lock_right.push_back(gstObject);
      }
    }
  }
  for(const auto &l : lock_left)
  {
    for(const auto &r : lock_right)
    {
      if(std::fabs(l.x - r.x) < 3)
      {
        return true;
      }
    }
  }
  return false;
}

#if FS_CHECK(CFG_ROS2)
void fs::FsNode::syncFsTF(CamFS                                                                         &fs,
                          fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE> &fs_buffer,
                          const fs::RingBuffer<EgoMotion, EMO_BUFFER_SIZE>                              &emo_buffer,
                          int                                                                            left,
                          int                                                                            right)

{
  auto fsIter = std::find_if(fs_buffer.cbegin(), fs_buffer.cend(), [this, left, right](const std::unique_ptr<uto::proto::CameraFreespace> &ptr) { return ptr->header().time_meas() > fusion_timestamp_ - left * TIME_SCALE_MS && ptr->header().time_meas() < fusion_timestamp_ + right * TIME_SCALE_MS; });
  if(fsIter != fs_buffer.cend())
  {
    const auto timestamp                         = fsIter->get()->header().time_meas();
    fs.timestamp                                 = timestamp;
    fs.points_ptr                                = parseCameraFreespace(fsIter->get());
    std::unique_ptr<EgoMotion> emo_pos_Twb_cam   = nullptr;
    std::unique_ptr<EgoMotion> emo_pos_Twb_lidar = nullptr;
    if(fs.points_ptr != nullptr && getAlignedEgoMotionNoLock(fs.timestamp, emo_pos_Twb_cam, emo_buffer) && getAlignedEgoMotionNoLock(fusion_timestamp_, emo_pos_Twb_lidar, emo_buffer))
    {
      const auto Twb_fw    = Utils::makeTFrom6Dof(emo_pos_Twb_cam->yaw, emo_pos_Twb_cam->pitch, emo_pos_Twb_cam->roll, emo_pos_Twb_cam->translation.x(), emo_pos_Twb_cam->translation.y(), emo_pos_Twb_cam->translation.z());
      const auto Twb_lidar = Utils::makeTFrom6Dof(emo_pos_Twb_lidar->yaw, emo_pos_Twb_lidar->pitch, emo_pos_Twb_lidar->roll, emo_pos_Twb_lidar->translation.x(), emo_pos_Twb_lidar->translation.y(), emo_pos_Twb_lidar->translation.z());
      fs.T_cam2lidar       = Utils::inverse(Twb_fw) * Twb_lidar;
    }

    while(fs_buffer.cbegin() < fsIter + 1)
    {
      fs_buffer.popFront();
    }
  }
}

#if FS_CHECK(CFG_VIS_ENABLE)
void fs::FsNode::syncImg(std::unique_ptr<sensor_msgs::msg::CompressedImage>                                  &img,
                         CamFS                                                                               &fs,
                         fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> &img_buffer)
{
  if (fs.points_ptr == nullptr)
  {
    // If points_ptr is null, use the latest available image from the buffer
    if (!img_buffer.empty())
    {
      img = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_buffer.back());
    }
  }
  else
  {
    const auto timestamp = fs.timestamp;

    // Find the image with a timestamp closest to the current timestamp
    auto imgIter = std::min_element(img_buffer.cbegin(), img_buffer.cend(), [timestamp](const auto &ptr1, const auto &ptr2) {
        const auto time1 = ptr1->header.stamp.sec * INT_1E9 + ptr1->header.stamp.nanosec;
        const auto time2 = ptr2->header.stamp.sec * INT_1E9 + ptr2->header.stamp.nanosec;
        return std::abs(time1 - timestamp) < std::abs(time2 - timestamp);
    });

    if (imgIter != img_buffer.cend())
    {
      img = std::make_unique<sensor_msgs::msg::CompressedImage>(**imgIter);
    }
  }
}

//
//void fs::FsNode::syncImg(std::unique_ptr<sensor_msgs::msg::CompressedImage>                                  &img,
//                         CamFS                                                                               &fs,
//                         fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> &img_buffer)
//{
//  if(fs.points_ptr == nullptr)
//  {
//    if(!img_buffer.empty())
//    {
//      img = std::make_unique<sensor_msgs::msg::CompressedImage>(*img_buffer.back());
//    }
//  }
//  else
//  {
//    const auto timestamp = fs.timestamp;
//    auto       imgIter   = std::find_if(img_buffer.cbegin(), img_buffer.cend(), [timestamp](const std::unique_ptr<sensor_msgs::msg::CompressedImage> &ptr) { return ptr->header.stamp.sec * INT_1E9 + ptr->header.stamp.nanosec == timestamp; });
//    if(imgIter != img_buffer.cend())
//    {
//      img = std::move(*imgIter);
//      while(img_buffer.cbegin() < imgIter + 1)
//      {
//        img_buffer.popFront();
//      }
//    }
//  }
//}
#endif // -- CFG_VIS_ENABLE
#endif

#if FS_CHECK(CFG_VIS_ENABLE)

void fs::FsNode::publishVis()
{
  // -- 1. 在 manager 里调用 draw
  // -- 2. 在 node    里调用 publish
  // -- 3. 数据存在 vis 内
  auto header     = std_msgs::msg::Header();
  header.frame_id = "ego";

#if !FS_CHECK(CFG_ROS2)
  Vis::drawBlindPointCloud(lidar_blind_area_);
  blind_cld_pub_ptr_->publish(Vis::getBlindPointCloud());
  Vis::drawBlindMakers(lidar_blind_l_poly_, lidar_blind_r_poly_);
  bev_cld_pub_ptr_->publish(Vis::getBevPointCloud()); // -- current bev dectect
#endif
#if FS_CHECK(CFG_USE_MCAP_FUSION)
  if(fusion_timestamp_ != 0)
  {
    rclcpp::Time curr_timestamp(fusion_timestamp_);
    this->m_bagWriterPtr->write(Vis::getMarkerArray(), "/fs/marker_array_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(Vis::getBlindArray(), "/fs/blind_marker_array_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(Vis::getPolygonPointCloud(), "/fs/polygon_cloud", curr_timestamp);
    this->m_bagWriterPtr->write(Vis::getColorTablePointCloud(), "/fs/color_table_point_cloud", curr_timestamp);
    this->m_bagWriterPtr->write(Vis::getOutBboxPointCloud(), "/fs/out_bbox_point_cloud", curr_timestamp);
    geometry_msgs::msg::TransformStamped tf_msg = generate_transform();
    this->m_bagWriterPtr->write(tf_msg, "/tf", curr_timestamp);
    this->m_bagWriterPtr->write(Vis::getLidarPointCloud2(), "/fs/lidar_point_cloud2", curr_timestamp);
    this->m_bagWriterPtr->write(Vis::getRoadPointCloud2(), "/fs/road_model_point_cloud2", curr_timestamp);
    this->m_bagWriterPtr->write(generate_time_log(curr_timestamp), "/stamp", curr_timestamp);

    for(auto &&pub : Vis::getSelfReceivePointCloud2())
    {
      if(pub.first == fs::GridLabel::OTHER)
      {
        this->m_bagWriterPtr->write(pub.second, "/fs/cld_self_receive_OTHER2", curr_timestamp);
      }
      if(pub.first == fs::GridLabel::PEDESTRIAN)
      {
        this->m_bagWriterPtr->write(pub.second, "/fs/cld_self_receive_PEDESTRIAN2", curr_timestamp);
      }
      if(pub.first == fs::GridLabel::CYCLIST)
      {
        this->m_bagWriterPtr->write(pub.second, "/fs/cld_self_receive_CYCLIST2", curr_timestamp);
      }
      if(pub.first == fs::GridLabel::VEHICLE)
      {
        this->m_bagWriterPtr->write(pub.second, "/fs/cld_self_receive_VEHICLE2", curr_timestamp);
      }
      if(pub.first == fs::GridLabel::RTG_QC)
      {
        this->m_bagWriterPtr->write(pub.second, "/fs/cld_self_receive_RTG_QC2", curr_timestamp);
      }
      if(pub.first == fs::GridLabel::LOCK_BOX)
      {
        this->m_bagWriterPtr->write(pub.second, "/fs/cld_self_receive_LOCK_BOX2", curr_timestamp);
      }
      if(pub.first == fs::GridLabel::CONE)
      {
        this->m_bagWriterPtr->write(pub.second, "/fs/cld_self_receive_CONE2", curr_timestamp);
      }
      if(pub.first == fs::GridLabel::AIV)
      {
        this->m_bagWriterPtr->write(pub.second, "/fs/cld_self_receive_AIV2", curr_timestamp);
      }
      if(pub.first == fs::GridLabel::TREE_BRANCH)
      {
        this->m_bagWriterPtr->write(pub.second, "/fs/cld_self_receive_TREE_BRANCH2", curr_timestamp);
      }
    }

    std::shared_ptr<sensor_msgs::msg::CompressedImage> fw_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getView(SensorId::CAMERA_FW)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> fn_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getView(SensorId::CAMERA_FN)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> fl_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getView(SensorId::CAMERA_FL)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> fr_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getView(SensorId::CAMERA_FR)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> rw_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getView(SensorId::CAMERA_RW)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> rn_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getView(SensorId::CAMERA_RN)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> rl_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getView(SensorId::CAMERA_RL)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> rr_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getView(SensorId::CAMERA_RR)).toCompressedImageMsg();

    std::shared_ptr<sensor_msgs::msg::CompressedImage> sfw_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getCylinderView(CySensorId::CAMERA_CFW)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> sfl_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getCylinderView(CySensorId::CAMERA_CFL)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> sfr_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getCylinderView(CySensorId::CAMERA_CFR)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> srl_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getCylinderView(CySensorId::CAMERA_CRL)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> srr_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getCylinderView(CySensorId::CAMERA_CRR)).toCompressedImageMsg();
    std::shared_ptr<sensor_msgs::msg::CompressedImage> srw_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getCylinderView(CySensorId::CAMERA_CRW)).toCompressedImageMsg();

    this->m_bagWriterPtr->write(*fw_img_msg, "/fs/FW_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*fn_img_msg, "/fs/FN_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*fl_img_msg, "/fs/FL_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*fr_img_msg, "/fs/FR_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*rw_img_msg, "/fs/RW_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*rn_img_msg, "/fs/RN_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*rl_img_msg, "/fs/RL_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*rr_img_msg, "/fs/RR_canvas", curr_timestamp);

    this->m_bagWriterPtr->write(*sfw_img_msg, "/fs/SFW_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*sfl_img_msg, "/fs/SFL_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*sfr_img_msg, "/fs/SFR_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*srl_img_msg, "/fs/SRL_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*srr_img_msg, "/fs/SRR_canvas", curr_timestamp);
    this->m_bagWriterPtr->write(*srw_img_msg, "/fs/SRW_canvas", curr_timestamp);
  }
#else // -- CFG_USE_MCAP_FUSION
  Vis::drawZone(Expand::instance().getZoneBox());
  marker_array_pub_ptr_->publish(Vis::getMarkerArray());                 // -- marker
  blind_marker_array_pub_ptr_->publish(Vis::getBlindArray());            // -- blind area
  polygon_cld_pub_ptr_->publish(Vis::getPolygonPointCloud());            // -- current lidar dectect
  lidar_cld_history_pub_ptr_->publish(Vis::getLidarPointCloudHistory()); // -- history tracking
  color_table_cld_pub_ptr_->publish(Vis::getColorTablePointCloud());     // -- color table
  out_bbox_cld_pub_ptr_->publish(Vis::getOutBboxPointCloud());
#if FS_CHECK(CFG_USE_FOXGLOV_VIS)
  lidar_cld_pub_ptr2_->publish(Vis::getLidarPointCloud2()); // -- current lidar dectect
  for(auto &pub : map_pub_built_in2_)
  {
    pub.second->publish(Vis::getGridPointCloudSplit2().at(pub.first));
  }
#else
  lidar_cld_pub_ptr_->publish(Vis::getLidarPointCloud()); // -- current lidar dectect
  road_cld_pub_ptr_->publish(Vis::getRoadPointCloud());   // -- current road dectect
  for(auto &pub : map_pub_built_in_)
  {
    pub.second->publish(Vis::getGridPointCloudSplit().at(pub.first));
  }
#endif
#if FS_CHECK(CFG_ROS2)
#if FS_CHECK(CFG_USE_FOXGLOV_VIS)
  for(auto &pub : map_pub_self_receive2_)
  {
    pub.second->publish(Vis::getSelfReceivePointCloud2().at(pub.first));
  }
#else
  for(auto &pub : map_pub_self_receive_)
  {
    pub.second->publish(Vis::getSelfReceivePointCloud().at(pub.first));
  }
#endif
#else
#if FS_CHECK(!CFG_DEBUG_PERCEPTION_FREESPACES)
#if FS_CHECK(CFG_USE_FOXGLOV_VIS)
  for(auto &pub : map_pub_self_receive2_)
  {
    pub.second->publish(Vis::getSelfReceivePointCloud2().at(pub.first));
  }
#else
  for(auto &pub : map_pub_self_receive_)
  {
    pub.second->publish(Vis::getSelfReceivePointCloud().at(pub.first));
  }
#endif
#endif
#endif

#if !FS_CHECK(CFG_ROS2)
  ego_bbox_cld_pub_ptr_->publish(Vis::getegoBboxPointCloud()); // -- ego region
#endif //  -- CFG_ROS2
  sensor_msgs::msg::Image::SharedPtr bev_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getBEVView()).toImageMsg();
  sensor_msgs::msg::Image::SharedPtr remap_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getRemapView()).toImageMsg();
  //zylinder example
  // sensor_msgs::msg::Image::SharedPtr zylinder_img_msg = cv_bridge::CvImage(header, "bgr8", Vis::getZYlinderView()).toImageMsg();
  for(auto &pub : map_pub_img_)
  {
    pub.second->publish(*(cv_bridge::CvImage(header, "bgr8", Vis::getView(pub.first)).toImageMsg()));
  }

#if FS_CHECK(CFG_USE_SVC)
  for(auto &pub : cy_map_pub_img_)
  {
    pub.second->publish(*(cv_bridge::CvImage(header, "bgr8", Vis::getCylinderView(pub.first)).toImageMsg()));
  }
#endif
  //zylinder example
  // zylinder_img_pub_ptr_->publish(*zylinder_img_msg);
  bev_img_pub_ptr_->publish(*bev_img_msg);
  remap_img_pub_ptr_->publish(*remap_img_msg);
#endif
}

#endif // -- CFG_VIS_ENABLE