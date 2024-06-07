#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <common/ring_buffer.h>
#include <common/publisher.h>
#include <common/subscriber.h>
#include <camera_calib.pb.h>
#include <perception_camera.pb.h>
#include <sensor_msgs/msg/image.hpp>
#include <locator_vehicle_pose.pb.h>
#include <perception_freespace.pb.h>
#include <perception_signal.pb.h>
#include <sensor_table.pb.h>
#include <mechanical_info.pb.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <error_code.pb.h>
#include <rosbag2_cpp/writer.hpp>

#include "rwlock.h"
#include "debug/vis.h"
#include "defs.h"
#include "types.h"
#include "macro.h"
#include "core/freespace_manager.h"

namespace fs
{
  class FsNode final : public rclcpp::Node
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(FsNode);

    FsNode():
      Node(NODE_NAME) { init(); }

  private:
    void init();
    void process();
    void loadBag();
    void resetBuffers();
    void removeObsoleteInputs();

    // clang-format off
    void subscribeVehiclePose(std::unique_ptr<uto::proto::VehiclePose> msgPtr) { processVehiclePose(*msgPtr); }
    void subscribeCameraFs(std::unique_ptr<uto::proto::CameraFreespace> msgPtr) { processCameraFs(m_fsCfBuffer, msgPtr, SensorId::CAMERA_CENTER_FRONT); }
#if FS_CHECK(CFG_USE_FISHEYE_FS)
    void subscribeFsFisheyeFcf(std::unique_ptr<uto::proto::CameraFreespace> msgPtr) { processCameraFs(m_fsFisheyeFcfBuffer, msgPtr, SensorId::FISHEYE_FCF); }
    void subscribeFsFisheyeFll(std::unique_ptr<uto::proto::CameraFreespace> msgPtr) { processCameraFs(m_fsFisheyeFcfBuffer, msgPtr, SensorId::FISHEYE_FLL); }
    void subscribeFsFisheyeFrr(std::unique_ptr<uto::proto::CameraFreespace> msgPtr) { processCameraFs(m_fsFisheyeFcfBuffer, msgPtr, SensorId::FISHEYE_FRR); }
    void subscribeFsFisheyeBcb(std::unique_ptr<uto::proto::CameraFreespace> msgPtr) { processCameraFs(m_fsFisheyeFcfBuffer, msgPtr, SensorId::FISHEYE_BCB); }
    void subscribeFsFisheyeBll(std::unique_ptr<uto::proto::CameraFreespace> msgPtr) { processCameraFs(m_fsFisheyeFcfBuffer, msgPtr, SensorId::FISHEYE_BLL); }
    void subscribeFsFisheyeBrr(std::unique_ptr<uto::proto::CameraFreespace> msgPtr) { processCameraFs(m_fsFisheyeFcfBuffer, msgPtr, SensorId::FISHEYE_BRR); }
#endif
    void subscribeLidarFs(std::unique_ptr<uto::proto::PerceptionFreespace> msgPtr) { processLidarFs(msgPtr); }
    void subscribeOD(std::unique_ptr<uto::proto::PerceptionObstacles> msgPtr) { processOd(msgPtr); }
    void subscribeImageCf(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr);
    void subscribeImageFisheyeFcf(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr);
    void subscribeImageFisheyeFll(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr);
    void subscribeImageFisheyeFrr(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr);
    void subscribeImageFisheyeBcb(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr);
    void subscribeImageFisheyeBll(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr);
    void subscribeImageFisheyeBrr(std::unique_ptr<sensor_msgs::msg::CompressedImage> msgPtr);
    void subscribeCalibCf(std::unique_ptr<uto::proto::CameraCalib> msgPtr);
    void subscribeCalibFisheyeFcf(std::unique_ptr<uto::proto::CameraCalib> msgPtr);
    void subscribeCalibFisheyeFll(std::unique_ptr<uto::proto::CameraCalib> msgPtr);
    void subscribeCalibFisheyeFrr(std::unique_ptr<uto::proto::CameraCalib> msgPtr);
    void subscribeCalibFisheyeBcb(std::unique_ptr<uto::proto::CameraCalib> msgPtr);
    void subscribeCalibFisheyeBll(std::unique_ptr<uto::proto::CameraCalib> msgPtr);
    void subscribeCalibFisheyeBrr(std::unique_ptr<uto::proto::CameraCalib> msgPtr);
    void subscribeGates(std::unique_ptr<uto::proto::PerceptionGates> msgPtr);
    void subscribeMechanicalInfo(std::unique_ptr<uto::proto::MechanicalInfo> msgPtr);
    void subscribeSensorTable(std::unique_ptr<uto::proto::SensorTable> msgPtr);
    void subscribePerceptionFreespace(std::unique_ptr<uto::proto::PerceptionFreespace> msgPtr);
    // clang-format on

    void processVehiclePose(const uto::proto::VehiclePose& vehiclePose);
    void processCameraFs(fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>& camFsBuffer, std::unique_ptr<uto::proto::CameraFreespace>& msgPtr, const SensorId sensorId);
    void processLidarFs(std::unique_ptr<uto::proto::PerceptionFreespace>& msgPtr);
    void processOd(std::unique_ptr<uto::proto::PerceptionObstacles>& msgPtr);
    void publishDiagnosisMsg();

    bool getAlignedInputs(std::unique_ptr<uto::proto::PerceptionFreespace>&   fsLidarPtr,
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
                          std::unique_ptr<EgoMotion>&                         egoMotionPtr);

    void broadcastTF();
    void publishFusionFreespace(const uto::proto::PerceptionFreespace& lidarFsPtr);
    void publishVis();
    void publishVisPerceptionFreespace(const uto::proto::PerceptionFreespace& perceptionFreespace);

    static std::string getOutFilePath(const std::string& pathOut, const std::string& bagPath);

    RWLock                               m_rwLock;
    rclcpp::TimerBase::SharedPtr         m_timer = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(PROCESS_PERIOD_MS), [this] { process(); });
    std::unique_ptr<rosbag2_cpp::Reader> m_bagReaderPtr{std::make_unique<rosbag2_cpp::Reader>()};
    int64_t                              m_fusionTimestampNs         = 0;
    int                                  m_vehicleFrameCount         = 0;
    int                                  m_abnormalCount             = ABNORMAL_COUNT_THRESH;
    int64_t                              m_cameraLastSubTimestampNs  = 0;
    int64_t                              m_fisheyeLastSubTimestampNs = 0;
    int64_t                              m_lidarLastSubTimestampNs   = 0;
    int64_t                              m_odLastSubTimestampNs      = 0;

    FreespaceManager m_freespaceManager;

    // message buffers
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> m_imageCfBuffer;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> m_imageFisheyeFcfBuffer;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> m_imageFisheyeFllBuffer;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> m_imageFisheyeFrrBuffer;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> m_imageFisheyeBcbBuffer;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> m_imageFisheyeBllBuffer;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> m_imageFisheyeBrrBuffer;
    fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionFreespace>, MSG_BUFFER_SIZE>   m_lidarFsBuffer;
    fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionGates>, MSG_BUFFER_SIZE>       m_gatesBuffer;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>       m_fsCfBuffer;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>       m_fsFisheyeFcfBuffer;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>       m_fsFisheyeFllBuffer;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>       m_fsFisheyeFrrBuffer;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>       m_fsFisheyeBcbBuffer;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>       m_fsFisheyeBllBuffer;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>       m_fsFisheyeBrrBuffer;
    fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionObstacles>, MSG_BUFFER_SIZE>   m_odBuffer;

    // subscribers
    uto::Subscriber<uto::proto::VehiclePose>           m_vehiclePoseSub{VEHICLE_POSE_TOPIC, this, [this](auto&& PH1) { subscribeVehiclePose(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::PerceptionFreespace>   m_lidarFsSub{LIDAR_FS_TOPIC, this, [this](auto&& PH1) { subscribeLidarFs(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::PerceptionObstacles>   m_odSub{OD_TOPIC, this, [this](auto&& PH1) { subscribeOD(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> m_imageCfSub{IMAGE_CF_TOPIC, this, [this](auto&& PH1) { subscribeImageCf(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::PerceptionGates>       m_gatesSub{GATES_TOPIC, this, [this](auto&& PH1) { subscribeGates(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::MechanicalInfo>        m_mechanicalInfoSub{MECHANICAL_INFO_TOPIC, this, [this](auto&& PH1) { subscribeMechanicalInfo(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::SensorTable>           m_sensorTableSub{SENSOR_TABLE_TOPIC, this, [this](auto&& PH1) { subscribeSensorTable(std::forward<decltype(PH1)>(PH1)); }};

#if FS_CHECK(CFG_USE_CF_FS)
    uto::Subscriber<uto::proto::CameraFreespace> m_cameraFsSub{FS_CF_TOPIC, this, [this](auto&& PH1) { subscribeCameraFs(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     m_calibCfSub{CALIB_CF_TOPIC, this, [this](auto&& PH1) { subscribeCalibCf(std::forward<decltype(PH1)>(PH1)); }};
#endif
#if FS_CHECK(CFG_USE_FISHEYE_FS)
    uto::Subscriber<uto::proto::CameraFreespace> m_fsFisheyeFcfSub{FS_FISHEYE_FCF_TOPIC, this, [this](auto&& PH1) { subscribeFsFisheyeFcf(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace> m_fsFisheyeFllSub{FS_FISHEYE_FLL_TOPIC, this, [this](auto&& PH1) { subscribeFsFisheyeFll(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace> m_fsFisheyeFrrSub{FS_FISHEYE_FRR_TOPIC, this, [this](auto&& PH1) { subscribeFsFisheyeFrr(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace> m_fsFisheyeBcbSub{FS_FISHEYE_BCB_TOPIC, this, [this](auto&& PH1) { subscribeFsFisheyeBcb(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace> m_fsFisheyeBllSub{FS_FISHEYE_BLL_TOPIC, this, [this](auto&& PH1) { subscribeFsFisheyeBll(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace> m_fsFisheyeBrrSub{FS_FISHEYE_BRR_TOPIC, this, [this](auto&& PH1) { subscribeFsFisheyeBrr(std::forward<decltype(PH1)>(PH1)); }};

    uto::Subscriber<uto::proto::CameraCalib> m_calibFcfSub{CALIB_FISHEYE_FCF_TOPIC, this, [this](auto&& PH1) { subscribeCalibFisheyeFcf(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib> m_calibFllSub{CALIB_FISHEYE_FLL_TOPIC, this, [this](auto&& PH1) { subscribeCalibFisheyeFll(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib> m_calibFrrSub{CALIB_FISHEYE_FRR_TOPIC, this, [this](auto&& PH1) { subscribeCalibFisheyeFrr(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib> m_calibBcbSub{CALIB_FISHEYE_BCB_TOPIC, this, [this](auto&& PH1) { subscribeCalibFisheyeBcb(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib> m_calibBllSub{CALIB_FISHEYE_BLL_TOPIC, this, [this](auto&& PH1) { subscribeCalibFisheyeBll(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib> m_calibBrrSub{CALIB_FISHEYE_BRR_TOPIC, this, [this](auto&& PH1) { subscribeCalibFisheyeBrr(std::forward<decltype(PH1)>(PH1)); }};

    uto::Subscriber<sensor_msgs::msg::CompressedImage> m_imageFcfSub{IMAGE_FISHEYE_FCF_TOPIC, this, [this](auto&& PH1) { subscribeImageFisheyeFcf(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> m_imageFllSub{IMAGE_FISHEYE_FLL_TOPIC, this, [this](auto&& PH1) { subscribeImageFisheyeFll(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> m_imageFrrSub{IMAGE_FISHEYE_FRR_TOPIC, this, [this](auto&& PH1) { subscribeImageFisheyeFrr(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> m_imageBcbSub{IMAGE_FISHEYE_BCB_TOPIC, this, [this](auto&& PH1) { subscribeImageFisheyeBcb(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> m_imageBllSub{IMAGE_FISHEYE_BLL_TOPIC, this, [this](auto&& PH1) { subscribeImageFisheyeBll(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> m_imageBrrSub{IMAGE_FISHEYE_BRR_TOPIC, this, [this](auto&& PH1) { subscribeImageFisheyeBrr(std::forward<decltype(PH1)>(PH1)); }};
#endif

#if FS_CHECK(CFG_VIS_DRAW_PERCEPTION_FREESPACE)
    uto::Subscriber<uto::proto::PerceptionFreespace> m_perceptionFreespaceSub{FUSION_FS_TOPIC, this, [this](auto&& PH1) { subscribePerceptionFreespace(std::forward<decltype(PH1)>(PH1)); }};
#endif
    // publishers
    uto::Publisher<uto::proto::ErrorCode>           m_diagPub{MONITOR_TOPIC, this};
    uto::Publisher<uto::proto::PerceptionFreespace> m_fusionFsPub{FUSION_FS_TOPIC, this};

#if FS_CHECK(CFG_VIS_ENABLE)
    std::unique_ptr<rosbag2_cpp::Writer> m_rosWriterPtr = std::make_unique<rosbag2_cpp::Writer>();
#endif
  };
} // namespace fs