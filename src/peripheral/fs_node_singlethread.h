#ifndef NODE_H_
#define NODE_H_

#include "diagnoser.h"
#include "rwlock.h"
#include "defs.h"
#include "types.h"
#include "macro.h"
#include "switcher.h"
#include "utils.h"
#include "peripheral/camera/camera_manager.h"
#include "vehicle.h"
#include "core/freespace_manager.h"
#include <common/log.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "ring_buffer.h"
#include <unistd.h>

#if FS_CHECK(CFG_VIS_ENABLE)
#include "debug/vis.h"
#include <cv_bridge/cv_bridge.h>
#endif

#if FS_CHECK(CFG_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include "rosbag2_cpp/writer.hpp"
#include <common/publisher.h>
#include <common/subscriber.h>
#include <camera_calib.pb.h>
#include <perception_camera.pb.h>
#include <geometry.pb.h>
#include <locator_vehicle_pose.pb.h>
#include <perception_freespace.pb.h>
#include <perception_signal.pb.h>
#include <error_code.pb.h>
#include <perception_obstacles.pb.h>
#include <project_config.pb.h>
#include <perception_lane.pb.h>
#include <mechanical_info.pb.h>
#if FS_CHECK(CFG_USE_ROAD_MODEL)
#include <perception_road_model.pb.h>
#endif
#if FS_CHECK(CFG_VIS_ENABLE)
#include <sensor_msgs/msg/image.hpp>
#endif
#else
#include <lcm/IMAGE.hpp>
#include <lcm/LOGITECH_IMAGE.hpp>
#include <lcm/TRANSFORM.hpp>
#include <lcm/LocFusion.hpp>
#include <lcm/PERCEPTION_LINES.hpp>
#include <lcm/CameraFreespace.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcm/Lidar_FreeSpace_v2.hpp>
#include <lcm/PERCEPTION_OBSTACLES.hpp>
#include <lcm/VCU_VEHILCE_INFO.hpp>
#include <lcm/NORAML_CAMERA_LANE_CONTROL.hpp>
#include <lcm/PerceptionFreeSpace.hpp>
#include <DMAIV/include_lcm13/DmAivBatch.hpp>
#include <lcm/IntegratedPosition.hpp>
#include <proto/monitor_errorCode.pb.h>
#include <lcm/CameraFreespaceSeg.hpp>
#endif

namespace fs
{
  class FsNode final : public rclcpp::Node
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(FsNode);

    FsNode():
      Node(NODE_NAME)
    {
      init();
    }

  private:
    void publishMonitorMsg();

    void                           init();
    void                           process();
    void                           resetBuffers();
    LocFlag                        getLocFlag();
    void                           publishVis();
    static EgoMotion               interpolateEgoMotion(int64_t timestamp, const EgoMotion& rhs_value, const EgoMotion& rhs_ego_motion);
    bool                           getAlignedEgoMotion(int64_t timestamp, std::unique_ptr<EgoMotion>& ego_motion_ptr, const fs::RingBuffer<EgoMotion, EMO_BUFFER_SIZE>& emo_buffer);
    bool                           getAlignedEgoMotionNoLock(int64_t timestamp, std::unique_ptr<EgoMotion>& ego_motion_ptr, const fs::RingBuffer<EgoMotion, EMO_BUFFER_SIZE>& emo_buffer);
    std::unique_ptr<fs::EgoMotion> getLatestEgoMotion(const fs::RingBuffer<EgoMotion, EMO_BUFFER_SIZE>& emo_buffer);
    void                           refineVotObject(const LidarFS* lidar_fs_ptr, ObjectStates* vot_object_ptr);
#if FS_CHECK(CFG_USE_ROAD_MODEL)
    void refineWithRoadModel(fs::LidarFS* lidar_fs_ptr, std::shared_ptr<RoadModelFS> road_model_proto_ptr);
#endif

    rclcpp::TimerBase::SharedPtr               timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(PROCESS_PERIOD_MS), [this] { process(); });
    RWLock                                     rw_lock_;
    fs::RingBuffer<EgoMotion, EMO_BUFFER_SIZE> emo_pos_buffer_;
    fs::RingBuffer<EgoMotion, EMO_BUFFER_SIZE> emo_align_pos_buffer_;

    int64_t                fusion_timestamp_ = 0;
    FreespaceManager       freespace_manager_;
    std::vector<GridLabel> vot_type_order_ = {GridLabel::OTHER, GridLabel::RTG_QC, GridLabel::CONE, GridLabel::CYCLIST, GridLabel::PEDESTRIAN};

    // -- start pos offset
    double init_x_    = 0.;
    double init_y_    = 0.;
    double init_z_    = 0.;
    bool   init_flag_ = false;

    // -- start align_pos offset
    double init_x_align_    = 0.;
    double init_y_align_    = 0.;
    double init_z_align_    = 0.;
    bool   init_flag_align_ = false;

    LocFlag  loc_flag_             = LocFlag::VALID;
    int64_t  loc_valid_first_time_ = 0;
    uint64_t lidar_count_;

#if FS_CHECK(CFG_VIS_ENABLE)
    // publishers
    std::unordered_map<fs::SensorId, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> map_pub_img_ = {
#if FS_CHECK(CFG_USE_SVC)
      {fs::SensorId::CAMERA_SFW, this->create_publisher<sensor_msgs::msg::Image>("/fs/SFW_canvas", 1000 / PROCESS_PERIOD_MS)},
      {fs::SensorId::CAMERA_SFL, this->create_publisher<sensor_msgs::msg::Image>("/fs/SFL_canvas", 1000 / PROCESS_PERIOD_MS)},
      {fs::SensorId::CAMERA_SFR, this->create_publisher<sensor_msgs::msg::Image>("/fs/SFR_canvas", 1000 / PROCESS_PERIOD_MS)},
      {fs::SensorId::CAMERA_SRL, this->create_publisher<sensor_msgs::msg::Image>("/fs/SRL_canvas", 1000 / PROCESS_PERIOD_MS)},
      {fs::SensorId::CAMERA_SRR, this->create_publisher<sensor_msgs::msg::Image>("/fs/SRR_canvas", 1000 / PROCESS_PERIOD_MS)},
      {fs::SensorId::CAMERA_SRW, this->create_publisher<sensor_msgs::msg::Image>("/fs/SRW_canvas", 1000 / PROCESS_PERIOD_MS)},
#endif // -- CFG_USE_SVC
      { fs::SensorId::CAMERA_FW,
        this->create_publisher<sensor_msgs::msg::Image>("/fs/FW_canvas", 1000 / PROCESS_PERIOD_MS) }
      //      {fs::SensorId::CAMERA_RW, this->create_publisher<sensor_msgs::msg::Image>("/fs/FN_canvas", 1000 / PROCESS_PERIOD_MS)},
      //      {fs::SensorId::CAMERA_FN, this->create_publisher<sensor_msgs::msg::Image>("/fs/FL_canvas", 1000 / PROCESS_PERIOD_MS)},
      //      {fs::SensorId::CAMERA_RN, this->create_publisher<sensor_msgs::msg::Image>("/fs/FR_canvas", 1000 / PROCESS_PERIOD_MS)},
      //      {fs::SensorId::CAMERA_FL, this->create_publisher<sensor_msgs::msg::Image>("/fs/RW_canvas", 1000 / PROCESS_PERIOD_MS)},
      //      {fs::SensorId::CAMERA_FR, this->create_publisher<sensor_msgs::msg::Image>("/fs/RN_canvas", 1000 / PROCESS_PERIOD_MS)},
      //      {fs::SensorId::CAMERA_RL, this->create_publisher<sensor_msgs::msg::Image>("/fs/RL_canvas", 1000 / PROCESS_PERIOD_MS)},
      //      { fs::SensorId::CAMERA_RR,this->create_publisher<sensor_msgs::msg::Image>("/fs/RR_canvas", 1000 / PROCESS_PERIOD_MS)

    };

#if FS_CHECK(CFG_VIS_ENABLE)
    std::unordered_map<fs::CySensorId, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> cy_map_pub_img_ = {
#if FS_CHECK(CFG_USE_SVC)
      {fs::CySensorId::CAMERA_CFW, this->create_publisher<sensor_msgs::msg::Image>("/fs/CFW_canvas", 1000 / PROCESS_PERIOD_MS)},
      {fs::CySensorId::CAMERA_CFL, this->create_publisher<sensor_msgs::msg::Image>("/fs/CFL_canvas", 1000 / PROCESS_PERIOD_MS)},
      {fs::CySensorId::CAMERA_CFR, this->create_publisher<sensor_msgs::msg::Image>("/fs/CFR_canvas", 1000 / PROCESS_PERIOD_MS)},
      {fs::CySensorId::CAMERA_CRL, this->create_publisher<sensor_msgs::msg::Image>("/fs/CRL_canvas", 1000 / PROCESS_PERIOD_MS)},
      {fs::CySensorId::CAMERA_CRR, this->create_publisher<sensor_msgs::msg::Image>("/fs/CRR_canvas", 1000 / PROCESS_PERIOD_MS)},
      {fs::CySensorId::CAMERA_CRW, this->create_publisher<sensor_msgs::msg::Image>("/fs/CRW_canvas", 1000 / PROCESS_PERIOD_MS)},
#endif // -- CFG_USE_SVC
    };
#endif

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr remap_img_pub_ptr_ = this->create_publisher<sensor_msgs::msg::Image>("/fs/REMAP_canvas", 1000 / PROCESS_PERIOD_MS);
    //zylinder example
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr zylinder_img_pub_ptr_ = this->create_publisher<sensor_msgs::msg::Image>("/fs/Zylinder_canvas", 1000 / PROCESS_PERIOD_MS);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              bev_img_pub_ptr_            = this->create_publisher<sensor_msgs::msg::Image>("/fs/BEV_canvas", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr         lidar_cld_pub_ptr_          = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/lidar_point_cloud", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr         road_cld_pub_ptr_           = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/road_point_cloud", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr         bev_cld_pub_ptr_            = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/bev_point_cloud", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr         blind_cld_pub_ptr_          = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/blindpoint_cloud", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr         polygon_cld_pub_ptr_        = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/polygon_cloud", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr         lidar_cld_history_pub_ptr_  = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/lidar_point_cloud_history", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_ptr_       = this->create_publisher<visualization_msgs::msg::MarkerArray>("/fs/marker_array_canvas", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr blind_marker_array_pub_ptr_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/fs/blind_marker_array_canvas", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr         tpts_bbox_cld_pub_ptr_      = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/tpts_bbox_point_cloud", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr         out_bbox_cld_pub_ptr_       = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/out_bbox_point_cloud", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr         color_table_cld_pub_ptr_    = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/color_table_point_cloud", 1000 / PROCESS_PERIOD_MS);

    // -- split
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_OTHER      = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_OTHER", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_PEDESTRIAN = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_PEDESTRIAN", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_CYCLIST    = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_CYCLIST", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_VEHICLE    = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_VEHICLE", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_RTG_QC     = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_RTG_QC", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_LOCK_BOX   = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_LOCK_BOX", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_CONE       = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_CONE", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_AIV        = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_AIV", 1000 / PROCESS_PERIOD_MS);

    std::map<const fs::GridLabel, rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr> map_pub_built_in_ = {{fs::GridLabel::OTHER, cld_pub_OTHER},
                                                                                                                   {fs::GridLabel::PEDESTRIAN, cld_pub_PEDESTRIAN},
                                                                                                                   {fs::GridLabel::VEHICLE, cld_pub_VEHICLE},
                                                                                                                   {fs::GridLabel::RTG_QC, cld_pub_RTG_QC},
                                                                                                                   {fs::GridLabel::LOCK_BOX, cld_pub_LOCK_BOX},
                                                                                                                   {fs::GridLabel::CONE, cld_pub_CONE},
                                                                                                                   { fs::GridLabel::AIV,
                                                                                                                     cld_pub_AIV }};

    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_self_receive_OTHER       = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_self_receive_OTHER", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_self_receive_PEDESTRIAN  = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_self_receive_PEDESTRIAN", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_self_receive_CYCLIST     = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_self_receive_CYCLIST", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_self_receive_VEHICLE     = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_self_receive_VEHICLE", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_self_receive_RTG_QC      = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_self_receive_RTG_QC", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_self_receive_LOCK_BOX    = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_self_receive_LOCK_BOX", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_self_receive_CONE        = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_self_receive_CONE", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_self_receive_AIV         = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_self_receive_AIV", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr cld_pub_self_receive_TREE_BRANCH = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/cld_self_receive_TREE_BRANCH", 1000 / PROCESS_PERIOD_MS);

    std::map<const fs::GridLabel, rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr> map_pub_self_receive_ = {{fs::GridLabel::OTHER, cld_pub_self_receive_OTHER},
                                                                                                                       {fs::GridLabel::PEDESTRIAN, cld_pub_self_receive_PEDESTRIAN},
                                                                                                                       {fs::GridLabel::CYCLIST, cld_pub_self_receive_CYCLIST},
                                                                                                                       {fs::GridLabel::VEHICLE, cld_pub_self_receive_VEHICLE},
                                                                                                                       {fs::GridLabel::RTG_QC, cld_pub_self_receive_RTG_QC},
                                                                                                                       {fs::GridLabel::LOCK_BOX, cld_pub_self_receive_LOCK_BOX},
                                                                                                                       {fs::GridLabel::CONE, cld_pub_self_receive_CONE},
                                                                                                                       {fs::GridLabel::TREE_BRANCH, cld_pub_self_receive_TREE_BRANCH},
                                                                                                                       { fs::GridLabel::AIV,
                                                                                                                         cld_pub_self_receive_AIV }};
//#if FS_CHECK(CFG_USE_FOXGLOV_VIS)
#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_cld_pub_ptr2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/lidar_point_cloud2", 1000 / PROCESS_PERIOD_MS);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_OTHER2      = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_OTHER2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_PEDESTRIAN2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_PEDESTRIAN2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_CYCLIST2    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_CYCLIST2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_VEHICLE2    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_VEHICLE2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_RTG_QC2     = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_RTG_QC2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_LOCK_BOX2   = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_LOCK_BOX2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_CONE2       = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_CONE2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_AIV2        = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_AIV2", 1000 / PROCESS_PERIOD_MS);

    std::map<const fs::GridLabel, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> map_pub_built_in2_ = {{fs::GridLabel::OTHER, cld_pub_OTHER2},
                                                                                                                     {fs::GridLabel::PEDESTRIAN, cld_pub_PEDESTRIAN2},
                                                                                                                     {fs::GridLabel::VEHICLE, cld_pub_VEHICLE2},
                                                                                                                     {fs::GridLabel::RTG_QC, cld_pub_RTG_QC2},
                                                                                                                     {fs::GridLabel::LOCK_BOX, cld_pub_LOCK_BOX2},
                                                                                                                     {fs::GridLabel::CONE, cld_pub_CONE2},
                                                                                                                     { fs::GridLabel::AIV,
                                                                                                                       cld_pub_AIV2 }};

    //--self receive
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_self_receive_OTHER2       = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_self_receive_OTHER2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_self_receive_PEDESTRIAN2  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_self_receive_PEDESTRIAN2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_self_receive_CYCLIST2     = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_self_receive_CYCLIST2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_self_receive_VEHICLE2     = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_self_receive_VEHICLE2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_self_receive_RTG_QC2      = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_self_receive_RTG_QC2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_self_receive_LOCK_BOX2    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_self_receive_LOCK_BOX2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_self_receive_CONE2        = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_self_receive_CONE2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_self_receive_AIV2         = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_self_receive_AIV2", 1000 / PROCESS_PERIOD_MS);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cld_pub_self_receive_TREE_BRANCH2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs/cld_self_receive_TREE_BRANCH2", 1000 / PROCESS_PERIOD_MS);

    std::map<const fs::GridLabel, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> map_pub_self_receive2_ = {{fs::GridLabel::OTHER, cld_pub_self_receive_OTHER2},
                                                                                                                         {fs::GridLabel::PEDESTRIAN, cld_pub_self_receive_PEDESTRIAN2},
                                                                                                                         {fs::GridLabel::CYCLIST, cld_pub_self_receive_CYCLIST2},
                                                                                                                         {fs::GridLabel::VEHICLE, cld_pub_self_receive_VEHICLE2},
                                                                                                                         {fs::GridLabel::RTG_QC, cld_pub_self_receive_RTG_QC2},
                                                                                                                         {fs::GridLabel::LOCK_BOX, cld_pub_self_receive_LOCK_BOX2},
                                                                                                                         {fs::GridLabel::CONE, cld_pub_self_receive_CONE2},
                                                                                                                         {fs::GridLabel::TREE_BRANCH, cld_pub_self_receive_TREE_BRANCH2},
                                                                                                                         { fs::GridLabel::AIV,
                                                                                                                           cld_pub_self_receive_AIV2 }};
#endif

#if !FS_CHECK(CFG_ROS2)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr ego_bbox_cld_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/fs/ego_bbox_point_cloud", 1000 / PROCESS_PERIOD_MS);
#endif
#endif
    std::string recording_path_;
    std::string path_in_;
    std::string path_out_;
    int         vehicle_number_;
    int         vehicle_frame_count = 0;

#if FS_CHECK(CFG_ROS2)
#if FS_CHECK(CFG_VIS_ENABLE)
    void getAlignedInputs(std::unique_ptr<uto::proto::PerceptionFreespace>& fs_lidar_proto_ptr,
                          //                          std::unique_ptr<uto::proto::CameraFreespace>&       fs_FW_proto_ptr,
                          std::array<CamFS, (uint8_t)SensorId::MAX_SENSOR_NUM>& array_fs,
                          std::unique_ptr<uto::proto::PerceptionGates>&         gates_proto_ptr,
                          //                          std::unique_ptr<sensor_msgs::msg::CompressedImage>& image_FW_ros_ptr,
                          std::array<std::unique_ptr<sensor_msgs::msg::CompressedImage>, (uint8_t)SensorId::MAX_SENSOR_NUM>& array_img,
                          std::unique_ptr<EgoMotion>&                                                                        ego_motion_pos_ptr,
#if FS_CHECK(CFG_USE_ROAD_MODEL)
                          std::unique_ptr<uto::proto::RoadModel>& road_model_proto_ptr,
#endif
                          std::unique_ptr<uto::proto::PerceptionObstacles>& vot_object_proto_ptr);
#else
    void getAlignedInputs(std::unique_ptr<uto::proto::PerceptionFreespace>&     fs_lidar_proto_ptr,
                          std::array<CamFS, (uint8_t)SensorId::MAX_SENSOR_NUM>& array_fs,
                          std::unique_ptr<uto::proto::PerceptionGates>&         gates_proto_ptr,
                          std::unique_ptr<EgoMotion>&                           emo_pos_ptr,
#if FS_CHECK(CFG_USE_ROAD_MODEL)
                          std::unique_ptr<uto::proto::RoadModel>&               road_model_proto_ptr,
#endif
                          std::unique_ptr<uto::proto::PerceptionObstacles>&     vot_object_proto_ptr);
#endif

    void removeNoise(const std::unique_ptr<std::vector<fs::CamFSPoint>> cam_fs,
                     LidarFS*                                           lidar_fs_ptr,
                     const SensorId&                                    sensor_id,
                     const FSMat4x4&                                    Tcl);

    void                                         removeGate(const uto::proto::PerceptionGates* gates_proto_ptr,
                                                            LidarFS*                           lidar_fs_ptr,
                                                            const EgoMotion*                   ego_Twb_ptr);
    cv::Point2f                                  unprojectFisheye(const cv::Point2f& point, const Camera* const camera, int cols, int rows);
    void                                         getRemoveNoiseFreespace3(LidarFS* lidar_fs_ptr, const std::vector<CamFSPoint>* camera_fs_ptr, const int& idx, const Camera* const camera, const FSMat4x4& Tcl);
    void                                         processVehiclePose(const uto::proto::VehiclePose& vehicle_pose_proto);
    std::unique_ptr<std::vector<fs::CamFSPoint>> parseCameraFreespace(const uto::proto::CameraFreespace* camera_fs_proto);
    LidarFSPtr                                   parseLidarFSFromProto(const uto::proto::PerceptionFreespace* lidar_fs_proto_ptr, const Vehicle& vehicle);
#if FS_CHECK(CFG_USE_ROAD_MODEL)
    RoadModelFSPtr parseRoadModelFromProto(const uto::proto::RoadModel* road_model_proto_ptr);
#endif
    ObjectStatesPtr                                  parseVotObjects(const uto::proto::PerceptionObstacles* vot_objects_proto_ptr);
    void                                             modifyErrorGates(const std::vector<fs::Rect>& gates_bbox, LidarFS* lidar_fs_ptr);
    void                                             updateGatePoints(const GatesInfoVec& gates_info_vec, std::vector<fs::Rect>& gates_bbox);
    std::unique_ptr<uto::proto::PerceptionObstacles> cloneVotObjects(std::unique_ptr<uto::proto::PerceptionObstacles>& vot_object_ptr);
    std::unique_ptr<uto::proto::PerceptionGates>     cloneGates(std::unique_ptr<uto::proto::PerceptionGates>& gates_ptr);
    bool                                             isLockStation(const fs::ObjectStatesPtr& vot_ptr);
    bool                                             isElectricFence();
    std::vector<std::vector<fs::LineCoeff>>          getAllLaneCoeff();
    std::vector<fs::LineCoeff>                       getLanesCoeff(const uto::proto::PerceptionLanes* perception_lines);
    std::unique_ptr<uto::proto::PerceptionLanes>     getLatestLanes(const fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionLanes>, MSG_BUFFER_SIZE>& buffer);

#if FS_CHECK(CFG_VIS_ENABLE)
    void syncImg(std::unique_ptr<sensor_msgs::msg::CompressedImage>&                                  img,
                 CamFS&                                                                               fs,
                 fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE>& img_buffer);
#endif // -- CFG_VIS_ENABLE
    void syncFsTF(CamFS&                                                                         fs,
                  fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>& fs_buffer,
                  const fs::RingBuffer<EgoMotion, EMO_BUFFER_SIZE>&                              emo_buffer,
                  int                                                                            left,
                  int                                                                            right);

    fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionFreespace>, MSG_BUFFER_SIZE> fs_lidar_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>     fs_fw_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE>     fs_fn_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionGates>, GATES_BUFFER_SIZE>   gates_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionObstacles>, MSG_BUFFER_SIZE> vot_object_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionLanes>, MSG_BUFFER_SIZE>     fc_lanes_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionLanes>, MSG_BUFFER_SIZE>     rc_lanes_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::PerceptionLanes>, MSG_BUFFER_SIZE>     lanes_buffer_;
#if FS_CHECK(CFG_USE_ROAD_MODEL)
    fs::RingBuffer<std::unique_ptr<uto::proto::RoadModel>, MSG_BUFFER_SIZE> road_model_buffer_;
#endif

#if FS_CHECK(CFG_USE_CAM_FISH_FS)
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE> fs_sfw_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE> fs_sfl_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE> fs_sfr_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE> fs_srl_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE> fs_srr_buffer_;
    fs::RingBuffer<std::unique_ptr<uto::proto::CameraFreespace>, MSG_BUFFER_SIZE> fs_srw_buffer_;
#endif //

#if FS_CHECK(CFG_VIS_ENABLE)
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_fw_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_fn_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_fl_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_fr_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_rw_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_rn_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_rl_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_rr_buffer_;
#if FS_CHECK(CFG_USE_SVC)
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_sfw_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_sfl_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_sfr_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_srl_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_srr_buffer_;
    fs::RingBuffer<std::unique_ptr<sensor_msgs::msg::CompressedImage>, MSG_BUFFER_SIZE> img_srw_buffer_;
#endif // -- CFG_USE_SVC
#endif // -- CFG_VIS_ENABLE
#if FS_CHECK(CFG_USE_MCAP_FUSION)
    std::shared_ptr<rosbag2_cpp::Writer> m_bagWriterPtr{std::make_shared<rosbag2_cpp::Writer>()};
#endif

#if FS_CHECK(CFG_LOAD_RECORDING)
  public:
    std::unique_ptr<rosbag2_cpp::Reader> m_bagReaderPtr{std::make_unique<rosbag2_cpp::Reader>()};
    void                                 loadBag();
    bool                                 empty_;
    u_long                               curr_db3_index_;
    std::vector<std::string>             db3_files_;
    std::vector<std::string>             db3_name_prefixes_;
#else
  private:
    void subscribeVehicleConfig(const std::unique_ptr<uto::proto::MechanicalInfo>& message);
    void subscribeVehiclePose(const std::unique_ptr<uto::proto::VehiclePose>& message);
    void subscribeVotObject(std::unique_ptr<uto::proto::PerceptionObstacles> message);
    void subscribeFsLidar(std::unique_ptr<uto::proto::PerceptionFreespace> message);

#if FS_CHECK(CFG_USE_MCAP_RAW)
    void createMcap();
    void subscribeFsLidarFromBag(std::unique_ptr<uto::proto::PerceptionFreespace> message);
#endif

    void subscribeFsFW(std::unique_ptr<uto::proto::CameraFreespace> message);
    void subscribeFsFN(std::unique_ptr<uto::proto::CameraFreespace> message);

    void subscribeSensorTable(const std::unique_ptr<uto::proto::SensorTable>& message);
    void subscribeCalibFW(const std::unique_ptr<uto::proto::CameraCalib>& message);
    void subscribeCalibFN(const std::unique_ptr<uto::proto::CameraCalib>& message);
    void subscribeCalibFL(const std::unique_ptr<uto::proto::CameraCalib>& message);
    void subscribeCalibFR(const std::unique_ptr<uto::proto::CameraCalib>& message);
    void subscribeCalibRW(const std::unique_ptr<uto::proto::CameraCalib>& message);
    void subscribeCalibRN(const std::unique_ptr<uto::proto::CameraCalib>& message);
    void subscribeCalibRL(const std::unique_ptr<uto::proto::CameraCalib>& message);
    void subscribeCalibRR(const std::unique_ptr<uto::proto::CameraCalib>& message);
    // #if FS_CHECK(CFG_USE_CAM_FISH_FS)
    void subscribeCalibSFW(std::unique_ptr<uto::proto::CameraCalib> message);
    void subscribeCalibSFL(std::unique_ptr<uto::proto::CameraCalib> message);
    void subscribeCalibSFR(std::unique_ptr<uto::proto::CameraCalib> message);
    void subscribeCalibSRL(std::unique_ptr<uto::proto::CameraCalib> message);
    void subscribeCalibSRR(std::unique_ptr<uto::proto::CameraCalib> message);
    void subscribeCalibSRW(std::unique_ptr<uto::proto::CameraCalib> message);
    // #endif

#if FS_CHECK(CFG_USE_CAM_FISH_FS)
    void subscribeFsSFW(std::unique_ptr<uto::proto::CameraFreespace> message);
    void subscribeFsSFL(std::unique_ptr<uto::proto::CameraFreespace> message);
    void subscribeFsSFR(std::unique_ptr<uto::proto::CameraFreespace> message);
    void subscribeFsSRL(std::unique_ptr<uto::proto::CameraFreespace> message);
    void subscribeFsSRR(std::unique_ptr<uto::proto::CameraFreespace> message);
    void subscribeFsSRW(std::unique_ptr<uto::proto::CameraFreespace> message);
#endif // -- CFG_USE_CAM_FISH_FS
    void subscribeGates(std::unique_ptr<uto::proto::PerceptionGates> message);
    void subscribeFCLanes(std::unique_ptr<uto::proto::PerceptionLanes> message);
    void subscribeRCLanes(std::unique_ptr<uto::proto::PerceptionLanes> message);
    void subscribeLanes(std::unique_ptr<uto::proto::PerceptionLanes> message);
#if FS_CHECK(CFG_USE_ROAD_MODEL)
    void subscribeRoadModel(std::unique_ptr<uto::proto::RoadModel> message);
#endif

    uto::Subscriber<uto::proto::SensorTable> sensor_table_sub_ptr_{SENSOR_TABLE_TOPIC, this, [this](auto&& PH1) { subscribeSensorTable(std::forward<decltype(PH1)>(PH1)); }};

    uto::Subscriber<uto::proto::CameraCalib>     calib_fw_sub_ptr_{CALIB_FW_TOPIC, this, [this](auto&& PH1) { subscribeCalibFW(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_fn_sub_ptr_{CALIB_FN_TOPIC, this, [this](auto&& PH1) { subscribeCalibFN(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_fl_sub_ptr_{CALIB_FL_TOPIC, this, [this](auto&& PH1) { subscribeCalibFL(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_fr_sub_ptr_{CALIB_FR_TOPIC, this, [this](auto&& PH1) { subscribeCalibFR(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_rw_sub_ptr_{CALIB_RW_TOPIC, this, [this](auto&& PH1) { subscribeCalibRW(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_rn_sub_ptr_{CALIB_RN_TOPIC, this, [this](auto&& PH1) { subscribeCalibRN(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_rl_sub_ptr_{CALIB_RL_TOPIC, this, [this](auto&& PH1) { subscribeCalibRL(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_rr_sub_ptr_{CALIB_RR_TOPIC, this, [this](auto&& PH1) { subscribeCalibRR(std::forward<decltype(PH1)>(PH1)); }};

#if FS_CHECK(CFG_USE_CAM_FISH_FS)
    uto::Subscriber<uto::proto::CameraCalib>     calib_sfw_sub_ptr_{CALIB_SFW_TOPIC, this, [this](auto&& PH1) { subscribeCalibSFW(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_sfl_sub_ptr_{CALIB_SFL_TOPIC, this, [this](auto&& PH1) { subscribeCalibSFL(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_sfr_sub_ptr_{CALIB_SFR_TOPIC, this, [this](auto&& PH1) { subscribeCalibSFR(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_srl_sub_ptr_{CALIB_SRL_TOPIC, this, [this](auto&& PH1) { subscribeCalibSRL(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_srr_sub_ptr_{CALIB_SRR_TOPIC, this, [this](auto&& PH1) { subscribeCalibSRR(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraCalib>     calib_srw_sub_ptr_{CALIB_SRW_TOPIC, this, [this](auto&& PH1) { subscribeCalibSRW(std::forward<decltype(PH1)>(PH1)); }};
#endif

#if FS_CHECK(CFG_USE_CAM_FISH_FS)
    uto::Subscriber<uto::proto::CameraFreespace> fs_sfw_sub_ptr_{FS_SFW_TOPIC, this, [this](auto&& PH1) { subscribeFsSFW(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace> fs_sfl_sub_ptr_{FS_SFL_TOPIC, this, [this](auto&& PH1) { subscribeFsSFL(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace> fs_sfr_sub_ptr_{FS_SFR_TOPIC, this, [this](auto&& PH1) { subscribeFsSFR(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace> fs_srl_sub_ptr_{FS_SRL_TOPIC, this, [this](auto&& PH1) { subscribeFsSRL(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace> fs_srr_sub_ptr_{FS_SRR_TOPIC, this, [this](auto&& PH1) { subscribeFsSRR(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace> fs_srw_sub_ptr_{FS_SRW_TOPIC, this, [this](auto&& PH1) { subscribeFsSRW(std::forward<decltype(PH1)>(PH1)); }};
#endif

    uto::Subscriber<uto::proto::VehiclePose>         pose_sub_ptr_{VEHICLE_POSE_TOPIC, this, [this](auto&& PH1) { subscribeVehiclePose(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::PerceptionFreespace> fs_lidar_sub_ptr_{LIDAR_FS_TOPIC, this, [this](auto&& PH1) { subscribeFsLidar(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::MechanicalInfo>      vehicle_config_sub_ptr_{VEHICLE_CONFIG_TOPIC, this, [this](auto&& PH1) { subscribeVehicleConfig(std::forward<decltype(PH1)>(PH1)); }};
//    uto::Subscriber<uto::proto::MechanicalInfo> vehicle_config_sub_ptr_{MECHANICAL_INFO_TOPIC, this, [this](auto&& PH1) { subscribeFsLidar(std::forward<decltype(PH1)>(PH1)); }};
#if FS_CHECK(CFG_USE_CAM_FW_FS)
    uto::Subscriber<uto::proto::CameraFreespace>     fs_fw_sub_ptr_{FS_FW_TOPIC, this, [this](auto&& PH1) { subscribeFsFW(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::CameraFreespace>     fs_fn_sub_ptr_{FS_FN_TOPIC, this, [this](auto&& PH1) { subscribeFsFN(std::forward<decltype(PH1)>(PH1)); }};
#endif
    uto::Subscriber<uto::proto::PerceptionObstacles> vot_sub_ptr_{VOT_OBJECTS_TOPIC, this, [this](auto&& PH1) { subscribeVotObject(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::PerceptionGates>     gates_sub_ptr_{GATES_TOPIC, this, [this](auto&& PH1) { subscribeGates(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::PerceptionLanes>     lanes_fc_sub_ptr_{LANE_FC_TOPIC, this, [this](auto&& PH1) { subscribeFCLanes(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::PerceptionLanes>     lanes_rc_sub_ptr_{LANE_RC_TOPIC, this, [this](auto&& PH1) { subscribeRCLanes(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<uto::proto::PerceptionLanes>     lanes_sub_ptr_{LANE_TOPIC, this, [this](auto&& PH1) { subscribeLanes(std::forward<decltype(PH1)>(PH1)); }};
#if FS_CHECK(CFG_USE_ROAD_MODEL)
    uto::Subscriber<uto::proto::RoadModel>           road_model_sub_ptr_{ROAD_MODEL_TOPIC, this, [this](auto&& PH1) { subscribeRoadModel(std::forward<decltype(PH1)>(PH1)); }};
#endif
#if FS_CHECK(CFG_DEBUG_PERCEPTION_FREESPACES)
    uto::Subscriber<uto::proto::PerceptionFreespace> bag_fs_lidar_sub_ptr_{FS_TOPIC, this, [this](auto&& PH1) { subscribeFsLidarFromBag(std::forward<decltype(PH1)>(PH1)); }};
#endif
#if FS_CHECK(CFG_VIS_ENABLE)

    void subscribeImageFW(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageFN(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageFL(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageFR(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageRW(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageRN(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageRL(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageRR(sensor_msgs::msg::CompressedImage::UniquePtr message);
#if FS_CHECK(CFG_USE_SVC)
    void subscribeImageSFW(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageSFL(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageSFR(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageSRL(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageSRR(sensor_msgs::msg::CompressedImage::UniquePtr message);
    void subscribeImageSRW(sensor_msgs::msg::CompressedImage::UniquePtr message);
#endif // -- CFG_USE_SVC

    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_fw_sub_ptr_{IMAGE_FW_TOPIC, this, [this](auto&& PH1) { subscribeImageFW(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_fn_sub_ptr_{IMAGE_FN_TOPIC, this, [this](auto&& PH1) { subscribeImageFN(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_fl_sub_ptr_{IMAGE_FL_TOPIC, this, [this](auto&& PH1) { subscribeImageFL(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_fr_sub_ptr_{IMAGE_FR_TOPIC, this, [this](auto&& PH1) { subscribeImageFR(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_rw_sub_ptr_{IMAGE_RW_TOPIC, this, [this](auto&& PH1) { subscribeImageRW(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_rn_sub_ptr_{IMAGE_RN_TOPIC, this, [this](auto&& PH1) { subscribeImageRN(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_rl_sub_ptr_{IMAGE_RL_TOPIC, this, [this](auto&& PH1) { subscribeImageRL(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_rr_sub_ptr_{IMAGE_RR_TOPIC, this, [this](auto&& PH1) { subscribeImageRR(std::forward<decltype(PH1)>(PH1)); }};

#if FS_CHECK(CFG_USE_SVC)
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_sfw_sub_ptr_{IMAGE_SFW_TOPIC, this, [this](auto&& PH1) { subscribeImageSFW(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_sfl_sub_ptr_{IMAGE_SFL_TOPIC, this, [this](auto&& PH1) { subscribeImageSFL(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_sfr_sub_ptr_{IMAGE_SFR_TOPIC, this, [this](auto&& PH1) { subscribeImageSFR(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_srl_sub_ptr_{IMAGE_SRL_TOPIC, this, [this](auto&& PH1) { subscribeImageSRL(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_srr_sub_ptr_{IMAGE_SRR_TOPIC, this, [this](auto&& PH1) { subscribeImageSRR(std::forward<decltype(PH1)>(PH1)); }};
    uto::Subscriber<sensor_msgs::msg::CompressedImage> img_srw_sub_ptr_{IMAGE_SRW_TOPIC, this, [this](auto&& PH1) { subscribeImageSRW(std::forward<decltype(PH1)>(PH1)); }};
#endif // -- CFG_USE_SVC
#endif // -- CFG_VIS_ENABLE

#endif // -- CFG_LOAD_RECORDING
    uto::Publisher<uto::proto::ErrorCode> fs_monitor_pub_{MONITOR_TOPIC, this};
#if FS_CHECK(!CFG_DEBUG_PERCEPTION_FREESPACES)
    uto::Publisher<uto::proto::PerceptionFreespace> fs_enu_pub_ptr_{FS_TOPIC, this};
#endif
#else
#if FS_CHECK(CFG_LOAD_RECORDING)
    void loadLcm();
#else
  public:
    // clang-format off
    void lcmReceiverThread_DEFAULT()             {while(0 == m_lcmHandler_DEFAULT.handle()) {}            }
    void lcmReceiverThread_TRANSFORM()           {while(0 == m_lcmHandler_TRANSFORM.handle()) {}          }
    void lcmReceiverThread_LIDAR_GRIDMAP()       {while(0 == m_lcmHandler_LIDAR_GRIDMAP.handle()) {}      }
    void lcmReceiverThread_VCU_VEHICLE_INFO()    {while(0 == m_lcmHandler_VCU_VEHICLE_INFO.handle()) {}   }
    void lcmReceiverThread_NORMAL_CAMERA_FS()    {while(0 == m_lcmHandler_NORMAL_CAMERA_FS.handle()) {}   }
    void lcmReceiverThread_LANE_CONTROL()        {while(0 == m_lcmHandler_LINE_CONTROL.handle()) {}       }
    void lcmReceiverThread_BEV_FS()              {while(0 == m_lcmHandler_BEV_FS.handle()) {}             }
    void lcmReceiverThread_LANE()                {while(0 == m_lcmHandler_LANE.handle()) {}               }
    void lcmReceiverThread_Image_FW()            {while(0 == m_lcmHandler_Image_FW.handle()) {}           }
    void lcmReceiverThread_Image_FN()            {while(0 == m_lcmHandler_Image_FN.handle()) {}           }
    void lcmReceiverThread_Image_FL()            {while(0 == m_lcmHandler_Image_FL.handle()) {}           }
    void lcmReceiverThread_Image_FR()            {while(0 == m_lcmHandler_Image_FR.handle()) {}           }
    void lcmReceiverThread_Image_RW()            {while(0 == m_lcmHandler_Image_RW.handle()) {}           }
    void lcmReceiverThread_Image_RN()            {while(0 == m_lcmHandler_Image_RN.handle()) {}           }
    void lcmReceiverThread_Image_RL()            {while(0 == m_lcmHandler_Image_RL.handle()) {}           }
    void lcmReceiverThread_Image_RR()            {while(0 == m_lcmHandler_Image_RR.handle()) {}           }
    void lcmReceiverThread_VOT()                 {while(0 == m_lcmHandler_VOT.handle()) {}                }
    void lcmReceiverThread_Planner_Gridmap_AIV() {while(0 == m_lcmHandler_Planner_Gridmap_AIV.handle()) {}}
    void lcmReceiverCallback(const lcm::ReceiveBuffer* recv_buf, const std::string& channel_name);

  private:
    lcm::LCM  m_lcmHandler_DEFAULT            {"udpm://239.255.76.67:" + std::to_string(LCM_PORT_DEFAULT                               ) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_NORMAL_CAMERA_FS   {"udpm://239.255.76.67:" + std::to_string(LCM_PORT_NORMAL_CAMERA_FS                      ) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_LINE_CONTROL       {"udpm://239.255.76.67:" + std::to_string(LCM_PORT_LINE_CONTROL                          ) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_VOT                {"udpm://239.255.76.67:" + std::to_string(LCM_PORT_VOT                                   ) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_TRANSFORM          {"udpm://239.255.76.67:" + std::to_string(LCM_PORT_TRANSFORM                             ) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_VCU_VEHICLE_INFO   {"udpm://239.255.76.67:" + std::to_string(LCM_PORT_VCU_VEHICLE_INFO                      ) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_LIDAR_GRIDMAP      {"udpm://239.255.76.67:" + std::to_string(LCM_PORT_LIDAR_GRIDMAP                         ) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_BEV_FS             {"udpm://239.255.76.67:" + std::to_string(LCM_PORT_CAM_FS_BEV                            ) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_LANE               {"udpm://239.255.76.67:" + std::to_string(LCM_PORT_LANE                                  ) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_Image_FW           {"udpm://239.255.76.67:" + std::to_string(CAMERA_PORT_TO_LCM_NAME.at(SensorId::CAMERA_FW)) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_Image_FN           {"udpm://239.255.76.67:" + std::to_string(CAMERA_PORT_TO_LCM_NAME.at(SensorId::CAMERA_FN)) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_Image_FL           {"udpm://239.255.76.67:" + std::to_string(CAMERA_PORT_TO_LCM_NAME.at(SensorId::CAMERA_FL)) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_Image_FR           {"udpm://239.255.76.67:" + std::to_string(CAMERA_PORT_TO_LCM_NAME.at(SensorId::CAMERA_FR)) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_Image_RW           {"udpm://239.255.76.67:" + std::to_string(CAMERA_PORT_TO_LCM_NAME.at(SensorId::CAMERA_RW)) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_Image_RN           {"udpm://239.255.76.67:" + std::to_string(CAMERA_PORT_TO_LCM_NAME.at(SensorId::CAMERA_RN)) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_Image_RL           {"udpm://239.255.76.67:" + std::to_string(CAMERA_PORT_TO_LCM_NAME.at(SensorId::CAMERA_RL)) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_Image_RR           {"udpm://239.255.76.67:" + std::to_string(CAMERA_PORT_TO_LCM_NAME.at(SensorId::CAMERA_RR)) + "?ttl=1"};
#endif
    lcm::LCM  m_lcmHandler_Planner_Gridmap_AIV{"udpm://239.255.76.67:" + std::to_string(LCM_PORT_Planner_Gridmap_AIV                   ) + "?ttl=1"};
    lcm::LCM  m_lcmHandler_Error_Code         {"udpm://239.255.76.67:" + std::to_string(LCM_PORT_Error_Code                            ) + "?ttl=1"};
    // clang-format on

    int64_t last_sync_bev_lidar_timestamp_ = 0;
    int64_t last_sync_vot_lidar_timestamp_ = 0;

    std::vector<cv::Point2f> lidar_blind_l_poly_;
    std::vector<cv::Point2f> lidar_blind_r_poly_;
    bool                     lidar_blind_area_[GRID_MAP_SIZE][GRID_MAP_SIZE];

    fs::RingBuffer<std::unique_ptr<Lidar_FreeSpace_v2>, MSG_BUFFER_SIZE>             fs_lidar_buffer_;
    fs::RingBuffer<std::unique_ptr<FS_BEV_SEG::CameraFreespaceSeg>, MSG_BUFFER_SIZE> fs_bev_buffer_;
    fs::RingBuffer<std::unique_ptr<FS_BEV::CameraFreespace>, MSG_BUFFER_SIZE>        fs_fw_buffer_;
    fs::RingBuffer<std::unique_ptr<FS_BEV::CameraFreespace>, MSG_BUFFER_SIZE>        fs_rw_buffer_;
    fs::RingBuffer<std::unique_ptr<PERCEPTION_LINES>, MSG_BUFFER_SIZE>               lanes_buffer_;
    fs::RingBuffer<std::unique_ptr<VCU_VEHILCE_INFO>, MSG_BUFFER_SIZE>               vcu_buffer_;
    fs::RingBuffer<std::unique_ptr<NORAML_CAMERA_LANE_CONTROL>, MSG_BUFFER_SIZE>     line_control_buffer_;
#if FS_CHECK(CFG_VIS_ENABLE)
    fs::RingBuffer<std::unique_ptr<DMAIV::DmAivBatch>, MSG_BUFFER_SIZE>              image_bev_buffer_;
    fs::RingBuffer<std::unique_ptr<LOGITECH_IMAGE>, MSG_BUFFER_SIZE>                 img_fw_buffer_;
    fs::RingBuffer<std::unique_ptr<LOGITECH_IMAGE>, MSG_BUFFER_SIZE>                 img_fn_buffer_;
    fs::RingBuffer<std::unique_ptr<LOGITECH_IMAGE>, MSG_BUFFER_SIZE>                 img_rw_buffer_;
    fs::RingBuffer<std::unique_ptr<LOGITECH_IMAGE>, MSG_BUFFER_SIZE>                 img_rn_buffer_;
    fs::RingBuffer<std::unique_ptr<LOGITECH_IMAGE>, MSG_BUFFER_SIZE>                 img_rl_buffer_;
    fs::RingBuffer<std::unique_ptr<LOGITECH_IMAGE>, MSG_BUFFER_SIZE>                 img_rr_buffer_;
    fs::RingBuffer<std::unique_ptr<LOGITECH_IMAGE>, MSG_BUFFER_SIZE>                 img_fl_buffer_;
    fs::RingBuffer<std::unique_ptr<LOGITECH_IMAGE>, MSG_BUFFER_SIZE>                 img_fr_buffer_;
#endif
    fs::RingBuffer<std::unique_ptr<PERCEPTION_OBSTACLES>, MSG_BUFFER_SIZE>           vot_objects_buffer_;

    void processVcuInfo(const void* data, uint32_t size);
    void processLaneControlInfo(const void* data, uint32_t size);
    void processVotObscales(const void* data, uint32_t size);
    template<class T>
    void                                            processImage(fs::RingBuffer<std::unique_ptr<T>, MSG_BUFFER_SIZE>& image_buffer, const void* data, uint32_t size);
    void                                            processVehiclePose(const void* data, uint32_t size);
    void                                            processLocalization(const void* data, const uint32_t size);
    void                                            processLidarFreeSpace(const void* data, uint32_t size);
    void                                            processBEVFreeSpace(const void* data, uint32_t size);
    void                                            processFWFreeSpace(const void* data, uint32_t size);
    void                                            processRWFreeSpace(const void* data, uint32_t size);
    void                                            processLocMonitor(const void* data, uint32_t size);
    void                                            processLane(const void* data, uint32_t size);
    LidarFSPtr                                      parseLidarFreespace(const Lidar_FreeSpace_v2* lidar_fs_lcm_ptr);
    bool                                            getAlignedVcu(int64_t timestamp, ObjectState& ego_state);
    std::unique_ptr<FS_BEV_SEG::CameraFreespaceSeg> cloneBevFs(std::unique_ptr<FS_BEV_SEG::CameraFreespaceSeg>& bev_fs_ptr);
    bool                                            getReverse();
    static ObjectState                              interpolateVcu(const int64_t timestamp, const VCU_VEHILCE_INFO* lhs_vcu, const VCU_VEHILCE_INFO* rhs_vcu);
    void                                            subscribeFsLidarFromBag(const void* data, uint32_t size);
    std::vector<cv::Point2f>                        getLanePolygon(LidarFS* lidar_fs_ptr, const PERCEPTION_LINES* perception_lines);
    bool                                            getAlignedInputs(std::unique_ptr<Lidar_FreeSpace_v2>&             fs_lidar_lcm_ptr,
                                                                     std::unique_ptr<FS_BEV_SEG::CameraFreespaceSeg>& fs_bev_lcm_ptr,
                                                                     std::unique_ptr<DMAIV::DmAivBatch>&              image_bev_lcm_ptr,
                                                                     std::unique_ptr<EgoMotion>&                      ego_motion_ptr,
                                                                     std::unique_ptr<PERCEPTION_OBSTACLES>&           vot_objects_lcm_ptr);
    ObjectStatesPtr                                 parseVotObjects(const PERCEPTION_OBSTACLES* vot_objects_lcm_ptr);
    bool                                            isLockStation(const fs::ObjectStatesPtr& vot_ptr);
    fs::BevSegPtr                                   parseBevSeg(const FS_BEV_SEG::CameraFreespaceSeg* bev_seg_lcm_ptr);
    std::unique_ptr<PERCEPTION_OBSTACLES>           cloneVotObjects(std::unique_ptr<PERCEPTION_OBSTACLES>& vot_object_ptr);

    /**
     * @brief  tf bev-fs to lidar coord ;
     *         update bev-fs position and  timestamp = lidar.timestamp
     * @param timestamp
     * @param bev_fs_lcm_ptr
     */
    void refineBevFs(int64_t timestamp, FS_BEV::CameraFreespace* bev_fs_lcm_ptr);

    void refineLine(int64_t timestamp, PERCEPTION_LINES* lane_lcm_ptr);

    void refineBevFsSeg(int64_t timestamp, BevSeg* bev_fs_lcm_ptr);

    void addBevFreespace(LidarFS* lidar_fs_ptr, const FS_BEV::CameraFreespace* bev_fs_ptr);

    void initBlindArea();
#endif // -- CFG_ROS2
  };
} // namespace fs

#endif //NODE_H_