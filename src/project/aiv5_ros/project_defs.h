#ifndef PROJECT_AIV5_ROS_DEFS_H_
#define PROJECT_AIV5_ROS_DEFS_H_
#include <cstdint>
namespace fs
{
  // topics
  static constexpr char FS_TOPIC[]           = "/perception/perception_freespace";
  static constexpr char GATES_TOPIC[]        = "/perception/perception_gates";
  static constexpr char SENSOR_TABLE_TOPIC[] = "/sensor_table/sensor_table";
  static constexpr char CALIB_FW_TOPIC[]     = "/calib/cam_fcf";
  static constexpr char CALIB_FN_TOPIC[]     = "/calib/cam_fcf_far";
  static constexpr char CALIB_FL_TOPIC[]     = "/calib/cam_flb";
  static constexpr char CALIB_FR_TOPIC[]     = "/calib/cam_frb";
  static constexpr char CALIB_RW_TOPIC[]     = "/calib/cam_bcb";
  static constexpr char CALIB_RN_TOPIC[]     = "/calib/cam_bcb_far";
  static constexpr char CALIB_RL_TOPIC[]     = "/calib/cam_blf";
  static constexpr char CALIB_RR_TOPIC[]     = "/calib/cam_brf";
  static constexpr char IMAGE_FW_TOPIC[]     = "/hal/sensor_raw/cam_fcf_jpeg";
  static constexpr char IMAGE_FN_TOPIC[]     = "/hal/sensor_raw/cam_fcf_far_jpeg";
  static constexpr char IMAGE_FL_TOPIC[]     = "/hal/sensor_raw/cam_flb_jpeg";
  static constexpr char IMAGE_FR_TOPIC[]     = "/hal/sensor_raw/cam_frb_jpeg";
  static constexpr char IMAGE_RW_TOPIC[]     = "/hal/sensor_raw/cam_bcb_jpeg";
  static constexpr char IMAGE_RN_TOPIC[]     = "/hal/sensor_raw/cam_bcb_far_jpeg";
  static constexpr char IMAGE_RL_TOPIC[]     = "/hal/sensor_raw/cam_blf_jpeg";
  static constexpr char IMAGE_RR_TOPIC[]     = "/hal/sensor_raw/cam_brf_jpeg";

  static constexpr char CALIB_SFW_TOPIC[] = "/calib/fisheye_fcf";
  static constexpr char CALIB_SFL_TOPIC[] = "/calib/fisheye_fll";
  static constexpr char CALIB_SFR_TOPIC[] = "/calib/fisheye_frr";
  static constexpr char CALIB_SRL_TOPIC[] = "/calib/fisheye_bll";
  static constexpr char CALIB_SRR_TOPIC[] = "/calib/fisheye_brr";
  static constexpr char CALIB_SRW_TOPIC[] = "/calib/fisheye_bcb";
  static constexpr char IMAGE_SFW_TOPIC[] = "/hal/sensor_raw/fisheye_fcf_jpeg";
  static constexpr char IMAGE_SFL_TOPIC[] = "/hal/sensor_raw/fisheye_fll_jpeg";
  static constexpr char IMAGE_SFR_TOPIC[] = "/hal/sensor_raw/fisheye_frr_jpeg";
  static constexpr char IMAGE_SRL_TOPIC[] = "/hal/sensor_raw/fisheye_bll_jpeg";
  static constexpr char IMAGE_SRR_TOPIC[] = "/hal/sensor_raw/fisheye_brr_jpeg";
  static constexpr char IMAGE_SRW_TOPIC[] = "/hal/sensor_raw/fisheye_bcb_jpeg";
  static constexpr char FS_SFW_TOPIC[]    = "/perception/fisheye_cam_fcf/camera_freespace";
  static constexpr char FS_SFL_TOPIC[]    = "/perception/fisheye_cam_fll/camera_freespace";
  static constexpr char FS_SFR_TOPIC[]    = "/perception/fisheye_cam_frr/camera_freespace";
  static constexpr char FS_SRL_TOPIC[]    = "/perception/fisheye_cam_bll/camera_freespace";
  static constexpr char FS_SRR_TOPIC[]    = "/perception/fisheye_cam_brr/camera_freespace";
  static constexpr char FS_SRW_TOPIC[]    = "/perception/fisheye_cam_bcb/camera_freespace";

  static constexpr char LIDAR_FS_TOPIC[]     = "/perception/lidar_freespace";
  static constexpr char FS_FW_TOPIC[]        = "/perception/camera_freespace"; // -- TODO: not realized yet
  static constexpr char FS_FN_TOPIC[]        = "/perception/camera_freespace"; // -- TODO: not realized yet
  static constexpr char VOT_OBJECTS_TOPIC[]  = "/perception/fused_objects";
  static constexpr char BEV_FS_TOPIC[]       = "/perception/camera_bev_freespace";
  static constexpr char VEHICLE_POSE_TOPIC[] = "/locator/vehicle_pose";
  static constexpr char MONITOR_TOPIC[]      = "/perception/fusion_fs_status"; // --  diagnosis monitor
  static constexpr char LANE_FC_TOPIC[]      = "/perception/fc_perception_lane";
  static constexpr char LANE_RC_TOPIC[]      = "/perception/rc_perception_lane";
  static constexpr char LANE_TOPIC[]         = "/perception/perception_lane";
  static constexpr char ROAD_MODEL_TOPIC[]   = "/perception/lidar_road_model";

  static constexpr char VEHICLE_CONFIG_TOPIC[]  = "/vehicle_config/vehicle_config";  ///< 旧的随车参数通道名称，但是目前项目都在用这个
  static constexpr char MECHANICAL_INFO_TOPIC[] = "/vehicle_config/mechanical_info"; ///< AIV2.0规定的新的随车机械参数通道名称，但是项目没用，可能在适配中，目前两个通道都接入

  static constexpr int     CAMERA_FRONT_CENTER_WIDTH  = 1920;
  static constexpr int     CAMERA_FRONT_CENTER_HEIGHT = 1208;
  static constexpr int     CAMERA_WIDTH               = 1920;
  static constexpr int     CAMERA_HEIGHT              = 1208;
  static constexpr int     BEV_WIDTH                  = 800 * 2;
  static constexpr int     BEV_HEIGHT                 = 1400 * 2;
  static constexpr int64_t TIME_SCALE_MS              = 1000000L;

  static constexpr float HALF_EGO_WIDTH_M = 1.4F; // -- m

  static constexpr int HALF_EGO_WIDTH  = 14; // -- grid
  static constexpr int HALF_EGO_LENGTH = 75; // -- grid

  static constexpr int REMOVE_REGION_FRONT = 20;
  static constexpr int REMOVE_REGION_BACK  = -20;
  static constexpr int REMOVE_REGION_LEFT  = 2;
  static constexpr int REMOVE_REGION_RIGHT = -2;

  static constexpr float RESOLUTION = 0.1f; // -- grid resolution

  static constexpr int FS_ROW_OUT     = 800;            // -- gridmap length
  static constexpr int FS_COL_OUT     = 400;            // -- gridmap width
  static constexpr int FS_CAR_ROW_OUT = FS_ROW_OUT / 2; // -- vehicle position x in gridmap
  static constexpr int FS_CAR_COL_OUT = FS_COL_OUT / 2; // -- vehicle position y in gridmap

  // -- fw remove noise
  static constexpr float FW_THRE = 1000.0f;

  // -- vot box expand
  static constexpr float STOPPED_THRE = 1e-9f;
  // -- tpts
  static constexpr float TPTS_LEFT  = -2.0f;
  static constexpr float TPTS_RIGHT = 2.0f;

  static const std::map<const SensorId, const int> SENSOR_ID_TO_CAMERA_ID = {
#if FS_CHECK(CFG_USE_CAM_FISH_FS)
    {SensorId::CAMERA_SFW, 21},
    {SensorId::CAMERA_SFL, 20},
    {SensorId::CAMERA_SFR, 22},
    {SensorId::CAMERA_SRL, 25},
    {SensorId::CAMERA_SRR, 23},
    {SensorId::CAMERA_SRW, 24},
#endif
    {SensorId::CAMERA_RL, 10},
    {SensorId::CAMERA_FL, 11},
    {SensorId::CAMERA_FW, 12},
    {SensorId::CAMERA_FN, 13},
    {SensorId::CAMERA_FR, 14},
    {SensorId::CAMERA_RR, 15},
    {SensorId::CAMERA_RW, 16},
    {SensorId::CAMERA_RN, 17}};

  static const std::map<const SensorId, const bool> SENSORS_USED = {
#if FS_CHECK(CFG_USE_CAM_FISH_FS)
    {SensorId::CAMERA_SFW, true},
    {SensorId::CAMERA_SFL, true},
    {SensorId::CAMERA_SFR, true},
    {SensorId::CAMERA_SRL, true},
    {SensorId::CAMERA_SRR, true},
    {SensorId::CAMERA_SRW, true},
#endif
    {SensorId::CAMERA_FW, true},
    {SensorId::CAMERA_FN, false},
    {SensorId::CAMERA_FL, false},
    {SensorId::CAMERA_FR, false},
    {SensorId::CAMERA_RW, false},
    {SensorId::CAMERA_RN, false},
    {SensorId::CAMERA_RL, false},
    {SensorId::CAMERA_RR, false}};

  static constexpr int ID_DEFAULT = 0;

  static constexpr float RADIUS = 2.0f;

} // namespace fs

#endif //PROJECT_AIV5_ROS_DEFS_H_
