#ifndef PROJECT_HDT_DEFS_H_
#define PROJECT_HDT_DEFS_H_
#include <cstdint>
namespace fs
{
  // topics
  static constexpr char FS_TOPIC[]           = "/perception/perception_freespace";
  static constexpr char GATES_TOPIC[]        = "/perception/perception_gates";
  static constexpr char SENSOR_TABLE_TOPIC[] = "/sensor_table/sensor_table";
  static constexpr char CALIB_FW_TOPIC[]     = "/calib/cam_fcf";
  static constexpr char CALIB_FN_TOPIC[]     = "/calib/cam_fcf_far";
  static constexpr char CALIB_FL_TOPIC[]     = "/calib/cam_flf";
  static constexpr char CALIB_FR_TOPIC[]     = "/calib/cam_frf";
  static constexpr char CALIB_RW_TOPIC[]     = "/calib/cam_bcb";
  static constexpr char CALIB_RN_TOPIC[]     = "/calib/cam_bcb_far";
  static constexpr char CALIB_RL_TOPIC[]     = "/calib/cam_flb";
  static constexpr char CALIB_RR_TOPIC[]     = "/calib/cam_frb";
  static constexpr char IMAGE_FW_TOPIC[]     = "/hal/sensor/cam_fcf_jpeg";
  static constexpr char IMAGE_FN_TOPIC[]     = "/hal/sensor/cam_fcf_far_jpeg";
  static constexpr char IMAGE_FL_TOPIC[]     = "/hal/sensor/cam_flf_jpeg";
  static constexpr char IMAGE_FR_TOPIC[]     = "/hal/sensor/cam_frf_jpeg";
  static constexpr char IMAGE_RW_TOPIC[]     = "/hal/sensor/cam_bcb_jpeg";
  static constexpr char IMAGE_RN_TOPIC[]     = "/hal/sensor/cam_bcb_far_jpeg";
  static constexpr char IMAGE_RL_TOPIC[]     = "/hal/sensor/cam_flb_jpeg";
  static constexpr char IMAGE_RR_TOPIC[]     = "/hal/sensor/cam_frb_jpeg";
  static constexpr char LIDAR_FS_TOPIC[]     = "/perception/lidar_freespace";
  static constexpr char FS_FW_TOPIC[]        = "/perception/camera_freespace";
  static constexpr char FS_FN_TOPIC[]        = "/perception/camera_freespace"; // -- TODO: not realized yet
  static constexpr char VOT_OBJECTS_TOPIC[]  = "/perception/fused_objects";
  static constexpr char VEHICLE_POSE_TOPIC[] = "/locator/vehicle_pose";
  static constexpr char MONITOR_TOPIC[]      = "/perception/fusion_fs_status"; // -- diagnosis monitor
  static constexpr char LANE_FC_TOPIC[]      = "/perception/fc_perception_lane";
  static constexpr char LANE_RC_TOPIC[]      = "/perception/rc_perception_lane";
  static constexpr char LANE_TOPIC[]         = "/perception/perception_lane";
  static constexpr char ROAD_MODEL_TOPIC[]   = "/perception/lidar_road_model";

  static constexpr char VEHICLE_CONFIG_TOPIC[]  = "/vehicle_config/vehicle_config";  ///< 旧的随车参数通道名称，但是目前项目都在用这个
  static constexpr char MECHANICAL_INFO_TOPIC[] = "/vehicle_config/mechanical_info"; ///< AIV2.0规定的新的随车机械参数通道名称，但是项目没用，可能在适配中，目前两个通道都接入

//  static constexpr char VEHICLE_CONFIG_TOPIC[]  = "/vehicle_config/vehicle_config";  ///< 旧的随车参数通道名称，但是目前项目都在用这个
//  static constexpr char MECHANICAL_INFO_TOPIC[] = "/vehicle_config/mechanical_info"; ///< AIV2.0规定的新的随车机械参数通道名称，但是项目没用，可能在适配中，目前两个通道都接入

  static constexpr int     CAMERA_FRONT_CENTER_WIDTH  = 3840;
  static constexpr int     CAMERA_FRONT_CENTER_HEIGHT = 2160;
  static constexpr int     CAMERA_WIDTH               = 2880;
  static constexpr int     CAMERA_HEIGHT              = 1860;
  static constexpr int     BEV_WIDTH                  = 800 * 2;
  static constexpr int     BEV_HEIGHT                 = 1400 * 2;
  static constexpr int64_t TIME_SCALE_MS              = 1000000L;

  static constexpr float HALF_EGO_WIDTH_M = 1.3F; // -- m

  static constexpr int HALF_EGO_WIDTH  = 14; // -- grid
  static constexpr int HALF_EGO_LENGTH = 75; // -- grid

  static constexpr int REMOVE_REGION_FRONT = 20;
  static constexpr int REMOVE_REGION_BACK  = -20;
  static constexpr int REMOVE_REGION_LEFT  = 2;
  static constexpr int REMOVE_REGION_RIGHT = -2;

  static constexpr float RESOLUTION = 0.1f; // -- grid resolution

  static constexpr int FS_ROW_OUT     = 800;            // -- gridmap length
  static constexpr int FS_COL_OUT     = 400;            // -- gridmap width
  static constexpr int FS_CAR_ROW_OUT = 200;            // -- vehicle position x in gridmap
  static constexpr int FS_CAR_COL_OUT = FS_COL_OUT / 2; // -- vehicle position y in gridmap

  // -- fw remove noise
  static constexpr float FW_THRE = 1000.0f;

  // -- vot box expand
  static constexpr float STOPPED_THRE = 1e-9f;
  // -- tpts
  static constexpr float TPTS_LEFT  = 2.0f;
  static constexpr float TPTS_RIGHT = -2.0f;

  static const std::map<const SensorId, const int> SENSOR_ID_TO_CAMERA_ID = {{SensorId::CAMERA_RL, 10},
                                                                             {SensorId::CAMERA_FL, 11},
                                                                             {SensorId::CAMERA_FW, 12},
                                                                             {SensorId::CAMERA_FN, 13},
                                                                             {SensorId::CAMERA_FR, 14},
                                                                             {SensorId::CAMERA_RR, 15},
                                                                             {SensorId::CAMERA_RW, 16}};

  static const std::map<const SensorId, const bool> SENSORS_USED = {{SensorId::CAMERA_FW, true},
                                                                    {SensorId::CAMERA_FN, true},
                                                                    {SensorId::CAMERA_FL, true},
                                                                    {SensorId::CAMERA_FR, true},
                                                                    {SensorId::CAMERA_RW, true},
                                                                    {SensorId::CAMERA_RN, false},
                                                                    {SensorId::CAMERA_RL, true},
                                                                    {SensorId::CAMERA_RR, true}};

  static constexpr int ID_DEFAULT = 0;

  static constexpr float RADIUS = 2.0f;

} // namespace fs

#endif //PROJECT_HDT_DEFS_H_
