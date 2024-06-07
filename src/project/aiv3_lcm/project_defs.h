#ifndef PROJECT_AIV3_LCM_DEFS_H_
#define PROJECT_AIV3_LCM_DEFS_H_
#include <cstdint>
namespace fs
{
  // -- lcm port
  static constexpr int     LCM_PORT_DEFAULT             = 7667;
  static constexpr int     LCM_PORT_NORMAL_CAMERA_FS    = 7172;
  static constexpr int     LCM_PORT_LINE_CONTROL        = 7136;
  static constexpr int     LCM_PORT_Planner_Gridmap_AIV = 7130;
  static constexpr int     LCM_PORT_Error_Code          = 7667; // -- 7188
  static constexpr int     LCM_PORT_TRANSFORM           = 7121;
  static constexpr int     LCM_PORT_VCU_VEHICLE_INFO    = 7119;
  static constexpr int     LCM_PORT_LIDAR_GRIDMAP       = 7116;
  static constexpr int     LCM_PORT_CAM_FS_BEV          = 7667; // -- 7192
  static constexpr int     LCM_PORT_LANE                = 7178;
  static constexpr int64_t TIME_SCALE_MS                = 1000L;

  static constexpr int LCM_PORT_VOT = 7148;

  // lcm topic
  static constexpr char TOPIC_TRANSFORM[]          = "TRANSFORM1";
  static constexpr char TOPIC_LIDAR_GRIDMAP[]      = "Lidar_Gridmap";
  static constexpr char TOPIC_NORMAL_CAMERA_FS[]   = "NORMAL_CAMERA_FS";
  static constexpr char TOPIC_CAM_FS_BEV[]         = "CAM_FS_BEV";
  static constexpr char TOPIC_VCU_VEHICLE_INFO[]   = "VCU_VEHICLE_INFO";
  static constexpr char TOPIC_LANE_CONTROL[]       = "LANE_CONTROL";
  static constexpr char TOPIC_INTEGRATEDPOSITION[] = "IntegratedPosition";
  static constexpr char TOPIC_LOC_MONITOR[]        = "localization2Monitor";
  static constexpr char TOPIC_LANE[]               = "PERCEPTION_LINES";
#if FS_CHECK(CFG_DEBUG_PERCEPTION_FREESPACES)
  static constexpr char TOPIC_PLANNER_GRIDMAP_AIV[] = "Planner_Gridmap_AIV";
#endif
  static constexpr char TOPIC_VOT[] = "PERCEPTION_OBSTACLES";

  static constexpr int CAMERA_FRONT_CENTER_WIDTH  = 3840;
  static constexpr int CAMERA_FRONT_CENTER_HEIGHT = 2160;
  static constexpr int CAMERA_WIDTH               = 2880;
  static constexpr int CAMERA_HEIGHT              = 1860;
  static constexpr int BEV_WIDTH                  = 800 * 2;
  static constexpr int BEV_HEIGHT                 = 1400 * 2;
  static constexpr int HALF_EGO_WIDTH             = 14; // -- grid
  static constexpr int HALF_EGO_LENGTH            = 75; // -- grid
  static constexpr int FACTOR                     = 50; // -- m

  static constexpr float HALF_EGO_WIDTH_M = 1.4F; // -- m

  static constexpr int REMOVE_REGION_FRONT = 20;
  static constexpr int REMOVE_REGION_BACK  = -20;
  static constexpr int REMOVE_REGION_LEFT  = 2;
  static constexpr int REMOVE_REGION_RIGHT = -2;

  static constexpr int FS_ROW_OUT     = 500;
  static constexpr int FS_COL_OUT     = 200;
  static constexpr int FS_CAR_ROW_OUT = FS_ROW_OUT / 2 - 1;
  static constexpr int FS_CAR_COL_OUT = FS_COL_OUT / 2 - 1;

  // -- fw remove noise
  static constexpr float FW_THRE = 1000.0f;

  // -- vot box expand
  static constexpr float STOPPED_THRE = 0.1f;

  // -- tpts
  static constexpr float TPTS_LEFT  = -2.0f;
  static constexpr float TPTS_RIGHT = 2.0f;

  static const std::map<const SensorId, const bool> SENSORS_USED = {{SensorId::CAMERA_FW, true},
                                                                    {SensorId::CAMERA_FN, true},
                                                                    {SensorId::CAMERA_FL, true},
                                                                    {SensorId::CAMERA_FR, true},
                                                                    {SensorId::CAMERA_RW, true},
                                                                    {SensorId::CAMERA_RN, true},
                                                                    {SensorId::CAMERA_RL, true},
                                                                    {SensorId::CAMERA_RR, true}};

  static const std::map<const SensorId, const std::string> SENSOR_ID_TO_STRING = {{SensorId::CAMERA_FW, "fc120"},
                                                                                  {SensorId::CAMERA_FN, "fc60"},
                                                                                  {SensorId::CAMERA_FL, "fl"},
                                                                                  {SensorId::CAMERA_FR, "fr"},
                                                                                  {SensorId::CAMERA_RL, "rl"},
                                                                                  {SensorId::CAMERA_RR, "rr"},
                                                                                  {SensorId::CAMERA_RW, "rc120"},
                                                                                  {SensorId::CAMERA_RN, "rc60"}};

  static constexpr int ID_DEFAULT = -1;

  static constexpr float RADIUS = 2.0f;
} // namespace fs

#endif //PROJECT_AIV3_LCM_DEFS_H_
