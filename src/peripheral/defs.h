#ifndef DEFS_H_
#define DEFS_H_

#include <map>
#include <perception_obstacles.pb.h>
#include <perception_camera.pb.h>

#include "types.h"
#include "project_defs.h"
#include "switcher.h"

namespace fs
{
#if FS_CHECK(CFG_LOAD_RECORDING)
  static constexpr int PROCESS_PERIOD_MS = 1;
#else
  static constexpr int PROCESS_PERIOD_MS = 50;
#endif

  // fix configs
  static constexpr char    NODE_NAME[]                = "uto_per_fs";
  static constexpr char    PROJECT_CONFIG_FILE[]      = "/opt/uto/config/project_config.prototxt";
  static constexpr int64_t INT_1E6                    = 1000000UL;
  static constexpr int64_t INT_1E9                    = 1000000000UL;
  static constexpr float   RAD2DEG                    = 180.f / M_PIf32;
  static constexpr int     MSG_BUFFER_SIZE            = 64;
  static constexpr int     EMO_BUFFER_SIZE            = 256;
  static constexpr int64_t SYNC_WARNING_NS            = 1000e6; // 1s
  static constexpr int64_t SYNC_EARLY_ARRIVAL_NS      = -30e6;  // -30ms
  static constexpr int     SYNC_MAX_WAIT_US           = 500e3;  // 0.5s
  static constexpr int     SYNC_MAX_LIDAR_BUFFER_SIZE = 10;
  static constexpr int     FISHEYE_IMAGE_WIDTH        = 1280;
  static constexpr int     FISHEYE_IMAGE_HEIGHT       = 720;
  static constexpr float   FISHEYE_IMAGE_MARGIN       = 45.F / RAD2DEG;
  static constexpr int     FISHEYE_INVALID_MARGIN     = 60;
  static constexpr float   GRID_SCALE                 = 0.1F;
  static constexpr float   GRID_SCALE_INV             = 1.F / GRID_SCALE;
  static constexpr int     GRID_MAP_SIZE_COLS         = 400;
  static constexpr int     EGO_INDEX_COL              = 200;
  static constexpr float   PUB_RANGE_LEFT_RIGHT       = 20.f; ///< coordinate: flu，unit: m
  static constexpr int     PUB_MIN_ROW                = EGO_INDEX_ROW - PUB_RANGE_FRONT * GRID_SCALE_INV;
  static constexpr int     PUB_MAX_ROW                = EGO_INDEX_ROW + PUB_RANGE_REAR * GRID_SCALE_INV;
  static constexpr int     PUB_MIN_COL                = EGO_INDEX_COL - PUB_RANGE_LEFT_RIGHT * GRID_SCALE_INV;
  static constexpr int     PUB_MAX_COL                = EGO_INDEX_COL + PUB_RANGE_LEFT_RIGHT * GRID_SCALE_INV;
  static constexpr float   OD_RELEVANT_RANGE          = EGO_INDEX_ROW * GRID_SCALE + 20.f;
  static constexpr float   CELL_SCALE_INV             = 1.f / 3.f;
  static constexpr int     CELL_ROWS                  = GRID_MAP_SIZE_ROWS * CELL_SCALE_INV + 1;
  static constexpr int     CELL_COLS                  = GRID_MAP_SIZE_COLS * CELL_SCALE_INV + 1;
  static constexpr float   GATES_LEN                  = 2.5f;
  static constexpr float   GATES_RANGE                = 2.5f;
  static constexpr int     ABNORMAL_COUNT_MAX         = 10;
  static constexpr int     ABNORMAL_COUNT_THRESH      = 5;
  static constexpr float   REGION_PASS_THROUGH_LEFT   = -2.0f;
  static constexpr float   REGION_PASS_THROUGH_RIGHT  = 2.0f;

  static_assert(PUB_MAX_ROW <= GRID_MAP_SIZE_ROWS, "pub range too large in row");
  static_assert(PUB_MIN_ROW >= 0, "pub range too small in row");
  static_assert(PUB_MAX_COL <= GRID_MAP_SIZE_COLS, "pub range too large in col");
  static_assert(PUB_MIN_COL >= 0, "pub range too small in col");

  // images
  static constexpr char IMAGE_CF_TOPIC[]          = "/hal/sensor/cam_fcf_jpeg";         ///< 前中前向120度相机jpeg图像
  static constexpr char IMAGE_FISHEYE_FCF_TOPIC[] = "/hal/sensor_raw/fisheye_fcf_jpeg"; ///< 鱼眼相机前视jpeg图像
  static constexpr char IMAGE_FISHEYE_FLL_TOPIC[] = "/hal/sensor_raw/fisheye_fll_jpeg"; ///< 鱼眼相机左前视jpeg图像
  static constexpr char IMAGE_FISHEYE_FRR_TOPIC[] = "/hal/sensor_raw/fisheye_frr_jpeg"; ///< 鱼眼相机右前视jpeg图像
  static constexpr char IMAGE_FISHEYE_BCB_TOPIC[] = "/hal/sensor_raw/fisheye_bcb_jpeg"; ///< 鱼眼相机后视jpeg图像
  static constexpr char IMAGE_FISHEYE_BLL_TOPIC[] = "/hal/sensor_raw/fisheye_bll_jpeg"; ///< 鱼眼相机左后视jpeg图像
  static constexpr char IMAGE_FISHEYE_BRR_TOPIC[] = "/hal/sensor_raw/fisheye_brr_jpeg"; ///< 鱼眼相机右后视jpeg图像

  // calib
  static constexpr char CALIB_CF_TOPIC[]          = "/calib/cam_fcf";
  static constexpr char CALIB_FISHEYE_FCF_TOPIC[] = "/calib/fisheye_fcf";
  static constexpr char CALIB_FISHEYE_FLL_TOPIC[] = "/calib/fisheye_fll";
  static constexpr char CALIB_FISHEYE_FRR_TOPIC[] = "/calib/fisheye_frr";
  static constexpr char CALIB_FISHEYE_BCB_TOPIC[] = "/calib/fisheye_bcb";
  static constexpr char CALIB_FISHEYE_BLL_TOPIC[] = "/calib/fisheye_bll";
  static constexpr char CALIB_FISHEYE_BRR_TOPIC[] = "/calib/fisheye_brr";

  // camera fs
  static constexpr char FS_CF_TOPIC[]          = "/perception/camera_freespace";
  static constexpr char FS_FISHEYE_FCF_TOPIC[] = "/perception/fisheye_cam_fcf/camera_freespace";
  static constexpr char FS_FISHEYE_FLL_TOPIC[] = "/perception/fisheye_cam_fll/camera_freespace";
  static constexpr char FS_FISHEYE_FRR_TOPIC[] = "/perception/fisheye_cam_frr/camera_freespace";
  static constexpr char FS_FISHEYE_BCB_TOPIC[] = "/perception/fisheye_cam_bcb/camera_freespace";
  static constexpr char FS_FISHEYE_BLL_TOPIC[] = "/perception/fisheye_cam_bll/camera_freespace";
  static constexpr char FS_FISHEYE_BRR_TOPIC[] = "/perception/fisheye_cam_brr/camera_freespace";

  static constexpr char FUSION_FS_TOPIC[]       = "/perception/perception_freespace";
  static constexpr char GATES_TOPIC[]           = "/perception/perception_gates";
  static constexpr char LIDAR_FS_TOPIC[]        = "/perception/lidar_freespace";
  static constexpr char VEHICLE_POSE_TOPIC[]    = "/locator/vehicle_pose";
  static constexpr char MECHANICAL_INFO_TOPIC[] = "/vehicle_config/mechanical_info";
  static constexpr char SENSOR_TABLE_TOPIC[]    = "/sensor_table/sensor_table";
  static constexpr char OD_TOPIC[]              = "/perception/fused_objects"; ///< 融合OD输出结果
  static constexpr char MONITOR_TOPIC[]         = "/perception/fusion_fs_status";
  static constexpr char ROAD_MODEL_TOPIC[]      = "/perception/lidar_road_model";

  // vis topics
  static constexpr char VIS_FRONT_VIEW_TOPIC[]               = "/fs/vis/front_view_canvas";
  static constexpr char VIS_FISHEYE_FCF_VIEW_TOPIC[]         = "/fs/vis/fisheye_fcf_view_canvas";
  static constexpr char VIS_FISHEYE_FLL_VIEW_TOPIC[]         = "/fs/vis/fisheye_fll_view_canvas";
  static constexpr char VIS_FISHEYE_FRR_VIEW_TOPIC[]         = "/fs/vis/fisheye_frr_view_canvas";
  static constexpr char VIS_FISHEYE_BLL_VIEW_TOPIC[]         = "/fs/vis/fisheye_bll_view_canvas";
  static constexpr char VIS_FISHEYE_BRR_VIEW_TOPIC[]         = "/fs/vis/fisheye_brr_view_canvas";
  static constexpr char VIS_FISHEYE_BCB_VIEW_TOPIC[]         = "/fs/vis/fisheye_bcb_view_canvas";
  static constexpr char VIS_LIDAR_POINT_CLOUD_TOPIC[]        = "/fs/vis/lidar_point_cloud";
  static constexpr char VIS_FUSION_POINT_CLOUD_TOPIC[]       = "/fs/vis/fusion_point_cloud";
  static constexpr char VIS_PASS_THROUGH_POINT_CLOUD_TOPIC[] = "/fs/vis/pass_through_point_cloud";
  static constexpr char VIS_PERCEPTION_FREESPACE_TOPIC[]     = "/fs/vis/perception_freespace";
  static constexpr char VIS_MARKER_TOPIC[]                   = "/fs/vis/marker_array_canvas";
  static constexpr char VIS_ROAD_MODEL_TOPIC[]               = "fs/vis/road_model_point_cloud";

  // tuning parameters
  static constexpr int PRIOR_LOGIT          = -2;
  static constexpr int LOGIT_OBSTACLE_LIDAR = 10;
  static constexpr int LOGIT_NOISE_LIDAR    = 4;
  static constexpr int LOGIT_FREE_CAMERA    = -7;

  static constexpr int MAX_LOGIT = 80;
  static constexpr int MIN_LOGIT = -10;

  static constexpr int LOGIT_OCCUPIED_THRESH = 20;

  static const std::map<const SensorId, const int> SENSOR_ID_TO_CAMERA_ID = {
    {SensorId::CAMERA_CENTER_FRONT, 12},
#if FS_CHECK(CFG_USE_FISHEYE_FS)
    {SensorId::FISHEYE_FCF, 21},
    {SensorId::FISHEYE_FLL, 20},
    {SensorId::FISHEYE_FRR, 22},
    {SensorId::FISHEYE_BCB, 24},
    {SensorId::FISHEYE_BLL, 25},
    {SensorId::FISHEYE_BRR, 23},
#endif
  };

  static const std::map<const SensorId, const std::string> SENSOR_ID_TO_STRING = {{SensorId::CAMERA_CENTER_FRONT, "cf"},
#if FS_CHECK(CFG_USE_FISHEYE_FS)
                                                                                  {SensorId::FISHEYE_FCF, "fisheye fcf"},
                                                                                  {SensorId::FISHEYE_FLL, "fisheye fll"},
                                                                                  {SensorId::FISHEYE_FRR, "fisheye frr"},
                                                                                  {SensorId::FISHEYE_BCB, "fisheye bcb"},
                                                                                  {SensorId::FISHEYE_BLL, "fisheye bll"},
                                                                                  {SensorId::FISHEYE_BRR, "fisheye brr"},
#endif
                                                                                  {SensorId::LIDAR, "lidar"}};

  static const std::map<const SafetyZone, const std::string> SAFETY_ZONE_TO_STRING = {{SafetyZone::DANGER, "danger"},
                                                                                      {SafetyZone::CAUTION, "caution"},
                                                                                      {SafetyZone::IGNORANCE, "ignore"}};

  static constexpr float DANGER_ZONE_X  = 15.F;
  static constexpr float DANGER_ZONE_Y  = 2.F;
  static constexpr float CAUTION_ZONE_X = 25.F;
  static constexpr float CAUTION_ZONE_Y = 4.F;

  static const std::map<const ObstacleClass, const float> DANGER_ZONE_DILATION_Y = {{ObstacleClass::VEHICLE, 0.5f},
                                                                                    {ObstacleClass::VRU, 0.5f},
                                                                                    {ObstacleClass::STATIC_OBJECT, 0.1f},
                                                                                    {ObstacleClass::CRANE, 0.2f}};

  static const std::map<const ObstacleClass, const float> DANGER_ZONE_DILATION_X = {{ObstacleClass::VEHICLE, 1.f},
                                                                                    {ObstacleClass::VRU, 1.f},
                                                                                    {ObstacleClass::STATIC_OBJECT, 0.2f},
                                                                                    {ObstacleClass::CRANE, 0.35f}};

  static const std::map<const ObstacleClass, const float> CAUTION_ZONE_DILATION_Y = {{ObstacleClass::VEHICLE, 1.0f},
                                                                                     {ObstacleClass::VRU, 1.0f},
                                                                                     {ObstacleClass::STATIC_OBJECT, 0.2f},
                                                                                     {ObstacleClass::CRANE, 0.35f}};

  static const std::map<const ObstacleClass, const float> CAUTION_ZONE_DILATION_X = {{ObstacleClass::VEHICLE, 1.5f},
                                                                                     {ObstacleClass::VRU, 2.f},
                                                                                     {ObstacleClass::STATIC_OBJECT, 0.2f},
                                                                                     {ObstacleClass::CRANE, 0.35f}};

  static const std::map<const ObstacleClass, const float> IGNORANCE_ZONE_DILATION_Y = {{ObstacleClass::VEHICLE, 2.0f},
                                                                                       {ObstacleClass::VRU, 2.0f},
                                                                                       {ObstacleClass::STATIC_OBJECT, 0.2f},
                                                                                       {ObstacleClass::CRANE, 0.35f}};

  static const std::map<const ObstacleClass, const float> IGNORANCE_ZONE_DILATION_X_LOW = {{ObstacleClass::VEHICLE, 2.f},
                                                                                           {ObstacleClass::VRU, 2.f},
                                                                                           {ObstacleClass::STATIC_OBJECT, 0.2f},
                                                                                           {ObstacleClass::CRANE, 0.35f}};

  static const std::map<const ObstacleClass, const float> IGNORANCE_ZONE_DILATION_X_HIGH = {{ObstacleClass::VEHICLE, 5.f},
                                                                                            {ObstacleClass::VRU, 5.f},
                                                                                            {ObstacleClass::STATIC_OBJECT, 0.2f},
                                                                                            {ObstacleClass::CRANE, 0.35f}};

  static const std::map<const ObstacleClass, const float> IGNORANCE_ZONE_DILATION_X_FACTOR = {{ObstacleClass::VEHICLE, 0.05f},
                                                                                              {ObstacleClass::VRU, 0.07f},
                                                                                              {ObstacleClass::STATIC_OBJECT, 0.0f},
                                                                                              {ObstacleClass::CRANE, 0.0f}};

  static constexpr CameraClass cameraFreespacePointType2CameraClass(const uto::proto::CameraFreespacePoint::Type src)
  {
    CameraClass dst = CameraClass::INVALID;

    switch(src)
    {
    case uto::proto::CameraFreespacePoint_Type_INVALID:
    case uto::proto::CameraFreespacePoint_Type_WIPER:
      dst = CameraClass::INVALID;
      break;
    case uto::proto::CameraFreespacePoint_Type_VPR:
      dst = CameraClass::VPR;
      break;
    case uto::proto::CameraFreespacePoint_Type_CURB:
    case uto::proto::CameraFreespacePoint_Type_CONE:
    case uto::proto::CameraFreespacePoint_Type_BARRICADE:
    case uto::proto::CameraFreespacePoint_Type_OTHER:
      dst = CameraClass::STATIC_BORDER;
      break;
    case uto::proto::CameraFreespacePoint_Type_DARK:
    case uto::proto::CameraFreespacePoint_Type_RAIN_FOG:
    case uto::proto::CameraFreespacePoint_Type_DUST:
      dst = CameraClass::NON_OBSTACLE;
      break;
    default:
      dst = CameraClass::INVALID;
    }

    return dst;
  }

  static constexpr ObstacleClass perceptionClass2ObjectClass(const uto::proto::SubObstacle_ObstacleType perceptionClass)
  {
    ObstacleClass objClass = ObstacleClass::MAX_NUM;
    switch(perceptionClass)
    {
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TYPE_UNKNOWN:
      objClass = ObstacleClass::STATIC_OBJECT;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_PEDESTRIAN:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_PEDESTRIAN_NONSTANDING:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_RIDER:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BICYCLE:
      objClass = ObstacleClass::VRU;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_CAR:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BUS:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRUCK:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_HEAVY_EQUIPMENT:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRACTOR:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRAILER:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_REACH_STACKER:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_REACH_STACKER_WITH_CONTAINER:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_STACKER:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_STACKER_WITH_CONTAINER:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_AIV:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_EXCAVATOR:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BULLDOZER:
      objClass = ObstacleClass::VEHICLE;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_CONES:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_FALLEN_CONE:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_BOX:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_FRAME:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_STATION:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_SPRINKLER:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_WARNING_TRIANGLE:
      objClass = ObstacleClass::STATIC_OBJECT;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_RTG:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_QC:
      objClass = ObstacleClass::CRANE;
      break;
    default:
      // do nothing
      break;
    }
    return objClass;
  }

  namespace geofence
  {
    namespace la
    {
      //高架桥电子围栏
      static const std::vector<std::vector<Eigen::Vector2d>> TUNNEL_ELECTRONIC_FENCE = {{Eigen::Vector2d{-139.20090380853117, -140.9870775419565},   //[115.968928, 32.327075],
                                                                                         Eigen::Vector2d{-138.25953977403051, -155.40312134522483},  //[115.968938, 32.326945],
                                                                                         Eigen::Vector2d{-260.85116184638275, -160.26887906184922},  //[115.9676360, 32.3269011],
                                                                                         Eigen::Vector2d{-261.4628406498344, -147.25007852811896}}}; //[115.9676295, 32.3270185],

      //地磅电子围栏
      static const std::vector<std::vector<Eigen::Vector2d>> WEIGHBRIDGE_ELECTRONIC_FENCE = {{Eigen::Vector2d{-388.6599833432116, -186.3466904909233},    //[115.9662786, 32.3266659],
                                                                                              Eigen::Vector2d{-388.2364592293144, -191.03746119467232},   //[115.9662831, 32.3266236],
                                                                                              Eigen::Vector2d{-466.7915888290158, -196.54549146456978},   //[115.9654488, 32.3265739],
                                                                                              Eigen::Vector2d{-467.0456429645488, -192.8971152032728}},   //[115.9654461, 32.3266068]
                                                                                             {Eigen::Vector2d{-488.7494824751389, -208.021829156155},     //[115.9652156, 32.3264704],
                                                                                              Eigen::Vector2d{-488.58677221807795, -211.75891549031758},  //[115.96521733, 32.3264367],
                                                                                              Eigen::Vector2d{-410.5333592426956, -206.04033880913866},   //[115.9660463, 32.3264883],
                                                                                              Eigen::Vector2d{-410.8062531147573, -202.09255340179175}}}; //[115.9660434, 32.3265239]

    } // namespace la
  }   // namespace geofence

  namespace diag
  {
    static constexpr int INPUT_DELAY_ERROR_COUNT_THRESH     = 5;      ///< Monitor count value
    static constexpr int INPUT_TIMEOUT_ERROR_COUNT_THRESH   = 10;     ///< Monitor time threshold value, 500ms
    static constexpr int PROCESS_TIMEOUT_ERROR_COUNT_THRESH = 20;     ///< Monitor time threshold value, 1s
    static constexpr int SYNC_DELAY_THRESH_US               = 1000E3; ///< Monitor time sync value, us
    static constexpr int SYNC_EARLY_ARRIVAL_THRESH_US       = -30E3;  ///< Monitor time sync value, us

  } // namespace diag

} // namespace fs

#endif //DEFS_H_
