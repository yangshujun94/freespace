#ifndef DEFS_H_
#define DEFS_H_

#include <cstdint>
#include <map>
#include "types.h"
#include "switcher.h"
#include "project_defs.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <common/log.h>

#if FS_CHECK(CFG_ROS2)

#include "perception_obstacles.pb.h"
#include "perception_freespace.pb.h"

#endif

namespace fs
{
  // fix configs
  static constexpr char    NODE_NAME[]                  = "uto_per_fs";
  static constexpr char    PROJECT_CONFIG_FILE[]        = "/opt/uto/config/project_config.prototxt";
  static constexpr int64_t INT_1E6                      = 1000000L;
  static constexpr int64_t INT_1E9                      = 1000000000L;
  static constexpr int64_t DELAY_MS                     = 1000L;
  static constexpr float   SQRT2                        = 1.414f;
  static constexpr float   RAD2DEG                      = 180.f / M_PIf32;
  static constexpr int     PROCESS_PERIOD_MS            = 50;
  static constexpr int     MSG_BUFFER_SIZE              = 64;
  static constexpr int     GATES_BUFFER_SIZE            = 4;
  static constexpr int     EMO_BUFFER_SIZE              = 512;
  static constexpr int     SYNC_WARNING_NS              = 1e9L; // 1s
  static constexpr int     SYNC_MAX_CYCLE_WITHOUT_INPUT = 5;
  static constexpr int     SYNC_MAX_LIDAR_BUFFER_SIZE   = 0;
  static constexpr int     GRID_MAP_SIZE                = 2048;
  static constexpr int     HALF_GRID_MAP_SIZE           = GRID_MAP_SIZE / 2;
  static constexpr int     HALF_GRID_MAP_SIZE_M         = GRID_MAP_SIZE / 2 * 0.1;
  static constexpr float   GRID_SCALE                   = 0.1F;
  static constexpr int     GRID_SCALE_INV               = 10;
  static constexpr float   GRID_MAP_M                   = GRID_MAP_SIZE * GRID_SCALE;

  static constexpr int LOC_VALID_NUM = 10;

  // -- probility
  static constexpr int   LOGIT_UP_CAN_MOVING_THRE = 3;
  static constexpr int   LOGIT_UP_STOPPED_THRE    = 11;
  static constexpr float LOGIT_DOWN_ALL_THRE      = 0.11f;
  static constexpr int   FRAMES_DESCEND           = 3; // -- while frames >= this number , all gridmap descend probability -0.1

  //  // -- vot box expand
  //  static constexpr float STOPPED_THRE = 0.1f;

  //  vision valid distance
  static constexpr float VISION_DISTANCE = 40.0f;

  // -- tpts region
  static constexpr int HALF_TPTS_WIDTH  = HALF_EGO_WIDTH + 400;
  static constexpr int HALF_TPTS_LENGTH = HALF_EGO_LENGTH + 800;

  // -- ratio top and bottom update
  static constexpr float HISTORY_RATIO = 0.9f;
  static constexpr float CURRENT_RATIO = 1.f - HISTORY_RATIO;

  // -- other
  static constexpr int   NUM_RAY       = 1440;
  static constexpr int   OFFSET_FS     = 50;   // -- due to extrinsic parameters fluctuation and time synchronization error, reserve some pixels for noise elimination
  static constexpr int   OFFSET_FISHFS = 0;    // -- due to extrinsic parameters fluctuation and time synchronization error, reserve some pixels for noise elimination
  static constexpr float GATES_LEN     = 2.5f; // -- 4/2 m --> 3.2/1.6 m
  static constexpr float GATES_RANGE   = 2.5f; // -- 5/2 m
  static constexpr float BLIND_EXPAND  = 1.0f; // -- m

  //fish eye
  // static constexpr float VIS_IMAGE_VIEW_SCALING = 0.25F;
  static constexpr float VIS_IMAGE_VIEW_SCALING = 1.0F;

  static const cv::Scalar COLOR_BLACK(50, 50, 50);
  static const cv::Scalar COLOR_GRAY(130, 130, 130);
  static const cv::Scalar COLOR_B(255, 0, 0);
  static const cv::Scalar COLOR_G(0, 255, 0);
  static const cv::Scalar COLOR_R(0, 0, 255);
  static const cv::Scalar COLOR_Y(0, 255, 255);
  static const cv::Scalar COLOR_W(255, 255, 255);
  static const cv::Scalar COLOR_P(160, 32, 240);
  static const cv::Scalar COLOR_ORANGE(0, 165, 255);
  static const cv::Scalar COLOR_GOLD(0, 215, 255);
  static const cv::Scalar COLOR_PALE_GREEN(152, 251, 152);
  static const cv::Scalar COLOR_DEEP_SKY(205, 191, 0);
  static const cv::Scalar COLOR_OLIVE_DRAB(107, 142, 35);
  static const cv::Scalar COLOR_BROWN(0, 75, 152);
  static const cv::Scalar COLOR_GREY_BLUE(230, 224, 176);
  static const cv::Scalar COLOR_DARK_BLACK(0, 0, 0);

  static ProjectLocation str2ProjectLocation(const std::string& projStr)
  {
    ProjectLocation projectLoc = ProjectLocation::UNKNOWN;
    if(projStr == "WS")
    {
      projectLoc = ProjectLocation::WS;
    }
    else if(projStr == "Y4")
    {
      projectLoc = ProjectLocation::WS;
    }
    else if(projStr == "DX")
    {
      projectLoc = ProjectLocation::DX;
    }
    else if(projStr == "QZ")
    {
      projectLoc = ProjectLocation::QZ;
    }
    else if(projStr == "QZ_ORIN")
    {
      projectLoc = ProjectLocation::QZ;
    }
    else if(projStr == "TC")
    {
      projectLoc = ProjectLocation::TC;
    }
    else if(projStr == "LJ")
    {
      projectLoc = ProjectLocation::LJ;
    }
    else
    {
      // do nothing
    }
    return projectLoc;
  }

  static const std::map<const ProjectLocation, const std::string> PROJECT_LOCATION_TO_STRING = {{ProjectLocation::UNKNOWN, "default"},
                                                                                                {ProjectLocation::WS, "WS"},
                                                                                                {ProjectLocation::DX, "DX"},
                                                                                                {ProjectLocation::QZ, "QZ"},
                                                                                                {ProjectLocation::TC, "TC"},
                                                                                                {ProjectLocation::LJ, "LJ"},
                                                                                                {ProjectLocation::QD, "QD"}};

  static const std::map<const GridLabel, const std::string> CLASS_STRINGS = {{GridLabel::UNKNOWN, "unknown"},
                                                                             {GridLabel::VEHICLE, "vehicle"},
                                                                             {GridLabel::EQUIPMENT_VEHICLE, "equipment_vehicle"},
                                                                             {GridLabel::PEDESTRIAN, "pedestrian"},
                                                                             {GridLabel::CYCLIST, "cyclist"},
                                                                             {GridLabel::RTG_QC, "rtg-qc"},
                                                                             {GridLabel::OCCUPIED, "occupied"},
                                                                             {GridLabel::UNOCCUPIED, "unoccupied"},
                                                                             {GridLabel::LOCK_BOX, "lock_box"},
                                                                             {GridLabel::LOCK_FRAME, "lock_frame"},
                                                                             {GridLabel::LOCK_STATION, "lock_station"},
                                                                             {GridLabel::OTHER, "other"},
                                                                             {GridLabel::PUDDLE, "puddle"},
                                                                             {GridLabel::POTHOLE, "pothole"},
                                                                             {GridLabel::GRASS, "grass"},
                                                                             {GridLabel::TREE_BRANCH, "tree_branch"},
                                                                             {GridLabel::AIV, "aiv"},
                                                                             {GridLabel::CONE, "cone"}};

  static const std::map<const GridLabel, const SubLabel> CLASS_TO_SUBCLASS = {{GridLabel::UNKNOWN, SubLabel::BARRICADE},
                                                                              {GridLabel::VEHICLE, SubLabel::VEHICLE},
                                                                              {GridLabel::EQUIPMENT_VEHICLE, SubLabel::EQUIPMENT},
                                                                              {GridLabel::PEDESTRIAN, SubLabel::PEDESTRIAN},
                                                                              {GridLabel::CYCLIST, SubLabel::VRU},
                                                                              {GridLabel::RTG_QC, SubLabel::EQUIPMENT},
                                                                              {GridLabel::OCCUPIED, SubLabel::BARRICADE},
                                                                              {GridLabel::UNOCCUPIED, SubLabel::BARRICADE},
                                                                              {GridLabel::LOCK_BOX, SubLabel::BARRICADE},
                                                                              {GridLabel::LOCK_FRAME, SubLabel::BARRICADE},
                                                                              {GridLabel::LOCK_STATION, SubLabel::BARRICADE},
                                                                              {GridLabel::OTHER, SubLabel::BARRICADE},
                                                                              {GridLabel::PUDDLE, SubLabel::BARRICADE},
                                                                              {GridLabel::POTHOLE, SubLabel::BARRICADE},
                                                                              {GridLabel::GRASS, SubLabel::BARRICADE},
                                                                              {GridLabel::TREE_BRANCH, SubLabel::BARRICADE},
                                                                              {GridLabel::AIV, SubLabel::VEHICLE},
                                                                              {GridLabel::CONE, SubLabel::BARRICADE}};

  static const std::map<const GridLabel, const cv::Scalar> CLASS_COLOR = {{GridLabel::UNKNOWN, COLOR_BLACK},
                                                                          {GridLabel::VEHICLE, COLOR_GRAY},
                                                                          {GridLabel::EQUIPMENT_VEHICLE, COLOR_GRAY},
                                                                          {GridLabel::PEDESTRIAN, COLOR_DARK_BLACK},
                                                                          {GridLabel::CYCLIST, COLOR_P},
                                                                          {GridLabel::RTG_QC, COLOR_ORANGE},
                                                                          {GridLabel::OCCUPIED, COLOR_PALE_GREEN},
                                                                          {GridLabel::UNOCCUPIED, COLOR_DEEP_SKY},
                                                                          {GridLabel::LOCK_BOX, COLOR_OLIVE_DRAB},
                                                                          {GridLabel::LOCK_FRAME, COLOR_BROWN},
                                                                          {GridLabel::LOCK_STATION, COLOR_GREY_BLUE},
                                                                          {GridLabel::OTHER, COLOR_BLACK},
                                                                          {GridLabel::AIV, COLOR_GRAY},
                                                                          {GridLabel::CONE, COLOR_B}};

  static const std::map<const std::string, const CameraModel> STRING_TO_CAMERA_MODEL = {{"PINHOLE", CameraModel::PINHOLE},
                                                                                        {"FISHEYE", CameraModel::FISHEYE}};

  static const std::map<const CameraFSType, const cv::Scalar> CAMERA_FS_COLOR = {{CameraFSType::INVALID, cv::Scalar(0, 0, 0)},       // -- black
                                                                                 {CameraFSType::CURB, cv::Scalar(255, 0, 0)},        // -- blue
                                                                                 {CameraFSType::VPR, cv::Scalar(244, 35, 232)},      // -- Purple
                                                                                 {CameraFSType::CONE, cv::Scalar(0, 255, 0)},        // -- green
                                                                                 {CameraFSType::BARRICADE, cv::Scalar(0, 0, 255)},   // -- red
                                                                                 {CameraFSType::OTHER, cv::Scalar(0, 255, 255)},     // -- Yellow
                                                                                 {CameraFSType::WIPPER, cv::Scalar(255, 255, 255)},  // -- White
                                                                                 {CameraFSType::DARK, cv::Scalar(128, 0, 128)},      // -- Magenta
                                                                                 {CameraFSType::RAIN_FOG, cv::Scalar(0, 165, 255)},  // -- Orange
                                                                                 {CameraFSType::DUST, cv::Scalar(255, 255, 0)},      // -- Cyan-Yellow
                                                                                 {CameraFSType::UNINIT, cv::Scalar(128, 128, 128)}}; // -- gray

#if FS_CHECK(CFG_ROS2)
  static GridLabel perceptionClass2ObjectClass(const uto::proto::SubObstacle_ObstacleType perceptionClass)
  {
    GridLabel objClass = GridLabel::OCCUPIED;
    switch(perceptionClass)
    {
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_RIDER:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BICYCLE:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRICYCLE:
      objClass = GridLabel::CYCLIST;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_PEDESTRIAN:
      objClass = GridLabel::PEDESTRIAN;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_CAR:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BUS:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRUCK:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRACTOR:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRAILER:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_WARNING_TRIANGLE:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_FORKLIFT:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_FORKLIFT_WITH_CARGO:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_EXCAVATOR:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BULLDOZER:
      objClass = GridLabel::VEHICLE;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_CONES:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BALL:
      objClass = GridLabel::CONE;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_HEAVY_EQUIPMENT:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_STACKER:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_STACKER_WITH_CONTAINER:
      objClass = GridLabel::EQUIPMENT_VEHICLE;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_RTG:
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_QC:
      objClass = GridLabel::RTG_QC;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_PARKING_LOCK_DOWN:
      objClass = GridLabel::UNOCCUPIED;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_BOX:
      objClass = GridLabel::LOCK_BOX;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_FRAME:
      objClass = GridLabel::LOCK_FRAME;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_STATION:
      objClass = GridLabel::LOCK_STATION;
      break;
    case uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_AIV:
      objClass = GridLabel::AIV;
      break;
    default:
      break;
    }
    return objClass;
  }

  static const std::map<const SensorId, const std::vector<int>> SENSOR_ID_TO_AREA = {{SensorId::CAMERA_SFW, {1, 2, 3, 7, 8}},
                                                                                     {SensorId::CAMERA_SFL, {2, 7}},
                                                                                     {SensorId::CAMERA_SFR, {3, 8}},
                                                                                     {SensorId::CAMERA_SRL, {5, 7}},
                                                                                     {SensorId::CAMERA_SRR, {6, 8}},
                                                                                     {SensorId::CAMERA_SRW, {4, 5, 6}}

  };

  static const std::map<const SensorId, const std::string> SENSOR_ID_TO_STRING = {{SensorId::CAMERA_FW, "fw"}, // -- Don't read calibration
                                                                                  {SensorId::CAMERA_FN, "fn"},
                                                                                  {SensorId::CAMERA_FL, "fl"},
                                                                                  {SensorId::CAMERA_FR, "fr"},
                                                                                  {SensorId::CAMERA_RL, "rl"},
                                                                                  {SensorId::CAMERA_RR, "rr"},
                                                                                  {SensorId::CAMERA_RW, "rw"},
                                                                                  {SensorId::CAMERA_RN, "rn"},
#if FS_CHECK(CFG_USE_CAM_FISH_FS)
                                                                                  {SensorId::CAMERA_SFW, "sfw"},
                                                                                  {SensorId::CAMERA_SFL, "sfl"},
                                                                                  {SensorId::CAMERA_SFR, "sfr"},
                                                                                  {SensorId::CAMERA_SRL, "srl"},
                                                                                  {SensorId::CAMERA_SRR, "srr"},
                                                                                  {SensorId::CAMERA_SRW, "srw"},
#endif
                                                                                  {SensorId::CAMERA_TRACKED, "cod"},
                                                                                  {SensorId::LIDAR, "ldr"},
                                                                                  {SensorId::RADAR_FRONT, "rdrf"}};

  // -- gt class <--> inner label
#if FS_CHECK(CFG_USE_LIGHT_COMMON)
  static const std::map<const uto::proto::PerceptionGrid_Label, const std::string> FS_PROTO_CLASS_TO_STRING = {{uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN, "unknown"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNOCCUPIED, "unoccupied"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED, "occupied"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_ROAD_EDGE, "road_edge"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GATES, "gates"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN, "pedestrian"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE, "vehicle"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE, "crane"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_COVER_PLATE, "cover_plate"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CONES, "cones"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_BOX, "lock_box"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_FRAME, "lock_frame"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_STATION, "lock_station"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PUDDLE, "puddle"},   // 水坑
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_POTHOLE, "pothole"}, // 道路坑洼
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GRASS, "grass"},     // 草
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_TREE_BRANCH, "tree_branch"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED_MOVING, "occupied_moving"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN_MOVING, "pedestrian_moving"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE_MOVING, "vehicle_moving"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE_MOVING, "crane_moving"}}; // 树枝

  static const std::map<const uto::proto::PerceptionGrid_Label, const cv::Scalar> FS_PROTO_CLASS_TO_COLOR = {{uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN, COLOR_BLACK},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNOCCUPIED, COLOR_GRAY},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED, COLOR_B},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_ROAD_EDGE, COLOR_G},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GATES, COLOR_R},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN, COLOR_Y},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE, COLOR_W},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE, COLOR_P},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_COVER_PLATE, COLOR_ORANGE},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CONES, COLOR_GOLD},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_BOX, COLOR_PALE_GREEN},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_FRAME, COLOR_DEEP_SKY},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_STATION, COLOR_OLIVE_DRAB},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PUDDLE, COLOR_BROWN},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_POTHOLE, COLOR_BROWN},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GRASS, COLOR_DARK_BLACK},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_TREE_BRANCH, COLOR_DARK_BLACK},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED_MOVING, COLOR_B},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN_MOVING, COLOR_Y},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE_MOVING, COLOR_Y},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE_MOVING, COLOR_P}};

  static const std::map<const uto::proto::PerceptionGrid_Label, const GridLabel> FS_PROTO_CLASS_TO_CLASS = {{uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN, GridLabel::OCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNOCCUPIED, GridLabel::UNOCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED, GridLabel::OCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_ROAD_EDGE, GridLabel::UNOCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GATES, GridLabel::OCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN, GridLabel::PEDESTRIAN},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE, GridLabel::VEHICLE},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE, GridLabel::RTG_QC},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_COVER_PLATE, GridLabel::OCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CONES, GridLabel::CONE},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_BOX, GridLabel::LOCK_BOX},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_FRAME, GridLabel::LOCK_FRAME},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_STATION, GridLabel::LOCK_STATION},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PUDDLE, GridLabel::PUDDLE},   // 水坑
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_POTHOLE, GridLabel::POTHOLE}, // 道路坑洼
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GRASS, GridLabel::GRASS},     // 草
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_TREE_BRANCH, GridLabel::TREE_BRANCH},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED_MOVING, GridLabel::OCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN_MOVING, GridLabel::PEDESTRIAN},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE_MOVING, GridLabel::VEHICLE},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE_MOVING, GridLabel::RTG_QC}}; // 树枝

  static const std::map<const uto::proto::PerceptionGrid_Label, const MotionState> HDT2_LIDAR_CLASS_TO_MOTION_STATE = {{uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNOCCUPIED, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_ROAD_EDGE, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GATES, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_COVER_PLATE, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CONES, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_BOX, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_FRAME, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_STATION, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PUDDLE, MotionState::STOPPED},  // 水坑
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_POTHOLE, MotionState::STOPPED}, // 道路坑洼
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GRASS, MotionState::STOPPED},   // 草
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_TREE_BRANCH, MotionState::STOPPED},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED_MOVING, MotionState::MOVING},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN_MOVING, MotionState::MOVING},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE_MOVING, MotionState::MOVING},
                                                                                                                       {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE_MOVING, MotionState::MOVING}}; // 树枝

  static const std::map<const GridLabel, const uto::proto::PerceptionGrid_Label> CLASS_TO_FS_PROTO_CLASS = {{GridLabel::UNKNOWN, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED},
                                                                                                            {GridLabel::VEHICLE, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE},
                                                                                                            {GridLabel::PEDESTRIAN, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN},
                                                                                                            {GridLabel::CYCLIST, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN},
                                                                                                            {GridLabel::CONE, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CONES},
                                                                                                            {GridLabel::RTG_QC, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE},
                                                                                                            {GridLabel::OCCUPIED, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED},
                                                                                                            {GridLabel::UNOCCUPIED, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNOCCUPIED},
                                                                                                            {GridLabel::LOCK_BOX, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_BOX},         // -- 锁钮箱
                                                                                                            {GridLabel::LOCK_FRAME, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_FRAME},     // -- 锁钮箱
                                                                                                            {GridLabel::LOCK_STATION, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_STATION}, // -- 锁钮箱
                                                                                                            {GridLabel::AIV, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE},
                                                                                                            {GridLabel::PUDDLE, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PUDDLE},
                                                                                                            {GridLabel::POTHOLE, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_POTHOLE},
                                                                                                            {GridLabel::GRASS, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GRASS},
                                                                                                            {GridLabel::TREE_BRANCH, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_TREE_BRANCH},
                                                                                                            {GridLabel::OTHER, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED}};

  static const std::map<const GridLabel, const uto::proto::PerceptionGrid_Label> MOVING_CLASS_TO_HDT2_LIDAR_CLASS = {{GridLabel::UNKNOWN, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED_MOVING},
                                                                                                                     {GridLabel::VEHICLE, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE_MOVING},
                                                                                                                     {GridLabel::PEDESTRIAN, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN_MOVING},
                                                                                                                     {GridLabel::CYCLIST, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN_MOVING},
                                                                                                                     {GridLabel::RTG_QC, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE_MOVING},
                                                                                                                     {GridLabel::OCCUPIED, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED_MOVING},
                                                                                                                     {GridLabel::AIV, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE_MOVING},
                                                                                                                     {GridLabel::OTHER, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED_MOVING}};
#else
  static const std::map<const uto::proto::PerceptionGrid_Label, const std::string> FS_PROTO_CLASS_TO_STRING = {{uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN, "unknown"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNOCCUPIED, "unoccupied"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED, "occupied"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_ROAD_EDGE, "road_edge"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GATES, "gates"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN, "pedestrian"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE, "vehicle"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE, "crane"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_COVER_PLATE, "cover_plate"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CONES, "cones"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_BOX, "lock_box"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_FRAME, "lock_frame"},
                                                                                                               {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_STATION, "lock_station"}};

  static const std::map<const uto::proto::PerceptionGrid_Label, const cv::Scalar> FS_PROTO_CLASS_TO_COLOR = {{uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN, COLOR_BLACK},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNOCCUPIED, COLOR_GRAY},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED, COLOR_B},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_ROAD_EDGE, COLOR_G},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GATES, COLOR_R},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN, COLOR_Y},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE, COLOR_DARK_BLACK},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE, COLOR_P},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_COVER_PLATE, COLOR_ORANGE},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CONES, COLOR_GOLD},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_BOX, COLOR_PALE_GREEN},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_FRAME, COLOR_DEEP_SKY},
                                                                                                             {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_STATION, COLOR_OLIVE_DRAB}};

  static const std::map<const uto::proto::PerceptionGrid_Label, const GridLabel> FS_PROTO_CLASS_TO_CLASS = {{uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN, GridLabel::TREE_BRANCH},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNOCCUPIED, GridLabel::UNOCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED, GridLabel::OCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_ROAD_EDGE, GridLabel::UNOCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_GATES, GridLabel::OCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN, GridLabel::PEDESTRIAN},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE, GridLabel::VEHICLE},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE, GridLabel::RTG_QC},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_COVER_PLATE, GridLabel::OCCUPIED},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CONES, GridLabel::CONE},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_BOX, GridLabel::LOCK_BOX},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_FRAME, GridLabel::LOCK_FRAME},
                                                                                                            {uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_STATION, GridLabel::LOCK_STATION}};

  static const std::map<const GridLabel, const uto::proto::PerceptionGrid_Label> CLASS_TO_FS_PROTO_CLASS = {{GridLabel::UNKNOWN, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN},
                                                                                                            {GridLabel::VEHICLE, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE},
                                                                                                            {GridLabel::EQUIPMENT_VEHICLE, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE},
                                                                                                            {GridLabel::PEDESTRIAN, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN},
                                                                                                            {GridLabel::CYCLIST, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_PEDESTRIAN},
                                                                                                            {GridLabel::CONE, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CONES},
                                                                                                            {GridLabel::RTG_QC, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_CRANE},
                                                                                                            {GridLabel::OCCUPIED, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED},
                                                                                                            {GridLabel::UNOCCUPIED, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNOCCUPIED},
                                                                                                            {GridLabel::LOCK_BOX, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_BOX},         // -- 锁钮箱
                                                                                                            {GridLabel::LOCK_FRAME, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_FRAME},     // -- 锁钮架
                                                                                                            {GridLabel::LOCK_STATION, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_LOCK_STATION}, // -- 锁钮箱
                                                                                                            {GridLabel::AIV, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_VEHICLE},
                                                                                                            {GridLabel::PUDDLE, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED},
                                                                                                            {GridLabel::POTHOLE, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED},
                                                                                                            {GridLabel::GRASS, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED},
                                                                                                            {GridLabel::TREE_BRANCH, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_UNKNOWN},
                                                                                                            {GridLabel::OTHER, uto::proto::PerceptionGrid::Label::PerceptionGrid_Label_OCCUPIED}};

  static const std::map<const GridLabel, const float> LIDAR_COMPENSATION = {{GridLabel::UNKNOWN, -6.0},
                                                                            {GridLabel::TREE_BRANCH, -6.0}};
#endif

  static const std::map<const uto::proto::SubObstacle_ObstacleType, const std::string> VOT_PROTO_CLASS_TO_STRING = {{uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TYPE_UNKNOWN, "type_unknown"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BICYCLE, "bicycle"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRICYCLE, "tricycle"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_PEDESTRIAN, "pedestrian"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_RIDER, "rider"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_CAR, "car"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BUS, "bus"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRUCK, "truck"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_CONES, "cones"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_HEAVY_EQUIPMENT, "heavy_equipment"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_RTG, "rtg"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRACTOR, "tractor"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRAILER, "trailer"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_PARKING_LOCK_UP, "parking_lock_up"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_PARKING_LOCK_DOWN, "parking_lock_down"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_WHEEL_STOPPER, "wheel_stopper"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_WARNING_TRIANGLE, "warning_triangle"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_QC, "qc"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_BOX, "lock_box"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_FRAME, "lock_frame"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_STATION, "lock_station"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BALL, "ball"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_STACKER, "stacker"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_STACKER_WITH_CONTAINER, "stacker_with_container"},
                                                                                                                    {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_AIV, "aiv"}};

  static const std::map<const uto::proto::SubObstacle_ObstacleType, const GridLabel> VOT_PROTO_CLASS_TO_CLASS = {{uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TYPE_UNKNOWN, GridLabel::OCCUPIED},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BICYCLE, GridLabel::CYCLIST},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRICYCLE, GridLabel::CYCLIST},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_PEDESTRIAN, GridLabel::PEDESTRIAN},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_RIDER, GridLabel::CYCLIST},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_CAR, GridLabel::VEHICLE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BUS, GridLabel::VEHICLE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRUCK, GridLabel::VEHICLE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_CONES, GridLabel::CONE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_HEAVY_EQUIPMENT, GridLabel::EQUIPMENT_VEHICLE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_RTG, GridLabel::RTG_QC},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRACTOR, GridLabel::VEHICLE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_TRAILER, GridLabel::VEHICLE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_PARKING_LOCK_UP, GridLabel::OCCUPIED},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_PARKING_LOCK_DOWN, GridLabel::UNOCCUPIED},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_WHEEL_STOPPER, GridLabel::OCCUPIED},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_WARNING_TRIANGLE, GridLabel::VEHICLE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_QC, GridLabel::RTG_QC},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_BOX, GridLabel::LOCK_BOX},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_FRAME, GridLabel::LOCK_FRAME},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_LOCK_STATION, GridLabel::LOCK_STATION},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_BALL, GridLabel::CONE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_STACKER, GridLabel::EQUIPMENT_VEHICLE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_STACKER_WITH_CONTAINER, GridLabel::EQUIPMENT_VEHICLE},
                                                                                                                 {uto::proto::SubObstacle_ObstacleType::SubObstacle_ObstacleType_AIV, GridLabel::AIV}};

  static const std::map<const MotionState, const uto::proto::PerceptionGrid_MoveStatus> MOVESTATUS_TO_HDT2_MOVESTATUS = {{MotionState::STOPPED, uto::proto::PerceptionGrid_MoveStatus::PerceptionGrid_MoveStatus_STATIONARY},
                                                                                                                         {MotionState::MOVING, uto::proto::PerceptionGrid_MoveStatus::PerceptionGrid_MoveStatus_MOVING}};
  static const std::map<const uto::proto::PerceptionGrid_MoveStatus, const MotionState> HDT2_MOVESTATUS_TO_MOVESTATUS = {{uto::proto::PerceptionGrid_MoveStatus::PerceptionGrid_MoveStatus_STATIONARY, MotionState::STOPPED},
                                                                                                                         {uto::proto::PerceptionGrid_MoveStatus::PerceptionGrid_MoveStatus_MOVING, MotionState::MOVING}};
#else
  static const std::map<const SensorId, const std::string> CAMERA_IMAGE_TO_LCM_NAME = {{SensorId::CAMERA_FN, "cam_fcf_far_jpeg"},
                                                                                       {SensorId::CAMERA_FW, "cam_fcf_jpeg"},
                                                                                       {SensorId::CAMERA_FL, "cam_flb_jpeg"},
                                                                                       {SensorId::CAMERA_FR, "cam_frb_jpeg"},
                                                                                       {SensorId::CAMERA_RW, "cam_bcb_jpeg"},
                                                                                       {SensorId::CAMERA_RN, "cam_bcb_far_jpeg"},
                                                                                       {SensorId::CAMERA_RL, "cam_blf_jpeg"},
                                                                                       {SensorId::CAMERA_RR, "cam_brf_jpeg"}};

  static const std::map<const SensorId, const int> CAMERA_PORT_TO_LCM_NAME = {{SensorId::CAMERA_FN, 7340},
                                                                              {SensorId::CAMERA_FW, 7341},
                                                                              {SensorId::CAMERA_FL, 7342},
                                                                              {SensorId::CAMERA_FR, 7343},
                                                                              {SensorId::CAMERA_RW, 7344},
                                                                              {SensorId::CAMERA_RN, 7345},
                                                                              {SensorId::CAMERA_RL, 7346},
                                                                              {SensorId::CAMERA_RR, 7347}};

  //// built-in --> lcm
  static const std::map<const GridLabel, const int8_t> CLASS_TO_FS_LCM_CLASS = {{GridLabel::UNKNOWN, 0},
                                                                                {GridLabel::VEHICLE, 6},
                                                                                {GridLabel::EQUIPMENT_VEHICLE, 6},
                                                                                {GridLabel::PEDESTRIAN, 3},
                                                                                {GridLabel::CYCLIST, 4},
                                                                                {GridLabel::CONE, 18},
                                                                                {GridLabel::RTG_QC, 10},
                                                                                {GridLabel::OCCUPIED, 0},
                                                                                {GridLabel::LOCK_BOX, 16},     // -- 锁钮箱
                                                                                {GridLabel::LOCK_FRAME, 21},   // -- 锁钮架子，集装箱架子，锁钮箱架子
                                                                                {GridLabel::LOCK_STATION, 16}, // -- 锁钮站
                                                                                {GridLabel::AIV, 13},
                                                                                {GridLabel::PUDDLE, 0},  // 水坑
                                                                                {GridLabel::POTHOLE, 0}, // 道路坑洼
                                                                                {GridLabel::GRASS, 0},   // 草
                                                                                {GridLabel::TREE_BRANCH, 0},
                                                                                {GridLabel::OTHER, 0}};

  //// lcm --> built-in
  static const std::map<const int8_t, const std::string> FS_LCM_CLASS_TO_STRING = {{0, "unknow"},
                                                                                   {3, "pedestrian"},
                                                                                   {4, "bike"},
                                                                                   {5, "car"},
                                                                                   {6, "truck"},
                                                                                   {10, "rtg"},
                                                                                   {13, "aiv"},
                                                                                   {15, "qc"},
                                                                                   {16, "lock_box"},
                                                                                   {18, "cones"},
                                                                                   {21, "lock_frame"}};

  static const std::map<const int8_t, const cv::Scalar> FS_LCM_CLASS_TO_COLOR = {{0, COLOR_BLACK},
                                                                                 {3, COLOR_GRAY},
                                                                                 {4, COLOR_B},
                                                                                 {5, COLOR_G},
                                                                                 {6, COLOR_R},
                                                                                 {10, COLOR_Y},
                                                                                 {13, COLOR_DARK_BLACK},
                                                                                 {15, COLOR_P},
                                                                                 {16, COLOR_ORANGE},
                                                                                 {18, COLOR_GOLD},
                                                                                 {21, COLOR_PALE_GREEN}};

  static const std::map<const int8_t, const GridLabel> FS_LCM_CLASS_TO_CLASS = {{0, GridLabel::OTHER},        // "unknow"
                                                                                {3, GridLabel::PEDESTRIAN},   // "pedestrian"
                                                                                {4, GridLabel::CYCLIST},      // "bike"
                                                                                {5, GridLabel::VEHICLE},      // "car"
                                                                                {6, GridLabel::VEHICLE},      // "truck"
                                                                                {10, GridLabel::RTG_QC},      // "rtg"
                                                                                {13, GridLabel::AIV},         // "aiv"
                                                                                {15, GridLabel::RTG_QC},      // "qc"
                                                                                {16, GridLabel::LOCK_BOX},    // "lock_box"
                                                                                {18, GridLabel::CONE},        // "cones"
                                                                                {21, GridLabel::LOCK_FRAME}}; // "lock_frame"

  //// -- aiv class <--> inner label  label=0 stable fsfddfdlabel=1: passing area label=2 moving label:3 -> road edge label=4: gate_fusion闸机杆 label=5: height悬空点
  static const std::map<const int8_t, const GridLabel> FS_LCM_LIDAR_CLASS_TO_BUILT_IN_CLASS = {{0, GridLabel::OCCUPIED},   // -- stable
                                                                                               {1, GridLabel::UNOCCUPIED}, // -- passing area
                                                                                               {2, GridLabel::OCCUPIED},   // -- moving
                                                                                               {3, GridLabel::UNOCCUPIED}, // -- road edge
                                                                                               {4, GridLabel::OCCUPIED},   // -- gate_fusion闸机杆
                                                                                               {5, GridLabel::OCCUPIED}};  // -- height悬空点

  static const std::map<const MotionState, const int8_t> MOTION_TO_LCM_MOTION = {{MotionState::STOPPED, 0},
                                                                                 {MotionState::MOVING, 1}};
  static const std::map<const int8_t, const MotionState> LCM_MOTION_TO_MOTION = {{0, MotionState::STOPPED},
                                                                                 {1, MotionState::MOVING}};

  //// -- vot --> built-in
  static const std::map<const int, const std::string> VOT_LCM_CLASS_TO_STRING = {{3, "pedestrian"},
                                                                                 {4, "cyclist"},
                                                                                 {5, "car"},
                                                                                 {6, "truck"},
                                                                                 {10, "rtg"},
                                                                                 {13, "aiv"},
                                                                                 {15, "qc"},
                                                                                 {16, "lock_box"},
                                                                                 {18, "cone"},
                                                                                 {19, "stacker"},                // -- stacker
                                                                                 {20, "stacker with container"}, // -- stacker with container
                                                                                 {21, "lock_frame"},
                                                                                 {26, "lock_station"}};

  static const std::map<const int, const GridLabel> VOT_LCM_CLASS_TO_CLASS = {{3, GridLabel::PEDESTRIAN},         // "pedestrian"
                                                                              {4, GridLabel::CYCLIST},            // "cyclist"
                                                                              {5, GridLabel::VEHICLE},            // "car"
                                                                              {6, GridLabel::VEHICLE},            // "truck"
                                                                              {10, GridLabel::RTG_QC},            // "rtg"
                                                                              {13, GridLabel::AIV},               // "aiv"
                                                                              {15, GridLabel::RTG_QC},            // "qc"
                                                                              {16, GridLabel::LOCK_BOX},          // "lock_box"
                                                                              {18, GridLabel::CONE},              // "cone"
                                                                              {19, GridLabel::EQUIPMENT_VEHICLE}, // "stacker"
                                                                              {20, GridLabel::EQUIPMENT_VEHICLE}, // "stacker with container"
                                                                              {21, GridLabel::LOCK_FRAME},        // "lock_frame"
                                                                              {26, GridLabel::LOCK_STATION}};     // "lock_station"

  //// -- bev
  // -- bev  <--> built-in
  static const std::map<const GridLabel, const int32_t> CLASS_TO_BEVCLASS = {{GridLabel::UNOCCUPIED, 0},   // -- 0   freespace       可通行区域
                                                                             {GridLabel::VEHICLE, 1},      // -- 1   vehicle         车
                                                                             {GridLabel::PEDESTRIAN, 2},   // -- 2   pedestrian      行人
                                                                                                           //                                                                           {GridLabel::OTHER          ,  3}, // -- 3   road_edge       普通路沿
                                                                                                           //                                                                           {GridLabel::OTHER          ,  4}, // -- 4   barricade       带高度的路沿
                                                                                                           //                                                                           {GridLabel::OTHER          ,  5}, // -- 5   container       集装箱
                                                                             {GridLabel::RTG_QC, 6},       // -- 6   RTG             吊车
                                                                             {GridLabel::OTHER, 7},        // -- 7   cover_plate     舱盖板
                                                                             {GridLabel::LOCK_BOX, 8},     // -- 8   lock_box        锁钮箱
                                                                             {GridLabel::LOCK_STATION, 9}, // -- 9   lock_station    锁钮站
                                                                             {GridLabel::LOCK_FRAME, 10},  // -- 10  container_shelf 集装箱架子，锁钮箱架子
                                                                                                           //                                                                           {GridLabel::CONTAINER_SHELF, 11}, // -- 11  hanging_shelf   吊架
                                                                                                           //                                                                           {GridLabel::OTHER          , 12}, // -- 12  black_roadstock 路中间的黑色障碍物
                                                                             {GridLabel::OTHER, 13},       // -- 13  other           路中间的黑色障碍物
                                                                             {GridLabel::CONE, 14}};       // -- 14  cone            锥桶

  static const std::map<const int32_t, const GridLabel> BEVCLASS_TO_CLASS = {{0, GridLabel::UNOCCUPIED},   // -- 0   freespace        UNOCCUPIED       可通行区域
                                                                             {1, GridLabel::VEHICLE},      // -- 1   vehicle          TRUCK            车
                                                                             {2, GridLabel::PEDESTRIAN},   // -- 2   pedestrian       PEDESTRIAN       行人
                                                                             {3, GridLabel::OTHER},        // -- 3   road_edge        OTHER            普通路沿
                                                                             {4, GridLabel::OTHER},        // -- 4   barricade        OTHER            带高度的路沿
                                                                             {5, GridLabel::OTHER},        // -- 5   container        OTHER            集装箱
                                                                             {6, GridLabel::RTG_QC},       // -- 6   RTG              RTG              吊车
                                                                             {7, GridLabel::OTHER},        // -- 7   cover_plate      OTHER            舱盖板
                                                                             {8, GridLabel::LOCK_BOX},     // -- 8   lock_box         LOCK_BOX         锁钮箱
                                                                             {9, GridLabel::LOCK_STATION}, // -- 9   lock_station     CONTAINER_SHELF  锁钮站
                                                                             {10, GridLabel::LOCK_FRAME},  // -- 10  container_shelf  CONTAINER_SHELF  集装箱架子，锁钮箱架子
                                                                             {11, GridLabel::RTG_QC},      // -- 11  hanging_shelf    OTHER            吊架
                                                                             {12, GridLabel::OTHER},       // -- 12  black_roadstock  OTHER            路中间的黑色障碍物
                                                                             {13, GridLabel::OTHER},       // -- 13  other            OTHER            其他
                                                                             {14, GridLabel::CONE}};       // -- 14  cone             CONE             锥桶
#endif

  static const std::map<const GridLabel, const MotionState> FS_BUILT_IN_CLASS_CAN_MOVING = {{GridLabel::UNKNOWN, MotionState::STOPPED},
                                                                                            {GridLabel::VEHICLE, MotionState::MOVING},
                                                                                            {GridLabel::EQUIPMENT_VEHICLE, MotionState::MOVING},
                                                                                            {GridLabel::PEDESTRIAN, MotionState::MOVING},
                                                                                            {GridLabel::CYCLIST, MotionState::MOVING},
                                                                                            {GridLabel::CONE, MotionState::STOPPED},
                                                                                            {GridLabel::RTG_QC, MotionState::MOVING},
                                                                                            {GridLabel::OCCUPIED, MotionState::STOPPED},
                                                                                            {GridLabel::UNOCCUPIED, MotionState::STOPPED},
                                                                                            {GridLabel::LOCK_BOX, MotionState::STOPPED},     // -- 锁钮箱
                                                                                            {GridLabel::LOCK_STATION, MotionState::STOPPED}, // -- 锁钮站
                                                                                            {GridLabel::LOCK_FRAME, MotionState::STOPPED},   // -- 锁钮架，集装箱架子，锁钮箱架子
                                                                                            {GridLabel::AIV, MotionState::MOVING},
                                                                                            {GridLabel::PUDDLE, MotionState::STOPPED},
                                                                                            {GridLabel::POTHOLE, MotionState::STOPPED},
                                                                                            {GridLabel::GRASS, MotionState::STOPPED},
                                                                                            {GridLabel::TREE_BRANCH, MotionState::STOPPED},
                                                                                            {GridLabel::OTHER, MotionState::MOVING}};
} // namespace fs

/* type
  id   英文字段         中文字段
  0   freespace       可通行区域
  1   vehicle         车
  2   pedestrian      行人
  3   road_edge       普通路沿
  4   barricade       带高度的路沿
  5   container       集装箱
  6   RTG             吊车
  7   cover_plate     舱盖板
  8   lock_box        锁钮箱
  9   lock_station    锁钮站
  10  container_shelf 集装箱架子，锁钮箱架子
  11  hanging_shelf   吊架
  12  black_roadstock 路中间的黑色障碍物
  13  other           其他
*/

/*
 *  PerceptionGrid nType:
    0 unknow OCCUPIED  未知的占据物
    3 pedestrain 行人
    4 bike  自行车
    5 car 小车
    6 truck  卡车
    10 RTG 轮胎吊
    15 QC 桥吊
    16 lock_box  锁扭箱
    18 CONES  锥桶
    21 lock_frame 锁扭架
 */

#endif //DEFS_H_
