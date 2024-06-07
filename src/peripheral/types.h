#ifndef TYPES_H_
#define TYPES_H_

#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "switcher.h"
#if FS_CHECK(CFG_ROS2)
#include <perception_freespace.pb.h>
#else
#include <lcm/PerceptionFreeSpace.hpp>
#endif

namespace fs
{
  using FSVec2f = Eigen::Vector2f;
  using FSVec3f = Eigen::Vector3f;
  using FSVec4f = Eigen::Vector4f;
  using FSVec2  = Eigen::Vector2i;
  using FSVec3  = Eigen::Vector3i;

#define DERIVE_FS_MAT_ALL_COLS(row)                    \
  typedef Eigen::Matrix<float, row, 2> FSMat##row##x2; \
  typedef Eigen::Matrix<float, row, 3> FSMat##row##x3; \
  typedef Eigen::Matrix<float, row, 4> FSMat##row##x4; \
  typedef Eigen::Matrix<float, row, 5> FSMat##row##x5; \
  typedef Eigen::Matrix<float, row, 6> FSMat##row##x6;
  DERIVE_FS_MAT_ALL_COLS(1)
  DERIVE_FS_MAT_ALL_COLS(2)
  DERIVE_FS_MAT_ALL_COLS(3)
  DERIVE_FS_MAT_ALL_COLS(4)
  DERIVE_FS_MAT_ALL_COLS(5)
  DERIVE_FS_MAT_ALL_COLS(6)
#undef DERIVE_FS_MAT_ALL_COLS

#define DERIVE_FS_MAT_ALL_COLS(row)                      \
  typedef Eigen::Matrix<double, row, 2> FSMat##row##x2d; \
  typedef Eigen::Matrix<double, row, 3> FSMat##row##x3d; \
  typedef Eigen::Matrix<double, row, 4> FSMat##row##x4d; \
  typedef Eigen::Matrix<double, row, 5> FSMat##row##x5d; \
  typedef Eigen::Matrix<double, row, 6> FSMat##row##x6d;
  DERIVE_FS_MAT_ALL_COLS(1)
  DERIVE_FS_MAT_ALL_COLS(2)
  DERIVE_FS_MAT_ALL_COLS(3)
  DERIVE_FS_MAT_ALL_COLS(4)
  DERIVE_FS_MAT_ALL_COLS(5)
  DERIVE_FS_MAT_ALL_COLS(6)
#undef DERIVE_FS_MAT_ALL_COLS

#define T2R(T)      (T.topLeftCorner<3, 3>())
#define T2R_inv(T)  (T.topLeftCorner<3, 3>().transpose())
#define T2t(T)      (T.topRightCorner<3, 1>())
#define HOMO3(p)    (FSVec3f(p.x(), p.y(), 1.0))
#define HOMO4(p)    (FSVec4f(p.x(), p.y(), p.z(), 1.0))
#define CV_HOMO3(p) (cv::Mat_<float><3, 1> << p.x, p.y, 1.0)
#define CV_HOMO4(p) (cv::Mat_<float><3, 1> << p.x, p.y, p.z, 1.0)

  enum FSResult : int
  {
    FS_ERROR = 0,
    FS_OK
  };
  struct LUTPos
  {
    int row = 0;
    int col = 0;
    LUTPos() {}
    LUTPos(int _row, int _col):
      row(_row),
      col(_col) {}
  };

  enum class GridLabel : int8_t
  {
    // clang-format off
    UNOCCUPIED = 0   ,
    OCCUPIED         ,
    UNKNOWN          ,
    VEHICLE          ,
    EQUIPMENT_VEHICLE,
    PEDESTRIAN       ,
    CYCLIST          ,
    CONE             ,
    RTG_QC           ,
    LOCK_BOX         ,
    LOCK_FRAME       ,
    LOCK_STATION     ,
    AIV              ,
    PUDDLE           , // 水坑
    POTHOLE          , // 道路坑洼
    GRASS            , // 草
    TREE_BRANCH      ,
    OTHER              // -- 其他未知障碍物

    // clang-format on

  };

  struct pair_hash
  {
    template<class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const
    {
      auto h1 = std::hash<T1>{}(p.first);
      auto h2 = std::hash<T2>{}(p.second);

      return h1 ^ h2;
    }
  };

  enum class SubLabel : int
  {
    VEHICLE = 0,
    VRU,
    PEDESTRIAN,
    BARRICADE,
    EQUIPMENT
  };

  enum class Zone : int
  {
    SAFETY = 0,
    CAUTION,
    IGNORANCE
  };

  enum class MotionState : int
  {
    STOPPED = 0,
    MOVING
  };
  enum class CameraFSType
  {
    INVALID   = 0, // -- 无效（截断，雨刷）
    CURB      = 1, // -- 路沿
    VPR       = 2, // -- 人，车，骑行者
    CONE      = 3, // -- 锥桶
    BARRICADE = 4, // -- 栅栏
    OTHER     = 5, // -- 其他
    WIPPER    = 6, // -- 雨刮
    DARK      = 7, // -- 黑暗
    RAIN_FOG  = 8, // -- 雨雾
    DUST      = 9, // -- 灰尘
    UNINIT    = 10
  };

  struct CamFSPoint
  {
    int          y    = 0;
    CameraFSType type = CameraFSType::INVALID; // -- 提供了初始值，默认构造函数就用初始值
    CamFSPoint()      = default;
    CamFSPoint(int _y, CameraFSType _type):
      y(_y),
      type(_type) {}
    CamFSPoint& operator=(const CamFSPoint& src) = default;
  };
  struct CamFS
  {
    std::unique_ptr<std::vector<fs::CamFSPoint>> points_ptr  = nullptr;
    int64_t                                      timestamp   = 0;
    FSMat4x4                                     T_cam2lidar = FSMat4x4::Identity();
  };

  enum class SensorId : uint8_t
  {
    CAMERA_FW = 0,
    CAMERA_FN,
    CAMERA_FL,
    CAMERA_FR,
    CAMERA_RW,
    CAMERA_RN,
    CAMERA_RL,
    CAMERA_RR,
    CAMERA_SFW, // -- surround front wide camera
    CAMERA_SFL,
    CAMERA_SFR,
    CAMERA_SRL, // -- surround rear wide camera
    CAMERA_SRR,
    CAMERA_SRW,
    MAX_SENSOR_NUM,
    CAMERA_BEV,
    CAMERA_TRACKED,
    LIDAR_LEFT,
    LIDAR_RIGHT,
    LIDAR,
    RADAR_FRONT,
    RADAR_FRONT_CORNER,
    RADAR_REAR_CORNER
  };

  enum class CySensorId : uint8_t
  {
    CAMERA_CFW = 0,
    CAMERA_CFL,
    CAMERA_CFR,
    CAMERA_CRL,
    CAMERA_CRR,
    CAMERA_CRW,
    MAX_CYSENSOR_NUM
  };

  enum class CameraModel : uint8_t
  {
    PINHOLE,
    FISHEYE
  };

  struct RoadPoint
  {
    int   x;
    int   y;
    float top;
    RoadPoint() = default;
    RoadPoint(int _x, int _y, float _height):
      x(_x),
      y(_y),
      top(_height){};
  };

  struct RoadModelFS
  {
    int64_t                                                   timestamp;
    std::unordered_map<std::pair<int, int>, float, pair_hash> points;
  };
  typedef std::shared_ptr<RoadModelFS> RoadModelFSPtr;

  struct FSPoint
  {
    float       x;
    float       y;
    float       top;
    float       bottom;
    float       ground;
    GridLabel   lidar_label;
    GridLabel   camera_label;
    bool        is_noise;
    MotionState motion_state;
    int         area;
    FSPoint() = default;
    FSPoint(float _x, float _y, float _top, float _bottom, GridLabel _label, MotionState _motion_state):
      x(_x),
      y(_y),
      top(_top),
      bottom(_bottom),
      lidar_label(_label),
      camera_label(GridLabel::OCCUPIED),
      is_noise(false),
      motion_state(_motion_state),
      ground(INT_MIN),
      area(-1){};
    FSPoint(float _x, float _y, float _top, float _bottom, GridLabel _label, MotionState _motion_state, int _area, float _ground):
      x(_x),
      y(_y),
      top(_top),
      bottom(_bottom),
      lidar_label(_label),
      camera_label(GridLabel::OCCUPIED),
      is_noise(false),
      motion_state(_motion_state),
      ground(_ground),
      area(_area){};
  };

  struct LidarFS
  {
    int64_t              timestamp;
    std::vector<FSPoint> points;
  };
  typedef std::shared_ptr<LidarFS>                 LidarFSPtr;
  typedef std::unordered_map<SensorId, LidarFSPtr> LidarArea;

  struct GateInfo
  {
    int64_t timestamp;
    int64_t request_id;
    uint8_t gate_state;
    double  gate_cx;
    double  gate_cy;
    double  gate_cz;
    float   stop_distance;
  };
  typedef std::shared_ptr<GateInfo> GateInfoPtr;
  typedef std::vector<GateInfoPtr>  GatesInfoVec;

  struct Rect
  {
    uint8_t gate_state = 2;
    float   min_x      = 0.;
    float   min_y      = 0.;
    float   max_x      = 0.;
    float   max_y      = 0.;
    Rect()             = default;
    Rect(uint8_t _gate_state, float _min_x, float _min_y, float _max_x, float _max_y):
      gate_state(_gate_state),
      min_x(_min_x),
      min_y(_min_y),
      max_x(_max_x),
      max_y(_max_y) {}
  };

  struct EgoMotion
  {
    int64_t timestamp = 0;
    bool    is_valid  = true;
    FSVec3f translation{};
    float   yaw    = 0.f;
    float   pitch  = 0.f;
    float   roll   = 0.f;
    double  init_x = 0.;
    double  init_y = 0.;
    double  init_z = 0.;
  };

  struct ObjectState
  {
    float       x            = 0.f;
    float       y            = 0.f;
    float       abs_vx       = 0.f;
    float       abs_vy       = 0.f;
    float       real_vx      = 0.f;
    float       real_vy      = 0.f;
    float       yaw          = 0.f;
    float       length       = 0.f;
    float       width        = 0.f;
    int16_t     id           = 0;
    GridLabel   type         = GridLabel::UNKNOWN;
    MotionState motion_state = MotionState::STOPPED;
    ObjectState() {}
    ObjectState(float _x, float _y, float _abs_vx, float _abs_vy, float _real_vx, float _real_vy, float _yaw, float _length, float _width, int16_t _id, GridLabel _type, MotionState _motion_state):
      x(_x),
      y(_y),
      abs_vx(_abs_vx),
      abs_vy(_abs_vy),
      real_vx(_real_vx),
      real_vy(_real_vy),
      yaw(_yaw),
      length(_length),
      width(_width),
      id(_id),
      type(_type),
      motion_state(_motion_state) {}
  };
  struct ObjectStates
  {
    int64_t                  timestamp;
    std::vector<ObjectState> objects;
  };
  typedef std::unique_ptr<ObjectStates> ObjectStatesPtr;

  struct ROI
  {
    int min_x = 0.;
    int min_y = 0.;
    int max_x = 0.;
    int max_y = 0.;
    ROI()     = default;
    ROI(int _min_x, int _min_y, int _max_x, int _max_y):
      min_x(_min_x),
      min_y(_min_y),
      max_x(_max_x),
      max_y(_max_y) {}
  };

  enum CuboidPointIndex
  {
    BOTTOM_REAR_LEFT = 0,
    BOTTOM_FRONT_LEFT,
    BOTTOM_FRONT_RIGHT,
    BOTTOM_REAR_RIGHT,
    CUBOID_POINT_MAX_NUM
  };
  enum LocFlag
  {
    INVALID,
    LOW_VALID,
    VALID
  };

  struct CloudPose
  {
    int64_t                             timestamp;
    EgoMotion                           pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    using Ptr = std::shared_ptr<CloudPose>;
  };

  enum class ProjectLocation : uint8_t
  {
    UNKNOWN = 0,
    WS,
    DX,
    QZ,
    TC,
    LJ,
    QD
  };
  enum class LinesPos : uint8_t
  {
    LEFT  = 0,
    RIGHT = 1,
    INVALID
  };
  struct LineCoeff
  {
    int64_t            timeastamp = 0;
    LinesPos           lines_pos  = LinesPos::INVALID;
    std::vector<float> coeff{};
    EgoMotion          emo;
    LineCoeff() {}
  };

  struct BevPoint
  {
    float x;
    float y;
    float score;
    int   label;
    BevPoint(float _x, float _y, float _score, int _label):
      x(_x),
      y(_y),
      score(_score),
      label(_label) {}
  };
  struct BevSeg
  {
    int64_t               timestamp = 0;
    std::vector<BevPoint> points;
    float                 width_m;
    float                 height_m;
  };
  typedef std::unique_ptr<BevSeg> BevSegPtr;

} // namespace fs

#endif //TYPES_H_
