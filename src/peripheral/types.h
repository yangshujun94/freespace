#ifndef TYPES_H_
#define TYPES_H_

#include <Eigen/Dense>
#include <cfloat>
#include "switcher.h"

namespace fs
{
  using EVector2 = Eigen::Vector2f;
  using EVector3 = Eigen::Vector3f;
  using EMatrix2 = Eigen::Matrix2f;
  using EMatrix3 = Eigen::Matrix3f;

  struct EgoMotion
  {
    int64_t timestampUs = 0U;

    EVector3           translation{};
    Eigen::Quaternionf quaternion{};

    // ego vehicle pose in Ltm coordinate system
    Eigen::Vector3d    translationLtm{};
    Eigen::Quaternionf quaternionLtm{};
  };

  enum class SensorId : uint8_t
  {
    CAMERA_CENTER_FRONT = 0,
#if FS_CHECK(CFG_USE_FISHEYE_FS)
    FISHEYE_FCF,
    FISHEYE_FLL,
    FISHEYE_FRR,
    FISHEYE_BCB,
    FISHEYE_BLL,
    FISHEYE_BRR,
#endif
    LIDAR,
    MAX_SENSOR_NUM
  };

  enum class ObstacleClass : uint8_t
  {
    NOISE, ///< specific class only for lidar noise
    VEHICLE,
    VRU,
    STATIC_OBJECT,
    CRANE,
    MAX_NUM ///< invalid value from input signal, which should by completely ignored
  };

  enum BorderType : uint8_t
  {
    BORDER_TYPE_FREE     = 0,
    BORDER_TYPE_OCCUPIED = 1,
    BORDER_TYPE_CONTOUR  = 2,
    BORDER_TYPE_MAX_NUM
  };

  struct Grid
  {
    int      logit = 0;
    EVector2 point{};
    bool     locked      = false;
    bool     isBorder    = true;
    bool     isFreeInCam = false;
    float    top         = 0.0f;
    float    ground      = 0.0f;
    float    bottom      = 0.0f;

#if FS_CHECK(CFG_DEBUG_GRID)
    int      logitPrev  = 0;
    int      logitLidar = 0;
    uint32_t history    = 1;
    int      rowPrev    = 0;
    int      colPrev    = 0;
#endif
  };

  enum CameraClass : uint8_t
  {
    INVALID,
    STATIC_BORDER,
    VPR,
    NON_OBSTACLE,
    MAX_NUM
  };

  enum ShapePointIndex
  {
    RL = 0,
    FL,
    FR,
    RR,
    SHAPE_POINT_MAX_NUM
  };

  struct Rect
  {
    float minX = FLT_MAX;
    float maxX = -FLT_MAX;
    float minY = FLT_MAX;
    float maxY = -FLT_MAX;
  };

  enum class SafetyZone : uint8_t
  {
    DANGER,
    CAUTION,
    IGNORANCE,
    MAX_NUM
  };
} // namespace fs

#endif //TYPES_H_
