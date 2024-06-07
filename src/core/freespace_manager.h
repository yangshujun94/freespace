#ifndef FREESPACE_MANAGER_H_
#define FREESPACE_MANAGER_H_

#include <common/log.h>

#include "peripheral/switcher.h"
#include "peripheral/types.h"
#include "peripheral/utils.h"
#include "peripheral/vehicle.h"
#include "core/expand/expand.h"
#include "grid_map.h"

#if FS_CHECK(CFG_VIS_ENABLE)
#include "debug/vis.h"
#endif // -- CFG_VIS_ENABLE

#if FS_CHECK(CFG_ROS2)
#include <perception_freespace.pb.h>
#include <perception_camera.pb.h>
#else
#include <lcm/IMAGE.hpp>
#include <lcm/LOGITECH_IMAGE.hpp>
#include <lcm/TRANSFORM.hpp>
#include <lcm/LocFusion.hpp>
#include <lcm/CameraFreespace.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcm/Lidar_FreeSpace_v2.hpp>
#include <lcm/VCU_VEHILCE_INFO.hpp>
#include <lcm/NORAML_CAMERA_LANE_CONTROL.hpp>
#include <DMAIV/include_lcm13/DmAivBatch.hpp>
#include <lcm/PerceptionFreeSpace.hpp>
#endif // -- CFG_ROS2

namespace fs
{
  class FreespaceManager
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(FreespaceManager);

    FreespaceManager();
    void resetFusionMap();
    void resetTptsMap();
    void assignFusionMapVotInfo(const ObjectStates *vot_object_refine_ptr);
    void assignTptsMapVotInfo(const ObjectStates *vot_object_refine_ptr);
    void setFusionMapResetFlag(bool flag)
    {
      is_fusion_map_reset = flag;
    }
    bool getFusionMapResetFlag()
    {
      return is_fusion_map_reset;
    }

#if FS_CHECK(CFG_ROS2)
    std::unique_ptr<uto::proto::PerceptionFreespace> tptsLidarFs(const std::unique_ptr<uto::proto::PerceptionFreespace> &lidar_fs_ptr, bool is_reverse = false /*placeholder*/);

    void fuseFreespace(const int64_t                                     fusion_timestamp,
                       const LidarFS                                    *fs_lidar_ptr,
                       const ObjectStates                               *vot_object_refine_ptr,
                       std::unique_ptr<uto::proto::PerceptionFreespace> &fusion_fs_output_ptr,
                       const EgoMotion                                  *emo_align_pos_fusion,
                       const EgoMotion                                  *emo_pos_fusion,
                       std::unique_ptr<uto::proto::PerceptionFreespace> &raw_lidar_fs,
                       std::vector<std::vector<fs::LineCoeff>>          &coeff_lines,
                       bool                                              is_lock_station,
                       bool                                              is_electric_fence,
                       bool                                              is_reverse = false /*placeholder*/);

    void tptsFreespace(const int64_t                                     fusion_timestamp,
                       const ObjectStates                               *vot_object_refine_ptr,
                       std::unique_ptr<uto::proto::PerceptionFreespace> &fusion_fs_output_ptr,
                       std::unique_ptr<uto::proto::PerceptionFreespace> &raw_lidar_fs,
                       bool                                              is_reverse = false /*placeholder*/);

    void getFreespace(std::unique_ptr<uto::proto::PerceptionFreespace> &lidar_fs_ptr);
    void downSample(std::unique_ptr<uto::proto::PerceptionFreespace> &fusion_fs_ptr);
    void removeOfGrass(uto::proto::PerceptionFreespace *lidar_fs, const std::vector<std::vector<fs::LineCoeff>> &cameras_lines, const EgoMotion *emo_fusion);
    void removeOfLockStation(uto::proto::PerceptionFreespace *lidar_fs, const std::vector<std::vector<fs::LineCoeff>> &coeff_lines, const EgoMotion *emo_fusion);
    void ego2Pub(uto::proto::PerceptionFreespace *fusion_fs_ptr, const EgoMotion *emo_pos_fusion);
#else
    std::unique_ptr<PerceptionFreeSpace> tptsLidarFs(const Lidar_FreeSpace_v2* lidar_fs_ptr, bool is_reverse);
    void                                 fuseFreespace(const int64_t                         fusion_timestamp,
                                                       const BevSeg*                         bev_seg_refine_ptr,
                                                       const LidarFS*                        fs_lidar_ptr,
                                                       const ObjectStates*                   vot_object_refine_ptr,
                                                       std::unique_ptr<PerceptionFreeSpace>& fusion_fs_output_ptr,
                                                       const EgoMotion*                      emo_pos_latest,
                                                       const EgoMotion*                      emo_pos_fusion,
                                                       float                                 resolution,
                                                       bool                                  is_reverse,
                                                       std::vector<cv::Point2f>&             polygon_lane,
                                                       bool                                  in_ele_fence,
                                                       bool                                  is_lock_station,
                                                       const Lidar_FreeSpace_v2*             fs_lidar_lcm_ptr);

    void tptsFreespace(const int64_t                         fusion_timestamp,
                       const ObjectStates*                   vot_object_refine_ptr,
                       std::unique_ptr<PerceptionFreeSpace>& fusion_fs_output_ptr,
                       float                                 resolution,
                       bool                                  is_reverse,
                       const Lidar_FreeSpace_v2*             raw_lidar_fs);

    std::unique_ptr<PerceptionFreeSpace> removeOfGrass(PerceptionFreeSpace* lidar_fs, std::vector<cv::Point2f>& polygon, const BevSeg* bev_seg_ptr, bool is_reverse);
    std::unique_ptr<PerceptionFreeSpace> removeOfLockStation(PerceptionFreeSpace* lidar_fs, const std::vector<cv::Point2f>& polygon, const BevSeg* bev_seg_ptr, bool is_reverse);

    void getFreespace(const EgoMotion* ego_motion_latest, PerceptionFreeSpace* lidar_fs, const EgoMotion* emo_pos_fusion, float resolution, bool is_reverse);

    void downSample(std::unique_ptr<PerceptionFreeSpace>& fusion_fs_ptr);
#endif // -- CFG_ROS2

  private:
    bool        getWordPolygon(const ObjectState &ob, std::vector<FSVec2f> &box_polygon_wrd, float &poly_min_x, float &poly_max_x, float &poly_min_y, float &poly_max_y);
    inline void updateLabelMotionProb(Grid &grid);
    bool        pointBetweenLanes(float x, float y, const std::vector<fs::LineCoeff> &camera_lines, const EgoMotion *emo_fusion);

    template<typename T1>
    void drawFreeSpace(const T1 &lidar_fs)
    {
#if FS_CHECK(CFG_VIS_ENABLE)
#if FS_CHECK(CFG_ROS2)
#if FS_CHECK(!CFG_DEBUG_PERCEPTION_FREESPACES)
      Vis::drawSelfReceivePointCloud(lidar_fs);
#endif
#else
      Vis::drawSelfReceivePointCloud(lidar_fs);
#endif // -- CFG_ROS2
#endif // -- CFG_VIS_ENABLE
    }

#if FS_CHECK(CFG_USE_MCAP_RAW)
#if FS_CHECK(CFG_ROS2)
#if FS_CHECK(CFG_PUB_ENU_FS)
    void drawBag(const std::unique_ptr<uto::proto::PerceptionFreespace> &message, const EgoMotion *ego_motion);
#else
    void drawBag(const std::unique_ptr<uto::proto::PerceptionFreespace> &message);
#endif // -- CFG_PUB_ENU_FS
#else
    void drawBag(const PerceptionFreeSpace &message, const EgoMotion *ego_motion);
#endif
#endif // -- CFG_USE_MCAP_RAW

    void drawOther(const ObjectStates *vot_object_refine_ptr, const BevSeg *fsBev = nullptr);
#if FS_CHECK(CFG_PUB_CONTOUR)
    int8_t raw_map_[FS_ROW_OUT][FS_COL_OUT];
#endif // -- CFG_PUB_CONTOUR
    GridMap fusion_map_{};
    Grid    tpts_map_[FS_ROW_OUT][FS_COL_OUT];
    bool    is_fusion_map_reset = false;
    bool    is_tpts_map_reset   = false;

    fs::RingBuffer<std::vector<FSVec3f>, 64> lidar_grids_buffer_{};
    std::int64_t                             fusion_timestamp_ = 0;
    float                                    dt_               = 0.f;
    int                                      logit_count_      = 0;
    std::string                              save_path_        = "/home/utopilot/workspace/output";

    cv::Mat mask_cv_ = cv::Mat::zeros(360, 360, CV_8U);

#if !FS_CHECK(CFG_ROS2)
    FSMat4x4 T_180_;
#endif // -- !CFG_ROS2
  };
} // namespace fs

#endif //FREESPACE_MANAGER_H_
