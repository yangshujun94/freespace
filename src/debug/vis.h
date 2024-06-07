#ifndef VIS_H_
#define VIS_H_

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <map>

#define private public

#include "vis_helper.h"
#include "core/grid_map.h"
#include "peripheral/defs.h"
#include "peripheral/switcher.h"
#include "peripheral/camera/camera_manager.h"
#include "peripheral/vehicle.h"
#include "core/expand/expand.h"

#if FS_CHECK(CFG_ROS2)
#include <perception_freespace.pb.h>
#else
#include <lcm/Lidar_FreeSpace_v2.hpp>
#include <lcm/CameraFreespace.hpp>
#include <lcm/PerceptionFreeSpace.hpp>
#include <DMAIV/include_lcm13/DmAivBatch.hpp>
#include "lcm/LOGITECH_IMAGE.hpp"
#endif

#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#endif

namespace fs
{
  static const std::map<const GridLabel, const cv::Scalar> BEV_FS_COLOR = {
    {GridLabel::UNOCCUPIED, COLOR_BLACK},
    {GridLabel::VEHICLE, COLOR_B},
    {GridLabel::PEDESTRIAN, COLOR_G},
    {GridLabel::RTG_QC, COLOR_P},
    {GridLabel::LOCK_BOX, COLOR_GOLD},
    {GridLabel::OTHER, COLOR_GREY_BLUE},
    {GridLabel::CONE, COLOR_W},
  };

  class GridMap;

  class Vis
  {
  public:
    static const cv::Mat &getBEVView()
    {
      return m_BEV_View;
    }
    static const cv::Mat &getView(SensorId camera_id)
    {
      return map_img_vis[(uint8_t)camera_id];
    }
    static const cv::Mat &getCylinderView(CySensorId camera_id)
    {
      return cy_map_img_vis[(uint8_t)camera_id];
    }
    static const cv::Mat &getRemapView()
    {
      return m_Remap_View;
    }
    //zylinder example
    static const cv::Mat &getZYlinderView()
    {
      return m_Test_Zylinder_View;
    }
#if !FS_CHECK(CFG_ROS2)
    static const sensor_msgs::msg::PointCloud &getegoBboxPointCloud()
    {
      return m_egoBboxPointCloud;
    }
#endif
    static cv::Scalar getFreespaceColor(CameraFSType id)
    {
      return CAMERA_FS_COLOR.at(id);
    }

    static const sensor_msgs::msg::PointCloud &getGridPointCloud()
    {
      return m_gridPointCloud;
    }

    static const visualization_msgs::msg::MarkerArray &getMarkerArray()
    {
      return m_markers;
    }

    static const visualization_msgs::msg::MarkerArray &getBlindArray()
    {
      return m_blind_markers;
    }

    static const sensor_msgs::msg::PointCloud &getLidarPointCloud()
    {
      return m_lidarPointCloud;
    }

    static const sensor_msgs::msg::PointCloud &getRoadPointCloud()
    {
      return m_roadPointCloud;
    }

    static const sensor_msgs::msg::PointCloud &getBevPointCloud()
    {
      return m_bevPointCloud;
    }

#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
    static const sensor_msgs::msg::PointCloud2 &getLidarPointCloud2()
    {
      return m_lidarPointCloud2;
    }

    static const sensor_msgs::msg::PointCloud2 &getRoadPointCloud2()
    {
      return m_roadPointCloud2;
    }
#endif
    static const sensor_msgs::msg::PointCloud &getBlindPointCloud()
    {
      return m_blindPointCloud;
    }

    static const sensor_msgs::msg::PointCloud &getPolygonPointCloud()
    {
      return m_PolygonPointCloud;
    }

    static const sensor_msgs::msg::PointCloud &getLidarPointCloudHistory()
    {
      return m_lidarPointCloudHistory;
    }

    static const sensor_msgs::msg::PointCloud &getTptsBboxPointCloud()
    {
      return m_tptsBboxPointCloud;
    }

    static const sensor_msgs::msg::PointCloud &getOutBboxPointCloud()
    {
      return m_outBboxPointCloud;
    }

    static const sensor_msgs::msg::PointCloud &getOutputPointCloud()
    {
      return m_outputPointCloud;
    }

    static const sensor_msgs::msg::PointCloud &getColorTablePointCloud()
    {
      return m_colorTablePointCloud;
    }

    static const std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud> &getGridPointCloudSplit()
    {
      return map_cld_built_in_;
    }

    static const std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud> &getSelfReceivePointCloud()
    {
      return map_cld_self_receive_;
    }

#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
    static const std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud2> &getGridPointCloudSplit2()
    {
      return map_cld_built_in2_;
    }

    static const std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud2> &getSelfReceivePointCloud2()
    {
      return map_cld_self_receive2_;
    }
#endif

    static void drawRemap();

    template<typename T>
    static std::map<T, fs::BoxString> getMapInfo(const std::map<const T, const std::string> &map, int img_h, int box_h, int box_w, int start_w)
    {
      const int h_step = (img_h - box_h * map.size()) / (2 * map.size());
      int       half   = map.size() / 2;

      std::map<T, fs::BoxString> res;
      int                        start_h = 0;
      for(const auto &c : map)
      {
        start_h += h_step;

        BoxString tmp;
        tmp.rect.x      = start_w;
        tmp.rect.y      = start_h;
        tmp.rect.width  = box_w;
        tmp.rect.height = box_h;
        tmp.str         = c.second;
        res[c.first]    = tmp;

        start_h += box_h;
        start_h += h_step;
      }
      return res;
    }

#if FS_CHECK(CFG_ROS2)

    static void drawSelfReceivePointCloud(const uto::proto::PerceptionFreespace &lidar_fs, const EgoMotion *ego_motion);
    static void drawSelfReceivePointCloud(const uto::proto::PerceptionFreespace &lidar_fs);
    //    static void prepare(const int64_t                                  timestamp,
    //                        const sensor_msgs::msg::CompressedImage       *img_fw_ptr,
    //                        const sensor_msgs::msg::CompressedImage       *img_fn_ptr,
    //                        const sensor_msgs::msg::CompressedImage       *img_fl_ptr,
    //                        const sensor_msgs::msg::CompressedImage       *img_fr_ptr,
    //                        const sensor_msgs::msg::CompressedImage       *img_rw_ptr,
    //                        const sensor_msgs::msg::CompressedImage       *img_rn_ptr,
    //                        const sensor_msgs::msg::CompressedImage       *img_rl_ptr,
    //                        const sensor_msgs::msg::CompressedImage       *img_rr_ptr,
    //                        const ObjectStates                            *vot_objects_ptr,
    //                        const LidarFS                                 *lidar_fs_ptr,
    //                        const std::vector<CamFS>                      *camera_freespace,
    //                        const FSMat4x4                                &Tcl,
    //                        const std::vector<std::vector<fs::LineCoeff>> &coeff_lanes);
    static void prepare(const int64_t                                                                                      timestamp,
                        std::array<std::unique_ptr<sensor_msgs::msg::CompressedImage>, (uint8_t)SensorId::MAX_SENSOR_NUM> &map_img,
                        const std::array<CamFS, (uint8_t)SensorId::MAX_SENSOR_NUM>                                        &map_fs,
                        const ObjectStates                                                                                *vot_objects_ptr,
                        const LidarFS                                                                                     *lidar_fs_ptr,
                        const std::vector<std::vector<fs::LineCoeff>>                                                     &coeff_lanes);

#else
    static void drawEgoBboxRviz();
    static void drawSelfReceivePointCloud(const PerceptionFreeSpace &lidar_fs);
    static void drawSelfReceivePointCloud(const Lidar_FreeSpace_v2 &lidar_fs);
    static void parseDmAivImage(const DMAIV::DmAivBatch &img_bev_ptr, const fs::SensorId sensorId, cv::Mat &dst);
    static void prepare(const int64_t                   timestamp,
                        const fs::LidarFS              *lidar_ptr,
                        const std::vector<cv::Point2f> &polygonm_line,
                        const DMAIV::DmAivBatch        *img_bev_ptr,
                        const BevSeg                   *camera_fs_bev_ptr,
                        const ObjectStates             *vot_object_raw_ptr,
                        const ObjectStates             *vot_object_ptr,
                        const LOGITECH_IMAGE           *image_fw_Ptr,
                        const LOGITECH_IMAGE           *image_fn_Ptr,
                        const LOGITECH_IMAGE           *image_rw_Ptr,
                        const LOGITECH_IMAGE           *image_rn_Ptr,
                        const LOGITECH_IMAGE           *image_fl_Ptr,
                        const LOGITECH_IMAGE           *image_fr_Ptr,
                        const LOGITECH_IMAGE           *image_rl_Ptr,
                        const LOGITECH_IMAGE           *image_rr_Ptr);
#endif
    static void        fishImageToCylindrical(cv::Mat &fisheye_image, cv::Mat &cylindrical_image, const Camera *const camera, const std::string &id, bool saveMap);
    static cv::Point2f unprojectFisheye(const cv::Point2f &point, const Camera *const camera, cv::Mat &cy_img);
    static void        getRemoveNoiseFreespace3(cv::Mat &img, cv::Mat &cy_img, const LidarFS *lidar_fs_ptr, const int &idx, const std::vector<CamFSPoint> *camera_freespace, const Camera *const camera, const FSMat4x4 &Tcl);

    static void getRemoveNoiseFreespace3(cv::Mat &img, const LidarFS *lidar_fs_ptr, const int &idx, const std::vector<CamFSPoint> *camera_freespace, const Camera *const camera, const FSMat4x4 &Tcl);

    static void drawGridMap(const GridMap &gridMap);

    static void drawGridMapsplit(const fs::GridMap &gridMap);

    static void drawLidarPointCloud(const LidarFS *fsLidar);

#if FS_CHECK(CFG_USE_ROAD_MODEL)
    static void drawRoadModelPointCloud(const RoadModelFS *fsLidar);
#endif

    static void emplaceBack(const GridLabel grid_label, const geometry_msgs::msg::Point32 &point, const float logit_motion, std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud> &map_cld);

    static void drawBevFs(const BevSeg *fsBev);
#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
    static void emplaceBack2(std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud> &map_cld, std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud2> &map_cld2);
#endif

    static void drawRvizColorTable();

    static void drawPolygonPointCloud(const int polygon[GRID_MAP_SIZE][GRID_MAP_SIZE]);

    static void drawLidarPointCloudHistory(const std::vector<FSVec3f> &pointsWrd);

    static void drawPolygonLane(const std::vector<std::vector<cv::Point2f>> &polygon_line);

    static void drawVotObject(const ObjectStates *vot_object_ptr, std::string type, const float dt);

    static void drawGates(const std::vector<fs::Rect> gates_bbox);

    static void drawZone(const std::vector<std::vector<fs::FSVec3f>> &gates_bbox);

    static void drawObjectBoxRviz(const ObjectState &ob, std::string type, const float dt);

    static void drawTptsBboxRviz();

    static void drawOutBboxRviz(float resolution = 0.1);

    static void drawBlindMakers(const std::vector<cv::Point2f> &l, const std::vector<cv::Point2f> &r);

    static void drawBlindPointCloud(bool blind[GRID_MAP_SIZE][GRID_MAP_SIZE]);

    static void drawLines(const std::vector<std::vector<fs::LineCoeff>> &lines);

#if FS_CHECK(CFG_ROS2)
    static void prepareRawImage(const sensor_msgs::msg::CompressedImage &src, const SensorId sensorId, cv::Mat &dst);
#else
    static void prepareRawImage(const LOGITECH_IMAGE &src, const SensorId sensorId, cv::Mat &dst);
#endif
    template<class T>
    static void decodeImg(const std::array<std::unique_ptr<sensor_msgs::msg::CompressedImage>, (uint8_t)SensorId::MAX_SENSOR_NUM> &map_img, T sensor, cv::Mat &img_out)
    {
      if(nullptr != map_img[(uint8_t)sensor])
      {
        const Camera *const camera = CameraManager::instance().getCamera(sensor).get();
        cv::Mat             img    = cv::imdecode(map_img[(uint8_t)sensor]->data, cv::IMREAD_COLOR);
        cv::resize(img, img_out, cv::Size{VIS_IMAGE_VIEW_SCALING * camera->getImageWidth(), VIS_IMAGE_VIEW_SCALING * camera->getImageHeight()});
      }
    }

  private:
    static void drawEgo();

    static void drawTrajectory();

    static void clearMarkers();

    static void drawTimestamp(const int64_t timestamp, cv::Mat &img);

#if !FS_CHECK(CFG_ROS2)
    static cv::Mat m_FW_View;
    static cv::Mat m_FN_View;
    static cv::Mat m_RW_View;
    static cv::Mat m_RN_View;
    static cv::Mat m_FL_View;
    static cv::Mat m_FR_View;
    static cv::Mat m_RL_View;
    static cv::Mat m_RR_View;
#endif

#if !FS_CHECK(CFG_ROS2)
    static sensor_msgs::msg::PointCloud m_egoBboxPointCloud;
#endif
    static cv::Mat m_BEV_View;
    static cv::Mat m_Remap_View;
    //zylinder example
    static cv::Mat                                                m_Test_Zylinder_View;
    static std::array<cv::Mat, (uint8_t)SensorId::MAX_SENSOR_NUM> map_img_vis;
    static std::array<cv::Mat, 6>                                 cy_map_img_vis;
    static visualization_msgs::msg::MarkerArray                   m_markers;
    static visualization_msgs::msg::MarkerArray                   m_blind_markers;
    static sensor_msgs::msg::PointCloud                           m_gridPointCloud;
    static sensor_msgs::msg::PointCloud                           m_outputPointCloud;
    static sensor_msgs::msg::PointCloud                           m_lidarPointCloud;
    static sensor_msgs::msg::PointCloud                           m_roadPointCloud;
    static sensor_msgs::msg::PointCloud                           m_bevPointCloud;
#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
    static sensor_msgs::msg::PointCloud2 m_lidarPointCloud2;
    static sensor_msgs::msg::PointCloud2 m_roadPointCloud2;
#endif
    static sensor_msgs::msg::PointCloud m_baglidarPointCloud;
    static sensor_msgs::msg::PointCloud m_blindPointCloud;
    static sensor_msgs::msg::PointCloud m_colorTablePointCloud;
    static sensor_msgs::msg::PointCloud m_PolygonPointCloud;
    static sensor_msgs::msg::PointCloud m_lidarPointCloudHistory;
    static sensor_msgs::msg::PointCloud m_tptsBboxPointCloud;
    static sensor_msgs::msg::PointCloud m_outBboxPointCloud;

    static sensor_msgs::msg::PointCloud cld_OTHER_;
    static sensor_msgs::msg::PointCloud cld_PEDESTRIAN_;      // -- 3 pedestrain 行人
    static sensor_msgs::msg::PointCloud cld_CYCLIST_;         // -- 4 bike  自行车
    static sensor_msgs::msg::PointCloud cld_VEHICLE_;         // -- 5 car 小车
    static sensor_msgs::msg::PointCloud cld_TRUCK_;           // -- 6 truck  卡车
    static sensor_msgs::msg::PointCloud cld_RTG_QC_;          // -- 10 RTG 轮胎吊
    static sensor_msgs::msg::PointCloud cld_QC_;              // -- 15 QC 桥吊
    static sensor_msgs::msg::PointCloud cld_LOCK_BOX_;        // -- 16 lock_box  锁扭箱
    static sensor_msgs::msg::PointCloud cld_CONE_;            // -- 18 CONES  锥桶
    static sensor_msgs::msg::PointCloud cld_TREE_BRANCH_;     // -- 13 aiv
    static sensor_msgs::msg::PointCloud cld_AIV_;             // -- 13 aiv
    static sensor_msgs::msg::PointCloud cld_CONTAINER_SHELF_; // -- 21 lock_frame 锁扭架

#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
    static sensor_msgs::msg::PointCloud2                                cld_OTHER2_;
    static sensor_msgs::msg::PointCloud2                                cld_PEDESTRIAN2_;      // -- 3 pedestrain 行人
    static sensor_msgs::msg::PointCloud2                                cld_CYCLIST2_;         // -- 4 bike  自行车
    static sensor_msgs::msg::PointCloud2                                cld_VEHICLE2_;         // -- 5 car 小车
    static sensor_msgs::msg::PointCloud2                                cld_TRUCK2_;           // -- 6 truck  卡车
    static sensor_msgs::msg::PointCloud2                                cld_RTG_QC2_;          // -- 10 RTG 轮胎吊
    static sensor_msgs::msg::PointCloud2                                cld_QC2_;              // -- 15 QC 桥吊
    static sensor_msgs::msg::PointCloud2                                cld_LOCK_BOX2_;        // -- 16 lock_box  锁扭箱
    static sensor_msgs::msg::PointCloud2                                cld_CONE2_;            // -- 18 CONES  锥桶
    static sensor_msgs::msg::PointCloud2                                cld_AIV2_;             // -- 13 aiv
    static sensor_msgs::msg::PointCloud2                                cld_TREE_BRANCH2_;     // -- 13 aiv
    static sensor_msgs::msg::PointCloud2                                cld_CONTAINER_SHELF2_; // -- 21 lock_frame 锁扭架
    static std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud2> map_cld_built_in2_;

    static sensor_msgs::msg::PointCloud2                                cld_self_receive_OTHER2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_PEDESTRIAN2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_CYCLIST2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_VEHICLE2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_TRUCK2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_RTG_QC2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_QC2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_LOCK_BOX2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_CONE2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_AIV2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_TREE_BRANCH2_;
    static sensor_msgs::msg::PointCloud2                                cld_self_receive_CONTAINER_SHELF2_;
    static std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud2> map_cld_self_receive2_;
#endif

    static sensor_msgs::msg::PointCloud cld_self_receive_OTHER_;
    static sensor_msgs::msg::PointCloud cld_self_receive_PEDESTRIAN_;
    static sensor_msgs::msg::PointCloud cld_self_receive_CYCLIST_;
    static sensor_msgs::msg::PointCloud cld_self_receive_VEHICLE_;
    static sensor_msgs::msg::PointCloud cld_self_receive_TRUCK_;
    static sensor_msgs::msg::PointCloud cld_self_receive_RTG_QC_;
    static sensor_msgs::msg::PointCloud cld_self_receive_QC_;
    static sensor_msgs::msg::PointCloud cld_self_receive_LOCK_BOX_;
    static sensor_msgs::msg::PointCloud cld_self_receive_CONE_;
    static sensor_msgs::msg::PointCloud cld_self_receive_TREE_BRANCH_;
    static sensor_msgs::msg::PointCloud cld_self_receive_AIV_;
    static sensor_msgs::msg::PointCloud cld_self_receive_CONTAINER_SHELF_;

    static std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud> map_cld_built_in_;
    static std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud> map_cld_self_receive_;

    // -- vis
  };
} // namespace fs

#endif //VIS_H_
