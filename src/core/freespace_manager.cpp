#include <locator_vehicle_pose.pb.h>
#include "freespace_manager.h"

fs::FreespaceManager::FreespaceManager()
{
#if !FS_CHECK(CFG_ROS2)
  T_180_ << -1, 0, 0, 0,
    0, -1, 0, 0,
    0, 0, 0, 1,
    0, 0, 0, 1;
#endif
  Grid tmp(GridLabel::OCCUPIED);
  for(int row = 0; row < FS_ROW_OUT; ++row)
  {
    for(int col = 0; col < FS_COL_OUT; ++col)
    {
      memcpy(&(tpts_map_[row][col]), &tmp, sizeof(tmp));
    }
  } // -- mod grid
  save_path_ = Utils::makeDirSimple("/home/utopilot/workspace/output/" + Utils::getCurrentTimeStamp(1));
}

bool fs::FreespaceManager::getWordPolygon(const ObjectState &ob, std::vector<FSVec2f> &box_polygon_wrd, float &poly_min_x, float &poly_max_x, float &poly_min_y, float &poly_max_y)
{
  std::string expand{};
  const auto &approx_polygon = Expand::instance().getPolygonByOval(ob, dt_, expand);

  // -- 1. get box corner in world
  for(auto &point : approx_polygon)
  {
    if(std::fabs(point.x) >= HALF_GRID_MAP_SIZE_M || std::fabs(point.y) >= HALF_GRID_MAP_SIZE_M)
    {
      box_polygon_wrd.clear();
      break;
    } // if one corner out map, jump this box
    auto p     = fusion_map_.pos2World(FSVec2f(point.x, point.y));
    poly_min_x = std::min(poly_min_x, p.x());
    poly_max_x = std::max(poly_max_x, p.x());
    poly_min_y = std::min(poly_min_y, p.y());
    poly_max_y = std::max(poly_max_y, p.y());
    box_polygon_wrd.emplace_back(p);
  } // -- word
  return box_polygon_wrd.empty();
}

void fs::FreespaceManager::updateLabelMotionProb(Grid &grid)
{
  grid.is_noise = false;
  if(grid.motion_state == MotionState::MOVING)
  {
    grid.motion_state = MotionState::STOPPED;
  } // -- switch motion state

  if((fusion_timestamp_ - grid.vot_timestamp) > 300 * TIME_SCALE_MS && grid.vot_timestamp)
  {
    grid.grid_label    = GridLabel::OCCUPIED;
    grid.vot_timestamp = 0;
    grid.id            = ID_DEFAULT;
  } // -- update motion

  grid.logit = std::fmax(0.0f, grid.logit);

  if(logit_count_ == 0 && grid.logit > 0)
  {
    grid.logit = std::fmax(0.0f, grid.logit - 1.0f);
//    if(grid.distance_y >= -2 && grid.distance_y < 0) {
//      UERROR << " id " << grid.id << " logit " << grid.logit << " row " << grid.row_test << " col " << grid.col_test << " " << grid.distance_x << " " << grid.distance_y;
//    }
  } // -- decrease prob
}

void fs::FreespaceManager::assignFusionMapVotInfo(const ObjectStates *vot_object_refine_ptr)
{
  if(vot_object_refine_ptr == nullptr)
  {
    return;
  }
  UINFO << "vot num: " << vot_object_refine_ptr->objects.size();
  for(auto &&ob : vot_object_refine_ptr->objects)
  {
    // clang-format off
    // -- get pos in world
    std::vector<FSVec2f> box_polygon_wrd{};
    std::vector<cv::Point> polygon_wrd_idx{};
    float poly_min_x = std::numeric_limits<float>::max();
    float poly_max_x = -std::numeric_limits<float>::max();
    float poly_min_y = std::numeric_limits<float>::max();
    float poly_max_y = -std::numeric_limits<float>::max(); // -- record min max
    if (getWordPolygon(ob, box_polygon_wrd, poly_min_x, poly_max_x, poly_min_y, poly_max_y)) { continue; }
    for (auto &p : box_polygon_wrd) { polygon_wrd_idx.emplace_back(std::round(p.x() * GRID_SCALE_INV), std::round(p.y() * GRID_SCALE_INV)); }// -- world pos --> idx

    const int start_x = (int) std::round(poly_min_x * GRID_SCALE_INV);
    const int start_y = (int) std::round(poly_min_y * GRID_SCALE_INV);
    std::vector<std::vector<cv::Point>> all_box_polygon_wrd(1);
    for (auto &p : polygon_wrd_idx) { all_box_polygon_wrd[0].emplace_back(p.x - start_x, p.y - start_y); } // -- move box to 0

    mask_cv_.setTo(cv::Scalar(0));
    cv::fillPoly(mask_cv_, all_box_polygon_wrd, cv::Scalar(255));
    MotionState motion_state = (std::hypot(ob.real_vx, ob.real_vy) > STOPPED_THRE) ? MotionState::MOVING : MotionState::STOPPED;
    if (ob.type == GridLabel::RTG_QC) { motion_state = ob.motion_state; }
    const int len_x = std::ceil((poly_max_x - poly_min_x) * GRID_SCALE_INV);
    const int len_y = std::ceil((poly_max_y - poly_min_y) * GRID_SCALE_INV);
    for (int x = 0; x < len_x; ++x) {
      for (int y = 0; y < len_y; ++y) {
        if (mask_cv_.at<uchar>(y, x) == 255) {
          auto &grid = fusion_map_.at((x + start_x) & 2047, (y + start_y) & 2047);
          grid.grid_label = ob.type;
          grid.vot_timestamp = vot_object_refine_ptr->timestamp;
          grid.id = ob.id;
          grid.motion_state = motion_state;
        }
      }
    }
//      cv::imwrite(save_path_ + "/" + std::to_string(fusion_timestamp) + "_" + std::to_string(ob.id) + "_cv.jpg", mask_cv_);
  }
}

void fs::FreespaceManager::assignTptsMapVotInfo(const ObjectStates *vot_object_refine_ptr) {
  if (vot_object_refine_ptr == nullptr) { return; }
  UINFO << "vot num: " << vot_object_refine_ptr->objects.size();
  for (auto &&ob : vot_object_refine_ptr->objects) {
    std::string expand{};
    const auto &approx_polygon = Expand::instance().getPolygonByOval(ob, dt_, expand);
    std::vector<FSVec2f> box_polygon_ego{};
    std::vector<cv::Point> polygon_ego_idx{};
    float poly_min_x = std::numeric_limits<float>::max();
    float poly_max_x = -std::numeric_limits<float>::max();
    float poly_min_y = std::numeric_limits<float>::max();
    float poly_max_y = -std::numeric_limits<float>::max(); // -- record min max

    for (auto &p : approx_polygon) {
      poly_min_x = std::min(poly_min_x, p.x);
      poly_max_x = std::max(poly_max_x, p.x);
      poly_min_y = std::min(poly_min_y, p.y);
      poly_max_y = std::max(poly_max_y, p.y);
      box_polygon_ego.emplace_back(FSVec2f(p.x, p.y));
    } // -- word
    for (auto &p : box_polygon_ego) { polygon_ego_idx.emplace_back(std::round(p.x() * GRID_SCALE_INV), std::round(p.y() * GRID_SCALE_INV)); }// -- world pos --> idx

    const int start_x = (int) std::round(poly_min_x * GRID_SCALE_INV);
    const int start_y = (int) std::round(poly_min_y * GRID_SCALE_INV);
    std::vector<std::vector<cv::Point>> box_polygon_local(1);
    for (auto &p : polygon_ego_idx) { box_polygon_local[0].emplace_back(p.x - start_x, p.y - start_y); } // -- move box to 0

    mask_cv_.setTo(cv::Scalar(0));
    cv::fillPoly(mask_cv_, box_polygon_local, cv::Scalar(255));
    MotionState motion_state = (std::hypot(ob.real_vx, ob.real_vy) > STOPPED_THRE) ? MotionState::MOVING : MotionState::STOPPED;
    if (ob.type == GridLabel::RTG_QC) { motion_state = ob.motion_state; }
    const int len_x = std::ceil((poly_max_x - poly_min_x) * GRID_SCALE_INV);
    const int len_y = std::ceil((poly_max_y - poly_min_y) * GRID_SCALE_INV);
    for (int x = 0; x < len_x; ++x) {
      for (int y = 0; y < len_y; ++y) {
        if (mask_cv_.at<uchar>(y, x) == 255) {
          int grid_x = (x + start_x) + FS_CAR_ROW_OUT;
          int grid_y = -(y + start_y) + FS_CAR_COL_OUT;
          if (grid_x < 0 || grid_x >= FS_ROW_OUT || grid_y < 0 || grid_y >= FS_COL_OUT) {
            continue;
          }
          auto &grid = tpts_map_[grid_x][grid_y];
          grid.grid_label = ob.type;
          grid.vot_timestamp = vot_object_refine_ptr->timestamp;
          grid.id = ob.id;
          grid.motion_state = motion_state;
        }
      }
    }
  }
}

void fs::FreespaceManager::resetFusionMap() {
  Grid tmp(0);
  for (int row = 0; row < GRID_MAP_SIZE; ++row) {
    for (int col = 0; col < GRID_MAP_SIZE; ++col) {
      memcpy(&(fusion_map_.at(row, col)), &tmp, sizeof(tmp));
    }
  } // -- mod grid
}

void fs::FreespaceManager::resetTptsMap() {
  Grid tmp(GridLabel::OCCUPIED);
  for (int row = 0; row < FS_ROW_OUT; ++row) {
    for (int col = 0; col < FS_COL_OUT; ++col) {
      if (tpts_map_[row][col].id == ID_DEFAULT) { continue; }
      memcpy(&(tpts_map_[row][col]), &tmp, sizeof(tmp));
    }
  } // -- mod grid
}

#if FS_CHECK(CFG_ROS2)
#if FS_CHECK(CFG_USE_MCAP_RAW)
#if FS_CHECK(CFG_PUB_ENU_FS)
void fs::FreespaceManager::drawBag(const std::unique_ptr<uto::proto::PerceptionFreespace> &message, const EgoMotion *ego_motion)
{
  Vis::drawSelfReceivePointCloud(*message.get(), ego_motion);
}
#else
void fs::FreespaceManager::drawBag(const std::unique_ptr<uto::proto::PerceptionFreespace> &message)
{
  Vis::drawSelfReceivePointCloud(*message.get());
}
#endif // -- CFG_PUB_ENU_FS
#endif // -- CFG_USE_MCAP_RAW
#else
#if FS_CHECK(CFG_DEBUG_PERCEPTION_FREESPACES)
void fs::FreespaceManager::drawBag(const PerceptionFreeSpace &message, const EgoMotion *ego_motion)
{
  Vis::drawSelfReceivePointCloud(message, ego_motion);
}
#endif // -- CFG_DEBUG_PERCEPTION_FREESPACES
#endif // -- CFG_ROS2

void fs::FreespaceManager::drawOther(const ObjectStates *vot_object_refine_ptr, const BevSeg *fsBev) {
#if FS_CHECK(CFG_VIS_ENABLE)
  Vis::drawRvizColorTable();
  Vis::drawVotObject(vot_object_refine_ptr, "", 0);
  Vis::drawVotObject(vot_object_refine_ptr, "expand", dt_);
  Vis::drawBevFs(fsBev);
  Vis::drawLidarPointCloudHistory(lidar_grids_buffer_.front());
  Vis::drawOutBboxRviz(0.1);
#if !FS_CHECK(CFG_ROS2)
  Vis::drawEgoBboxRviz();
#endif
#endif
}

#if FS_CHECK(CFG_ROS2)
void fs::FreespaceManager::fuseFreespace(const int64_t fusion_timestamp,
                                         const LidarFS *fs_lidar_ptr,
                                         const ObjectStates *vot_object_refine_ptr,
                                         std::unique_ptr<uto::proto::PerceptionFreespace> &fusion_fs_output_ptr,
                                         const EgoMotion *emo_align_pos_fusion,
                                         const EgoMotion *emo_pos_fusion,
                                         std::unique_ptr<uto::proto::PerceptionFreespace> &raw_lidar_fs,
                                         std::vector<std::vector<fs::LineCoeff>> &coeff_lines,
                                         bool is_lock_station,
                                         bool is_electric_fence,
                                         bool is_reverse)
#else
void fs::FreespaceManager::fuseFreespace(const int64_t                         fusion_timestamp,
                                         const BevSeg                         *bev_seg_refine_ptr,
                                         const LidarFS                        *fs_lidar_ptr,
                                         const ObjectStates                   *vot_object_refine_ptr,
                                         std::unique_ptr<PerceptionFreeSpace> &fusion_fs_output_ptr,
                                         const EgoMotion                      *emo_pos_latest,
                                         const EgoMotion                      *emo_pos_fusion,
                                         float                                 resolution,
                                         bool                                  is_reverse,
                                         std::vector<cv::Point2f>             &polygon_lane,
                                         bool                                  is_electric_fence,
                                         bool                                  is_lock_station,
                                         const Lidar_FreeSpace_v2             *raw_lidar_fs)
#endif // -- CFG_ROS2
{
  fusion_timestamp_ = fusion_timestamp;
  auto start = std::chrono::steady_clock::now();

  UINFO << "================================= start to fuse freespace, timestamp: " << fusion_timestamp << " ns =================================";

  const Vehicle &vehicle = Vehicle::getVehicle();
  const auto delta_pos = vehicle.transformT0toT1Ego(FSVec3f::Zero()); // -- 根据T矩阵计算的平移参数与实际自车的运动方向相反***

  fusion_map_.updateEgo(delta_pos.head(2), vehicle.getVecEgo2Wrd().head(2), vehicle.getRotEgo2Wrd().topLeftCorner(2, 2));
  if (vot_object_refine_ptr != nullptr) {
    dt_ = static_cast<float>(fusion_timestamp - vot_object_refine_ptr->timestamp) / (TIME_SCALE_MS * 1000);
    assignFusionMapVotInfo(vot_object_refine_ptr);
  } // -- for each object
int64_t curr_time = fusion_timestamp;
  for (const auto &point : fs_lidar_ptr->points) {
    if (point.lidar_label == GridLabel::UNOCCUPIED) { continue; }
    if (std::fabs(point.x) < HALF_GRID_MAP_SIZE_M && std::fabs(point.y) < HALF_GRID_MAP_SIZE_M) {
      const GridMap::Index idx = fusion_map_.pos2ModIdx(FSVec2f{point.x, point.y});
      auto &grid = fusion_map_.at(idx);
//      if(point.y < 0 && point.y > -2 ){
//          UERROR << "point is " << point.x << " y is " << point.y << " label is " << static_cast<int>(grid.grid_label) << " logit " << grid.logit << " is_noise " << point.is_noise << " row " << idx.row  << " col " << idx.col;
//      }
//      if(idx.row == 1916 && idx.col == 84){
//          UERROR << "point is " << point.x << " y is " << point.y << " label is " << static_cast<int>(grid.grid_label) << " logit " << grid.logit << " is_noise " << point.is_noise << " row " << idx.row  << " col " << idx.col;
//      }
      //// -- 2.1 assign top/bottom
      if (grid.top < 1e-6 && grid.bottom < 1e-6) {
        grid.top = point.top;
        grid.bottom = point.bottom;
        grid.ground = point.ground;
      } // -- init
      else {
          grid.top = point.top;
          grid.bottom = point.bottom;
          grid.ground = point.ground;
//        grid.top = grid.top * HISTORY_RATIO + CURRENT_RATIO * point.top;
//        grid.bottom = grid.bottom * HISTORY_RATIO + CURRENT_RATIO * point.bottom;
//      grid.ground = point.ground;
      } // -- height fusion

      //// -- 2.2 increase probability
      //logit上下限
      float up_limit = LOGIT_UP_STOPPED_THRE;
      float down_limit = 0.0f;
      if (FS_BUILT_IN_CLASS_CAN_MOVING.at(grid.grid_label) == MotionState::MOVING) {
        if (vot_object_refine_ptr != nullptr && grid.vot_timestamp == vot_object_refine_ptr->timestamp) { grid.tpts = true; }// -- flag tpts

        if (grid.motion_state != MotionState::MOVING) {
          if (grid.logit < LOGIT_UP_CAN_MOVING_THRE) {
            grid.logit += 1;
            //            if(grid.logit >2) {grid.grid_label = GridLabel::OCCUPIED;}
          }
          else { grid.logit = LOGIT_UP_CAN_MOVING_THRE; }
        } // -- can moving (vot)object set logit up threshold
        up_limit = LOGIT_UP_CAN_MOVING_THRE;
      }   // -- can moving, MotionState of UNKNOWN: STOPPED
      else {
        if (point.y >= TPTS_LEFT && point.y <= TPTS_RIGHT) {
          if (vot_object_refine_ptr != nullptr && grid.vot_timestamp == vot_object_refine_ptr->timestamp) { grid.tpts = true; }// -- flag tpts
          grid.motion_state = point.motion_state;
          if (grid.grid_label == GridLabel::OTHER || grid.grid_label == GridLabel::UNOCCUPIED || grid.grid_label == GridLabel::UNKNOWN || grid.is_noise) { grid.grid_label = GridLabel::OCCUPIED; }
        }
          // if(point.lidar_label == GridLabel::TREE_BRANCH){grid.grid_label = point.lidar_label; }
          if (grid.logit < LOGIT_UP_STOPPED_THRE) {
          grid.logit += 1;
          if (grid.logit >= 2 && grid.grid_label == GridLabel::UNKNOWN) { grid.grid_label = GridLabel::OCCUPIED; }
        }
        else { grid.logit = LOGIT_UP_STOPPED_THRE; }
      } // -- can't moving object don't set logit up threshold

      //增加除噪处理(原来的后置除噪移动到此处)
      //首先查询激光噪点/强化补偿
      float lidar_compensation = -1.0f * std::min(0.25f, std::max(0.1f, 1.0f - std::fabs(static_cast<float>(point.x)) / VISION_DISTANCE));
      auto it = LIDAR_COMPENSATION.find(point.lidar_label);
      if (it != LIDAR_COMPENSATION.end()) {
        lidar_compensation = it->second;
        //特殊类型的点需要刷label，防止下游收到label异常的点
        grid.grid_label = GridLabel::OCCUPIED;
      }
      //判断是否为视觉噪点
      if(point.is_noise){
        //如果为噪点，增加激光补偿
        grid.logit += lidar_compensation;
      }else{
        //如果非噪点，对强增加点增加概率
        if(lidar_compensation > 0.){
          grid.logit += -1;
          UERROR << " lidar_compensation " <<lidar_compensation;
        }
      }
//      if(point.x > 0 && point.x < 15 && point.y < 0 && point.y > -2 ){
//        UERROR << "11point is " << point.x << " y is " << point.y << " label is " << static_cast<int>(grid.grid_label) << " logit " << grid.logit << " is_noise " << point.is_noise << " row " << idx.row  << " col " << idx.col << " " << lidar_compensation;
//      }
//      if(idx.row == 1916 && idx.col == 84){
//        UERROR << "point is " << point.x << " y is " << point.y << " label is " << static_cast<int>(grid.grid_label) << " logit " << grid.logit << " is_noise " << point.is_noise << " row " << idx.row  << " col " << idx.col;
//      }
      //约束logit上下限
      grid.logit = std::min(std::max(grid.logit, down_limit), up_limit);
      //   UERROR << "pt is " << pos.x() << " y is " << pos.y() << " label is " << static_cast<int>(grid.grid_label) << " " << grid.logit;
//      if(point.y < 0 && point.y > -2 ){
//          UERROR << "22point is " << point.x << " y is " << point.y << " label is " << static_cast<int>(grid.grid_label) << " logit " << grid.logit << " is_noise " << point.is_noise << " row " << idx.row  << " col " << idx.col << " " << lidar_compensation;
//      }
    }
  }
  if (++logit_count_ >= FRAMES_DESCEND) { logit_count_ = 0;
  UERROR << "全局";}
#if FS_CHECK(CFG_VIS_ENABLE)
  Vis::drawGridMapsplit(fusion_map_);
#endif// -- CFG_VIS_ENABLE

#if FS_CHECK(CFG_ROS2)
  getFreespace(fusion_fs_output_ptr);

#else
  getFreespace(emo_pos_latest, fusion_fs_output_ptr.get(), emo_pos_fusion, resolution, is_reverse);
#endif // -- CFG_ROS2

#if FS_CHECK(CFG_AIV_LOCK_STATION_PUSH)
  if(fs::MapProvider::getMapProvider().useLockStationPush() && is_lock_station)
  {
    UWARN << " case lock_station";
#if FS_CHECK(CFG_ROS2)
    coeff_lines.clear();
    removeOfLockStation(fusion_fs_output_ptr.get(), coeff_lines,emo_pos_fusion);
#else
    polygon_lane.clear();
    fusion_fs_output_ptr = removeOfLockStation(fusion_fs_output_ptr.get(), polygon_lane, bev_seg_refine_ptr ,is_reverse);
#endif
  }
#endif // -- CFG_AIV_LOCK_STATION_PUSH

#if FS_CHECK(CFG_AIV_ELECTRIC_FENCE)
  if(fs::MapProvider::getMapProvider().useLaneRemoveGrass() && is_electric_fence)
  {
    UWARN << " case electric fence";
#if FS_CHECK(CFG_ROS2)
    removeOfGrass(fusion_fs_output_ptr.get(),coeff_lines,emo_pos_fusion);
#else
    fusion_fs_output_ptr = removeOfGrass(fusion_fs_output_ptr.get(),polygon_lane, bev_seg_refine_ptr, is_reverse);
#endif
  }
#endif // -- CFG_AIV_ELECTRIC_FENCE

#if FS_CHECK(CFG_HDT_DT_BT_REMOVE_GRASS)
  removeOfGrass(fusion_fs_output_ptr.get(), coeff_lines, emo_pos_fusion);
#endif // -- CFG_HDT_DT_BT_REMOVE_GRASS

#if FS_CHECK(CFG_PUB_CONTOUR)
  auto ds = std::chrono::steady_clock::now();
  downSample(fusion_fs_output_ptr);
  UINFO << "ds:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - ds).count();
#endif // --CFG_PUB_CONTOUR

#if FS_CHECK(CFG_ROS2)
    ego2Pub(fusion_fs_output_ptr.get(), emo_align_pos_fusion);
#endif
  
#if FS_CHECK(CFG_VIS_ENABLE)
#if FS_CHECK(!CFG_DEBUG_PERCEPTION_FREESPACES)
#if FS_CHECK(CFG_ROS2)
#if FS_CHECK(CFG_PUB_ENU_FS)
  Vis::drawSelfReceivePointCloud(*fusion_fs_output_ptr, emo_align_pos_fusion);
#else
    Vis::drawSelfReceivePointCloud(*fusion_fs_output_ptr);
#endif // --CFG_PUB_ENU_FS
#else
  Vis::drawSelfReceivePointCloud(*fusion_fs_output_ptr);
#endif // -- CFG_ROS2
#endif
#if FS_CHECK(CFG_ROS2)
drawOther(vot_object_refine_ptr);
#else
drawOther(vot_object_refine_ptr, bev_seg_refine_ptr);
#endif // -- CFG_ROS2
#endif // -- CFG_VIS_ENABLE
  const float duration = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
  UINFO << "process  time:  " << duration << " ms";
  UINFO << "=================================================================================================================================";
  // clang-format on
}

#if FS_CHECK(CFG_ROS2)
void fs::FreespaceManager::removeOfGrass(uto::proto::PerceptionFreespace *lidar_fs, const std::vector<std::vector<fs::LineCoeff>> &cameras_lines, const EgoMotion *emo_fusion)
{
  if(lidar_fs == nullptr || cameras_lines.empty() || emo_fusion == nullptr)
  {
    return;
  }
  int cur = 0;
  int end = lidar_fs->perception_gridmap().size() - 1;
  while(cur < end)
  {
    const auto &p = lidar_fs->perception_gridmap(cur);
    if(p.position().x() > REMOVE_REGION_BACK && p.position().x() < REMOVE_REGION_FRONT && p.position().y() > REMOVE_REGION_RIGHT && p.position().y() < REMOVE_REGION_LEFT)
    {
      // -- is between in line
      bool between_lines = false;
      for(const auto &camera_lines : cameras_lines)
      {
        if(pointBetweenLanes(p.position().x(), p.position().y(), camera_lines, emo_fusion))
        {
          between_lines = true;
          break;
        }
      }
      //// -- remove noise by lane
      if(between_lines && FS_PROTO_CLASS_TO_CLASS.at(p.label()) == GridLabel::OTHER && p.top() < 0.8f)
      {
        lidar_fs->mutable_perception_gridmap(cur)->CopyFrom(lidar_fs->perception_gridmap()[end]);
        lidar_fs->mutable_perception_gridmap()->erase(lidar_fs->mutable_perception_gridmap()->end() - 1);
        --end;
        continue;
      }
    }
    ++cur;
  }
}

void fs::FreespaceManager::ego2Pub(uto::proto::PerceptionFreespace *fusion_fs_ptr, const EgoMotion *emo_pos_fusion)
{
  //  uto::proto::Header header;
  //  header.set_time_meas(fusion_timestamp_);
  //  header.set_time_pub(fusion_timestamp_);
  //  fusion_fs_ptr->mutable_header()->CopyFrom(header);
  //  auto origin_point = fusion_fs_ptr->mutable_origin_point();
  //  origin_point->set_x(emo_pos_fusion->translation.x() + emo_pos_fusion->init_x);
  //  origin_point->set_y(emo_pos_fusion->translation.y() + emo_pos_fusion->init_y);
  //  fusion_fs_ptr->set_origin_point_theta(emo_pos_fusion->yaw);
  //  fusion_fs_ptr->set_length(FS_ROW_OUT);
  //  fusion_fs_ptr->set_width(FS_COL_OUT);
  //  fusion_fs_ptr->set_resolution(GRID_SCALE);
  //  fusion_fs_ptr->set_vehicle_origin_length(FS_CAR_ROW_OUT);
  //  fusion_fs_ptr->set_vehicle_origin_width(FS_CAR_COL_OUT);

  uto::proto::Header header;
  header.set_time_meas(fusion_timestamp_);
  //  header.set_time_pub(fusion_timestamp_);
  fusion_fs_ptr->mutable_header()->CopyFrom(header);
  auto origin_point = fusion_fs_ptr->mutable_origin_point();
#if FS_CHECK(CFG_PUB_ENU_FS)
  if(!emo_pos_fusion)
  {
    UERROR << "ptr null";
    return;
  }
  auto Twl = Utils::makeTFrom6Dof(emo_pos_fusion->yaw, 0., 0., emo_pos_fusion->translation.x(), emo_pos_fusion->translation.y(), 0.);
  origin_point->set_x(emo_pos_fusion->translation.x() + emo_pos_fusion->init_x);
  origin_point->set_y(emo_pos_fusion->translation.y() + emo_pos_fusion->init_y);
  fusion_fs_ptr->set_origin_point_theta(emo_pos_fusion->yaw);
#else
  origin_point->set_x(0.0f);
  origin_point->set_y(0.0f);
  fusion_fs_ptr->set_origin_point_theta(0.0f);
#endif
  fusion_fs_ptr->set_length(FS_ROW_OUT);
  fusion_fs_ptr->set_width(FS_COL_OUT);
  fusion_fs_ptr->set_resolution(GRID_SCALE);
  fusion_fs_ptr->set_vehicle_origin_length(FS_CAR_ROW_OUT);
  fusion_fs_ptr->set_vehicle_origin_width(FS_CAR_COL_OUT);

#if FS_CHECK(CFG_PUB_ENU_FS)
  for(int i = 0; i < fusion_fs_ptr->perception_gridmap().size(); ++i)
  {
    auto point_pos = fusion_fs_ptr->mutable_perception_gridmap(i)->mutable_position();

    const FSVec3f point_enu = (Twl * FSVec4f{point_pos->x(), point_pos->y(), 0.f, 1.0f}).topRows(3); // -- 1:1  enu car coord
    point_pos->set_x(point_enu.x() + emo_pos_fusion->init_x);
    point_pos->set_y(point_enu.y() + emo_pos_fusion->init_y);
  }
#else
#endif
}

bool fs::FreespaceManager::pointBetweenLanes(float x, float y, const std::vector<fs::LineCoeff> &camera_lines, const EgoMotion *emo_fusion)
{
  if(camera_lines.empty() || emo_fusion == nullptr)
  {
    return false;
  }

  bool res = false;
  for(auto &line : camera_lines)
  {
    const auto     Twb_lidar = Utils::makeTFrom6Dof(emo_fusion->yaw, 0., 0., emo_fusion->translation.x(), emo_fusion->translation.y(), 0.);
    const auto     Twb_lanes = Utils::makeTFrom6Dof(line.emo.yaw, 0., 0., line.emo.translation.x(), line.emo.translation.y(), 0.);
    const FSMat4x4 Tcl       = Utils::inverse(Twb_lanes) * Twb_lidar;
    const float    sync_x    = Tcl(0, 0) * x + Tcl(0, 1) * y + Tcl(0, 3);
    const float    sync_y    = Tcl(1, 0) * x + Tcl(1, 1) * y + Tcl(1, 3);
    if(line.lines_pos == LinesPos::LEFT)
    {
      res = res || (sync_y >= HALF_EGO_WIDTH_M && Utils::pointLowPolynomial(sync_x, sync_y, line.coeff));
    } // -- below line
    else if(line.lines_pos == LinesPos::RIGHT)
    {
      res = res || (sync_y <= -HALF_EGO_WIDTH_M && Utils::pointOnPolynomial(sync_x, sync_y, line.coeff));
    } // -- on line
    else
    {
    }
  }
  return res;
}

void fs::FreespaceManager::removeOfLockStation(uto::proto::PerceptionFreespace *lidar_fs, const std::vector<std::vector<fs::LineCoeff>> &coeff_lines, const EgoMotion *emo_fusion)
{
  if(lidar_fs == nullptr || emo_fusion == nullptr)
  {
    return;
  }

  int cur = 0;
  int end = lidar_fs->perception_gridmap().size() - 1;
  while(cur < end)
  {
    const auto &p = lidar_fs->mutable_perception_gridmap(cur);
    if(p->position().x() > REMOVE_REGION_BACK && p->position().x() < REMOVE_REGION_FRONT && p->position().y() > REMOVE_REGION_RIGHT && p->position().y() < REMOVE_REGION_LEFT)
    {
      //// -- push 20cm
      if(p->position().y() < -HALF_EGO_WIDTH_M)
      {
        p->mutable_position()->set_y(p->mutable_position()->y() - 0.2);
      }
      else if(p->position().y() > HALF_EGO_WIDTH_M)
      {
        p->mutable_position()->set_y(p->mutable_position()->y() + 0.2);
      }

      //// -- remove noise by lane
      if(!coeff_lines.empty())
      {
        // -- is between in line
        bool between_lanes = false;
        for(const auto &camera_lines : coeff_lines)
        {
          if(pointBetweenLanes(p->position().x(), p->position().y(), camera_lines, emo_fusion))
          {
            between_lanes = true;
            break;
          }
        } // -- fc-lanes rc-lanes
        // -- delete noise
        if(between_lanes && (FS_PROTO_CLASS_TO_CLASS.at(p->label()) == GridLabel::LOCK_BOX)) // -- inner --> lcm --> inner
        {
          lidar_fs->mutable_perception_gridmap(cur)->CopyFrom(lidar_fs->perception_gridmap()[end]);
          lidar_fs->mutable_perception_gridmap()->erase(lidar_fs->mutable_perception_gridmap()->end() - 1);
          --end;
          continue;
        }
      }
    }
    ++cur;
  }
}

std::unique_ptr<uto::proto::PerceptionFreespace> fs::FreespaceManager::tptsLidarFs(const std::unique_ptr<uto::proto::PerceptionFreespace> &lidar_fs_ptr, bool is_reverse)
{
  if(lidar_fs_ptr == nullptr)
  {
    return nullptr;
  }
  std::unique_ptr<uto::proto::PerceptionFreespace> fusion_fs_ptr = std::make_unique<uto::proto::PerceptionFreespace>();

  uto::proto::Header header;
  header.set_time_meas(fusion_timestamp_);
  //  header.set_time_pub(fusion_timestamp_);
  fusion_fs_ptr->mutable_header()->CopyFrom(header);
  auto origin_point = fusion_fs_ptr->mutable_origin_point();
  origin_point->set_x(0.0f);
  origin_point->set_y(0.0f);
  fusion_fs_ptr->set_origin_point_theta(0.0f);
  fusion_fs_ptr->set_length(FS_ROW_OUT);
  fusion_fs_ptr->set_width(FS_COL_OUT);
  fusion_fs_ptr->set_resolution(GRID_SCALE);
  fusion_fs_ptr->set_vehicle_origin_length(FS_CAR_ROW_OUT);
  fusion_fs_ptr->set_vehicle_origin_width(FS_CAR_COL_OUT);

#if FS_CHECK(CFG_USE_LIGHT_COMMON)
  for(auto &&p : *lidar_fs_ptr->mutable_perception_gridmap())
  {
    const auto &idx  = grid_map_.pos2ModIdx(FSVec2f{p.position().x(), p.position().y()});
    const auto &grid = grid_map_.at(idx);
    lidar_fs_ptr->add_grid_positions(p.position().x()); // x  自车系
    lidar_fs_ptr->add_grid_positions(p.position().y()); // y  自车系
    lidar_fs_ptr->add_altitudes(p.top() * 1000);        // top
    lidar_fs_ptr->add_altitudes(p.bottom() * 1000);
    if(grid.motion_state == MotionState::MOVING && MOVING_CLASS_TO_HDT2_LIDAR_CLASS.count(grid.grid_label))
    {
      lidar_fs_ptr->add_labels(MOVING_CLASS_TO_HDT2_LIDAR_CLASS.at(grid.grid_label));
    }
    else
    {
      lidar_fs_ptr->add_labels(CLASS_TO_FS_PROTO_CLASS.at(grid.grid_label));
    }
  }
#else
  for(auto &&p : *lidar_fs_ptr->mutable_perception_gridmap())
  {
    const int x_id = static_cast<int>(std::round(p.position().x() * GRID_SCALE_INV + FS_CAR_ROW_OUT));
    const int y_id = static_cast<int>(std::round(-p.position().y() * GRID_SCALE_INV + FS_CAR_COL_OUT));
    if(x_id < 0 || x_id >= FS_ROW_OUT || y_id < 0 || y_id >= FS_COL_OUT)
    {
      continue;
    }
    const auto &grid     = tpts_map_[x_id][y_id];
    auto        grid_map = fusion_fs_ptr->add_perception_gridmap();
    grid_map->set_label(CLASS_TO_FS_PROTO_CLASS.at(grid.grid_label));
    grid_map->set_move_status(MOVESTATUS_TO_HDT2_MOVESTATUS.at(grid.motion_state));
    grid_map->set_top(p.top());
    grid_map->set_bottom(p.bottom());
    grid_map->set_object_id(grid.id);
    auto point_pos = grid_map->mutable_position();
    point_pos->set_x(p.position().x());
    point_pos->set_y(p.position().y());
  }
#endif
  return std::move(fusion_fs_ptr);
}

void fs::FreespaceManager::getFreespace(std::unique_ptr<uto::proto::PerceptionFreespace> &lidar_fs_ptr)
{
  constexpr int min_x_ = -FS_CAR_ROW_OUT;
  constexpr int max_x_ = +FS_ROW_OUT - FS_CAR_ROW_OUT;
  constexpr int min_y_ = -FS_CAR_COL_OUT;
  constexpr int max_y_ = +FS_COL_OUT - FS_CAR_COL_OUT;

  //  uto::proto::Header header;
  //  header.set_time_meas(fusion_timestamp_);
  //  header.set_time_pub(fusion_timestamp_);
  //  lidar_fs_ptr->mutable_header()->CopyFrom(header);
  //  auto origin_point = lidar_fs_ptr->mutable_origin_point();
  //  origin_point->set_x(0.0f);
  //  origin_point->set_y(0.0f);
  //  lidar_fs_ptr->set_origin_point_theta(0.0f);
  //  lidar_fs_ptr->set_length(FS_ROW_OUT);
  //  lidar_fs_ptr->set_width(FS_COL_OUT);
  //  lidar_fs_ptr->set_resolution(GRID_SCALE);
  //  lidar_fs_ptr->set_vehicle_origin_length(FS_CAR_ROW_OUT);
  //  lidar_fs_ptr->set_vehicle_origin_width(FS_CAR_COL_OUT);

  for(int row = 0; row < GRID_MAP_SIZE; ++row)
  {
    for(int col = 0; col < GRID_MAP_SIZE; ++col)
    {
      auto &grid = fusion_map_.at(row, col);
      grid.id++;
      grid.row_test = row;
      grid.col_test = col;
      if(grid.logit >= 2 || grid.tpts)
      {
        const auto &ego_idx_cur_in = fusion_map_.modIdx2EgoCenterIdxCurrent(GridMap::Index{row, col}); // -- mod grid -->  grid of ego not left down
        if(ego_idx_cur_in.x() < min_x_ || ego_idx_cur_in.x() >= max_x_ || ego_idx_cur_in.y() < min_y_ || ego_idx_cur_in.y() >= max_y_)
        {
          if(grid.tpts)
          {
            grid.tpts = false;
          }
        } // -- select 200*500
        else
        {
          const FSVec2f pos = ego_idx_cur_in * GRID_SCALE;
          grid.distance_x = pos.x();
          grid.distance_y = pos.y();

          if(grid.tpts)
          {
            grid.tpts = false;
#if FS_CHECK(CFG_USE_LIGHT_COMMON)
            lidar_fs_ptr->add_grid_positions(pos.x());    // x  自车系
            lidar_fs_ptr->add_grid_positions(pos.y());    // y  自车系
            lidar_fs_ptr->add_altitudes(grid.top * 1000); // top
            lidar_fs_ptr->add_altitudes(grid.bottom * 1000);
            if(grid.motion_state == MotionState::MOVING && MOVING_CLASS_TO_HDT2_LIDAR_CLASS.count(grid.grid_label))
            {
              lidar_fs_ptr->add_labels(MOVING_CLASS_TO_HDT2_LIDAR_CLASS.at(grid.grid_label));
            }
            else
            {
              lidar_fs_ptr->add_labels(CLASS_TO_FS_PROTO_CLASS.at(grid.grid_label));
            }
#else
            auto grid_map = lidar_fs_ptr->add_perception_gridmap();
            grid_map->set_label(CLASS_TO_FS_PROTO_CLASS.at(grid.grid_label));
            grid_map->set_move_status(MOVESTATUS_TO_HDT2_MOVESTATUS.at(grid.motion_state));
            grid_map->set_top(grid.top);
            grid_map->set_bottom(grid.bottom);
            grid_map->set_object_id(grid.id);
            auto point_pos = grid_map->mutable_position();
            point_pos->set_x(pos.x());
            point_pos->set_y(pos.y());
#endif
          } // -- tpts
          else
          {
//            if(pos.y() >= -2 && pos.y() < 0) {
//              UERROR << "pt is " << pos.x() << " y is " << pos.y() << " logit " << grid.logit << " row " << row << " col " << col << " id "
//              << grid.id << " " << grid.id << " " << grid.row_test << " " << grid.col_test << " " << grid.distance_x << " " << grid.distance_y;
//            }
            if(grid.logit >= 2.0f)
            {
#if FS_CHECK(CFG_USE_LIGHT_COMMON)
              lidar_fs_ptr->add_grid_positions(pos.x());       // x  自车系
              lidar_fs_ptr->add_grid_positions(pos.y());       // y  自车系
              lidar_fs_ptr->add_altitudes(grid.top * 1000);    // top
              lidar_fs_ptr->add_altitudes(grid.bottom * 1000); // bottom
              if(grid.motion_state == MotionState::MOVING && MOVING_CLASS_TO_HDT2_LIDAR_CLASS.count(grid.grid_label))
              {
                lidar_fs_ptr->add_labels(MOVING_CLASS_TO_HDT2_LIDAR_CLASS.at(grid.grid_label));
              }
              else
              {
                lidar_fs_ptr->add_labels(CLASS_TO_FS_PROTO_CLASS.at(grid.grid_label));
              }
#else

              //  除噪相关内容已在上一级实现
              auto grid_map = lidar_fs_ptr->add_perception_gridmap();
              grid_map->set_label(CLASS_TO_FS_PROTO_CLASS.at(grid.grid_label));
              grid_map->set_move_status(MOVESTATUS_TO_HDT2_MOVESTATUS.at(grid.motion_state));
              grid_map->set_top(grid.top);
              grid_map->set_bottom(grid.bottom);
              grid_map->set_object_id(grid.id);
              auto point_pos = grid_map->mutable_position();
              point_pos->set_x(pos.x());
              point_pos->set_y(pos.y());
#endif
            }
          }
        }
      }
      updateLabelMotionProb(grid);
    }
  }
}

#if FS_CHECK(CFG_PUB_CONTOUR)
void fs::FreespaceManager::downSample(std::unique_ptr<uto::proto::PerceptionFreespace> &fusion_fs_ptr)
{
  if(fusion_fs_ptr == nullptr)
  {
    return;
  }
  // clang-format off
  UINFO << "before: " << fusion_fs_ptr->perception_gridmap().size();
  memset(raw_map_, 0, sizeof(raw_map_));

  std::vector<cv::Point> lut(fusion_fs_ptr->perception_gridmap().size());
  const auto &map_ego = fusion_fs_ptr->perception_gridmap();
  for (int i = 0; i < map_ego.size(); ++i) {
    int row = static_cast<int>(std::round(map_ego[i].position().x() * GRID_SCALE_INV * 0.33f + FS_CAR_ROW_OUT));// --分辨率调整为i原来的两倍
    int col = static_cast<int>(std::round(-map_ego[i].position().y() * GRID_SCALE_INV * 0.33f + FS_CAR_COL_OUT));
    if (row < 0 || row > FS_ROW_OUT - 1 || col < 0 || col > FS_COL_OUT - 1) { continue; }
    raw_map_[row][col] = 1;
    lut[i] = cv::Point(row, col);
  }

  // -- downSample
  int expand = 2;
  for (int col = 0; col < FS_COL_OUT; ++col) {
    for (int row = 0; row < FS_ROW_OUT; ++row) {
      if (raw_map_[row][col] == 0) { continue; }
      else {
        if (col < expand || row < expand || col >= FS_COL_OUT - 1 - expand || row >= FS_ROW_OUT - 1 - expand) {
          raw_map_[row][col] = 2;
          continue;
        }
        else {
          if (raw_map_[row][col + 1] == 0 && raw_map_[row][col + 2] == 0) { raw_map_[row][col] = 2; }
          else if (raw_map_[row][col - 1] == 0 && raw_map_[row][col - 2] == 0) { raw_map_[row][col] = 2; }
          else if (raw_map_[row + 1][col] == 0 && raw_map_[row + 2][col] == 0) { raw_map_[row][col] = 2; }
          else if (raw_map_[row - 1][col] == 0 && raw_map_[row - 2][col] == 0) { raw_map_[row][col] = 2; }
          else {}
        }
      }
    }
  }

  int cur = 0;
  int end = lut.size() - 1;
  while (cur < end) {
    if (lut[cur].x < 0 || lut[cur].x > FS_ROW_OUT - 1 || lut[cur].y < 0 || lut[cur].y > FS_COL_OUT - 1) { continue; }
    if (raw_map_[lut[cur].x][lut[cur].y] == 0) { continue; }
    else {
      if (raw_map_[lut[cur].x][lut[cur].y] == 2) {
        //fusion_fs_ptr->mutable_perception_gridmap(cur)->set_label(CLASS_TO_FS_PROTO_CLASS.at(GridLabel::PEDESTRIAN));
        ++cur;
      }
      else {
        std::swap(lut[cur], lut[end]);
        fusion_fs_ptr->mutable_perception_gridmap(cur)->CopyFrom(fusion_fs_ptr->perception_gridmap()[end]);
        fusion_fs_ptr->mutable_perception_gridmap()->erase(fusion_fs_ptr->mutable_perception_gridmap()->end() - 1);
        --end;
      }
    }
  }
  UINFO << "after: " << fusion_fs_ptr->perception_gridmap().size();
  // clang-format on
}
#endif // -- CFG_PUB_CONTOUR

#else

std::unique_ptr<PerceptionFreeSpace> fs::FreespaceManager::removeOfGrass(PerceptionFreeSpace *lidar_fs, std::vector<cv::Point2f> &polygon, const BevSeg *bev_seg_ptr, bool is_reverse)
{
  if(lidar_fs == nullptr)
  {
    return nullptr;
  }
  if(is_reverse)
  {
    for(auto &p : polygon)
    {
      p *= -1;
    }
  }
  PerceptionFreeSpace lidar_fs_new{};
  lidar_fs_new.stHeader.nTimeStamp = lidar_fs->stHeader.nTimeStamp;
  lidar_fs_new.stHeader.nCounter   = lidar_fs->stHeader.nCounter;
  lidar_fs_new.stHeader.nFrameID   = lidar_fs->stHeader.nCounter;
  lidar_fs_new.fResolution         = lidar_fs->fResolution;
  lidar_fs_new.nVehicleOriginRows  = lidar_fs->nVehicleOriginRows;
  lidar_fs_new.nVehicleOriginCols  = lidar_fs->nVehicleOriginCols;
  lidar_fs_new.fReserved[8]        = 2;
  auto &points                     = lidar_fs_new.gstPoints;

  //add query list
  std::unordered_map<std::pair<float, float>, std::pair<float, int>, pair_hash> query;

  if(bev_seg_ptr != nullptr)
  {
    for(auto &&p : bev_seg_ptr->points)
    {
      if(p.label == 1 && p.score > 0.5f && p.x > REMOVE_REGION_BACK && p.x < REMOVE_REGION_FRONT && p.y > -2 && p.y < 2)
      {
        int16_t x = static_cast<int16_t>(std::round(p.x / lidar_fs_new.fResolution + lidar_fs_new.nVehicleOriginRows));
        int16_t y = static_cast<int16_t>(std::round(-p.y / lidar_fs_new.fResolution + lidar_fs_new.nVehicleOriginCols));
        query.emplace(std::make_pair(x, y), std::make_pair(p.score, p.label));
      }
    }
  }

  for(auto &&p : lidar_fs->gstPoints)
  {
    if(!polygon.empty())
    {
      float x = (p.nRowIndex - lidar_fs->nVehicleOriginRows) * lidar_fs->fResolution;
      float y = (-p.nColIndex + lidar_fs->nVehicleOriginCols) * lidar_fs->fResolution;

      if(x > REMOVE_REGION_BACK && x < REMOVE_REGION_FRONT && y > -2 && y < 2)
      {
        //        if(Utils::pointInPoly(x, y, polygon) && FS_LCM_CLASS_TO_CLASS.at(p.nType) == GridLabel::OTHER && p.fHeight < 0.8)
        if(Utils::pointInPoly(x, y, polygon))
        {
          if(query.find(std::make_pair(p.nRowIndex, p.nColIndex)) != query.end())
          {
            if(query.at(std::make_pair(p.nRowIndex, p.nColIndex)).second == 1 && query.at(std::make_pair(p.nRowIndex, p.nColIndex)).first > 0.5f)
            {
              continue;
            }
          }
        }
      } // -- remove weeds
        //      if(x > REMOVE_REGION_BACK && x < REMOVE_REGION_FRONT && y > -1.65 && y < 1.65 && FS_LCM_CLASS_TO_CLASS.at(p.nType) == GridLabel::OTHER)
        //      {
        //        UDEBUG << "left: " << polygon[0] << " -- " << polygon[1] << "      right: " << polygon[3] << " -- " << polygon[2] << "     true: " << x << " " << y << " " << p.fHeight << "  type: " << CLASS_STRINGS.at(FS_LCM_CLASS_TO_CLASS.at(p.nType));
        //      } // -- debug info
    }

    points.emplace_back(p);
  }
  lidar_fs_new.nPointNum = lidar_fs_new.gstPoints.size();
  return std::make_unique<PerceptionFreeSpace>(lidar_fs_new);
}

std::unique_ptr<PerceptionFreeSpace> fs::FreespaceManager::removeOfLockStation(PerceptionFreeSpace *lidar_fs, const std::vector<cv::Point2f> &polygon, const BevSeg *bev_seg_ptr, bool is_reverse)
{
  if(lidar_fs == nullptr)
  {
    return nullptr;
  }
  PerceptionFreeSpace lidar_fs_new{};
  lidar_fs_new.stHeader.nTimeStamp = lidar_fs->stHeader.nTimeStamp;
  lidar_fs_new.stHeader.nCounter   = lidar_fs->stHeader.nCounter;
  lidar_fs_new.stHeader.nFrameID   = lidar_fs->stHeader.nFrameID;
  lidar_fs_new.fResolution         = lidar_fs->fResolution;
  lidar_fs_new.nVehicleOriginRows  = lidar_fs->nVehicleOriginRows;
  lidar_fs_new.nVehicleOriginCols  = lidar_fs->nVehicleOriginCols;
  lidar_fs_new.fReserved[9]        = 2;
  auto &points                     = lidar_fs_new.gstPoints;

  for(auto &&p : lidar_fs->gstPoints)
  {
    const float x         = (p.nRowIndex - lidar_fs->nVehicleOriginRows) * lidar_fs->fResolution;
    const float y         = (-p.nColIndex + lidar_fs->nVehicleOriginCols) * lidar_fs->fResolution;
    int16_t     nColIndex = p.nColIndex;

    const float x_push = x;
    float       y_push = y;
    if(x > REMOVE_REGION_BACK && x < REMOVE_REGION_FRONT && y > -2 && y < 2)
    {
      if(y < 0)
      {
        nColIndex += (int16_t)(0.2f / lidar_fs->fResolution);
      }
      else
      {
        nColIndex -= (int16_t)(0.2f / lidar_fs->fResolution);
      }
      y_push = (-nColIndex + lidar_fs->nVehicleOriginCols) * lidar_fs->fResolution;
      if(!polygon.empty())
      {
        if(Utils::pointInPoly(x_push, y_push, polygon) && (FS_LCM_CLASS_TO_CLASS.at(p.nType) == GridLabel::LOCK_BOX)) // -- inner --> lcm --> inner
        {
          continue;
        }
      }
      //      if(x > REMOVE_REGION_BACK && x < REMOVE_REGION_FRONT && y_push > -1.65f && y_push < 1.65f && (grid.grid_label == GridLabel::LOCK_STATION || grid.grid_label == GridLabel::LOCK_BOX))
      //      {
      //        UWARN << "left: " << polygon[0] << " -- " << polygon[1] << "      right: " << polygon[3] << " -- " << polygon[2] << "     true: " << y_push << " " << y_push << " " << CLASS_STRINGS.at(grid.grid_label);
      //      }
    }

    PerceptionGrid point{};
    point.nRowIndex   = p.nRowIndex;
    point.nColIndex   = nColIndex;
    point.fHeight     = p.fHeight;
    point.nId         = p.nId;
    point.nType       = p.nType;
    point.nMoveStatus = p.nMoveStatus;
    points.emplace_back(point);
  }

  if(bev_seg_ptr != nullptr)
  {
    for(auto &&p : bev_seg_ptr->points)
    {
      if(std::fabs(p.y) >= 1.4 && p.x > -8.0f && p.x < 25.6f)
      {
        int16_t x = static_cast<int16_t>(std::round(p.x / lidar_fs_new.fResolution + lidar_fs_new.nVehicleOriginRows));
        int16_t y = static_cast<int16_t>(std::round(-p.y / lidar_fs_new.fResolution + lidar_fs_new.nVehicleOriginCols));
        if(x >= 0 && y >= 0 && x < FS_ROW_OUT && y < FS_COL_OUT && p.label == 0 && p.score > 0.5f)
        {
          PerceptionGrid point{};
          point.nRowIndex   = x;
          point.nColIndex   = y;
          point.fHeight     = 0.0f;
          point.nId         = 0;
          point.nType       = CLASS_TO_FS_LCM_CLASS.at(GridLabel::OCCUPIED);
          point.nMoveStatus = MOTION_TO_LCM_MOTION.at(MotionState::STOPPED);
          points.emplace_back(point);
        }
      }
    }
  }

  lidar_fs_new.nPointNum = lidar_fs_new.gstPoints.size();
  return std::make_unique<PerceptionFreeSpace>(lidar_fs_new);
}

std::unique_ptr<PerceptionFreeSpace> fs::FreespaceManager::tptsLidarFs(const Lidar_FreeSpace_v2 *lidar_fs_ptr, bool is_reverse)
{
  if(lidar_fs_ptr == nullptr)
  {
    return nullptr;
  }
  PerceptionFreeSpace lidar_fs_new{};
  lidar_fs_new.stHeader.nTimeStamp = lidar_fs_ptr->stHeader.nTimeStamp;
  lidar_fs_new.stHeader.nCounter   = lidar_fs_ptr->stHeader.nCounter;
  lidar_fs_new.stHeader.nFrameID   = lidar_fs_ptr->stHeader.nFrameID;
  lidar_fs_new.fResolution         = lidar_fs_ptr->fResolution;
  lidar_fs_new.nVehicleOriginRows  = lidar_fs_ptr->nVehicleOriginRows;
  lidar_fs_new.nVehicleOriginCols  = lidar_fs_ptr->nVehicleOriginCols;
  auto &points                     = lidar_fs_new.gstPoints;

  for(auto &&p : lidar_fs_ptr->gstPoints)
  {
#if FS_CHECK(CFG_ROS2)
    if(FS_PROTO_CLASS_TO_CLASS.at(p.nType) == GridLabel::UNOCCUPIED)
#else
    if(FS_LCM_LIDAR_CLASS_TO_BUILT_IN_CLASS.at(p.nType) == GridLabel::UNOCCUPIED)
#endif
    {
      continue;
    }

    const auto    &grid = tpts_map_[p.nRowIndex][p.nColIndex];
    PerceptionGrid point{};
    if(is_reverse)
    {
      point.nRowIndex = -(p.nRowIndex - lidar_fs_ptr->nVehicleOriginRows) + lidar_fs_ptr->nVehicleOriginRows;
      point.nColIndex = -(p.nColIndex - lidar_fs_ptr->nVehicleOriginCols) + lidar_fs_ptr->nVehicleOriginCols;
    }
    else
    {
      point.nRowIndex = p.nRowIndex;
      point.nColIndex = p.nColIndex;
    }
    point.fHeight     = p.nHight * 0.1;
    point.nId         = grid.id;
    point.nType       = CLASS_TO_FS_LCM_CLASS.at(grid.grid_label);
    point.nMoveStatus = MOTION_TO_LCM_MOTION.at(grid.motion_state);
    points.emplace_back(point);
  }
  lidar_fs_new.nPointNum = lidar_fs_new.gstPoints.size();
  return std::make_unique<PerceptionFreeSpace>(lidar_fs_new);
}

void fs::FreespaceManager::getFreespace(const EgoMotion     *emo_pos_latest,
                                        PerceptionFreeSpace *lidar_fs,
                                        const EgoMotion     *emo_pos_fusion,
                                        float                resolution,
                                        bool                 is_reverse)
{
  auto     Twl = Utils::makeTFrom6Dof(emo_pos_fusion->yaw, 0., 0., emo_pos_fusion->translation.x(), emo_pos_fusion->translation.y(), 0.);
  auto     Twc = Utils::makeTFrom6Dof(emo_pos_latest->yaw, 0., 0., emo_pos_latest->translation.x(), emo_pos_latest->translation.y(), 0.);
  FSMat4x4 Tcl = Utils::inverse(Twc) * Twl;
  if(is_reverse)
  {
    Tcl = T_180_ * Tcl;
  }

  lidar_fs->stHeader.nTimeStamp = emo_pos_latest->timestamp;
  lidar_fs->stHeader.nCounter   = 0;
  lidar_fs->stHeader.nFrameID   = 0;
  lidar_fs->fResolution         = resolution;
  lidar_fs->nVehicleOriginRows  = FS_CAR_ROW_OUT;
  lidar_fs->nVehicleOriginCols  = FS_CAR_COL_OUT;
  auto &points                  = lidar_fs->gstPoints;

  if(resolution - 0.1 < 1e-6)
  {
    // -- 200*500(0.1m) bbox position in 400*1000(0.1m)
    constexpr int min_x_ = -FS_CAR_ROW_OUT;
    constexpr int max_x_ = +FS_ROW_OUT - FS_CAR_ROW_OUT;
    constexpr int min_y_ = -FS_CAR_COL_OUT;
    constexpr int max_y_ = +FS_COL_OUT - FS_CAR_COL_OUT;
    // -- car position: 400*1000(0.1m) --> 200*500(0.1m)
    constexpr int delta_car_row = FS_CAR_ROW_OUT;
    constexpr int delta_car_col = FS_CAR_COL_OUT;

    for(int row = 0; row < GRID_MAP_SIZE; ++row)
    {
      for(int col = 0; col < GRID_MAP_SIZE; ++col)
      {
        auto &grid = fusion_map_.at(row, col);
        if(grid.logit >= 2 || grid.tpts)
        {
          const auto &ego_idx_cur_in = fusion_map_.modIdx2EgoCenterIdxCurrent(GridMap::Index{row, col}); // -- mod grid -->  grid of ego not left down
          if(ego_idx_cur_in.x() < min_x_ || ego_idx_cur_in.x() >= max_x_ || ego_idx_cur_in.y() < min_y_ || ego_idx_cur_in.y() >= max_y_)
          {
            if(grid.tpts)
            {
              grid.tpts = false;
            }
          } // -- select 200*500
          else
          {
            if(grid.tpts)
            {
              grid.tpts             = false;
              const auto    pos     = ego_idx_cur_in * GRID_SCALE;
              const FSVec3f pos_new = (Tcl * HOMO4(FSVec3f(pos.x(), pos.y(), 0.f))).topRows(3); // -- 1:1  enu car coord
              int16_t       x       = static_cast<int16_t>(std::round(pos_new.x() * GRID_SCALE_INV + delta_car_row));
              int16_t       y       = static_cast<int16_t>(std::round(-pos_new.y() * GRID_SCALE_INV + delta_car_col));
              if(x >= 0 && y >= 0 && x < FS_ROW_OUT && y < FS_COL_OUT && grid.grid_label != GridLabel::UNOCCUPIED)
              {
                PerceptionGrid point{};
                point.nRowIndex   = x;
                point.nColIndex   = y;
                point.fHeight     = grid.top;
                point.nId         = grid.id;
                point.nType       = CLASS_TO_FS_LCM_CLASS.at(grid.grid_label);
                point.nMoveStatus = MOTION_TO_LCM_MOTION.at(grid.motion_state);
                points.emplace_back(point);
              }
            } // -- tpts
            else
            {
              if(grid.logit >= 2)
              {
                const auto    pos     = ego_idx_cur_in * GRID_SCALE;
                const FSVec3f pos_new = (Tcl * HOMO4(FSVec3f(pos.x(), pos.y(), 0.f))).topRows(3); // -- 1:1  enu car coord
                int16_t       x       = static_cast<int16_t>(std::round(pos_new.x() * GRID_SCALE_INV + delta_car_row));
                int16_t       y       = static_cast<int16_t>(std::round(-pos_new.y() * GRID_SCALE_INV + delta_car_col));
                if(x >= 0 && y >= 0 && x < FS_ROW_OUT && y < FS_COL_OUT && grid.grid_label != GridLabel::UNOCCUPIED)
                {
                  PerceptionGrid point{};
                  point.nRowIndex   = x;
                  point.nColIndex   = y;
                  point.fHeight     = grid.top;
                  point.nId         = grid.id;
                  point.nType       = CLASS_TO_FS_LCM_CLASS.at(grid.grid_label);
                  point.nMoveStatus = MOTION_TO_LCM_MOTION.at(grid.motion_state);
                  points.emplace_back(point);
                }
              }
            } // -- normal logit
          }
        }
        updateLabelMotionProb(grid);
      }
    } // -- mod grid
    lidar_fs->nPointNum = lidar_fs->gstPoints.size();
  }
  else
  {
    // -- 200*500(0.2m) bbox position in 2000*2000(0.1m)
    constexpr int min_x_ = -FS_CAR_ROW_OUT * 2;
    constexpr int max_x_ = +(FS_ROW_OUT - FS_CAR_ROW_OUT) * 2;
    constexpr int min_y_ = -FS_CAR_COL_OUT * 2;
    constexpr int max_y_ = +(FS_COL_OUT - FS_CAR_COL_OUT) * 2;
    // -- car position: 2000*2000(0.1m) --> 200*500(0.2m)
    constexpr int delta_car_row = FS_CAR_ROW_OUT * 2;
    constexpr int delta_car_col = FS_CAR_COL_OUT * 2;

    for(int row = 0; row < GRID_MAP_SIZE; ++row)
    {
      for(int col = 0; col < GRID_MAP_SIZE; ++col)
      {
        auto &grid = fusion_map_.at(row, col);
        if(grid.logit >= 2 || grid.tpts)
        {
          const auto ego_idx_cur_in = fusion_map_.modIdx2EgoCenterIdxCurrent(GridMap::Index{row, col}); // -- mod grid -->  grid of ego not left down
          if(ego_idx_cur_in.x() < min_x_ || ego_idx_cur_in.x() >= max_x_ || ego_idx_cur_in.y() < min_y_ || ego_idx_cur_in.y() >= max_y_)
          {
            if(grid.tpts)
            {
              grid.tpts = false;
            }
          } // -- select 200*500
          else
          {
            if(grid.tpts)
            {
              grid.tpts             = false;
              const auto    pos     = ego_idx_cur_in * GRID_SCALE;
              const FSVec3f pos_new = (Tcl * HOMO4(FSVec3f(pos.x(), pos.y(), 0.f))).topRows(3); // -- 1:1  enu car coord
              int16_t       x       = static_cast<int16_t>(std::round((pos_new.x() * GRID_SCALE_INV + delta_car_row) * 0.5));
              int16_t       y       = static_cast<int16_t>(std::round((-pos_new.y() * GRID_SCALE_INV + delta_car_col) * 0.5));
              if(x >= 0 && y >= 0 && x < FS_ROW_OUT && y < FS_COL_OUT && grid.grid_label != GridLabel::UNOCCUPIED)
              {
                PerceptionGrid point{};
                point.nRowIndex   = x;
                point.nColIndex   = y;
                point.fHeight     = grid.top;
                point.nId         = grid.id;
                point.nType       = CLASS_TO_FS_LCM_CLASS.at(grid.grid_label);
                point.nMoveStatus = MOTION_TO_LCM_MOTION.at(grid.motion_state);
                points.emplace_back(point);
              }

            } // -- tpts
            else
            {
              if(grid.logit >= 2)
              {
                const auto    pos     = ego_idx_cur_in * GRID_SCALE;
                const FSVec3f pos_new = (Tcl * HOMO4(FSVec3f(pos.x(), pos.y(), 0.f))).topRows(3); // -- 1:1  enu car coord
                int16_t       x       = static_cast<int16_t>(std::round((pos_new.x() * GRID_SCALE_INV + delta_car_row) * 0.5));
                int16_t       y       = static_cast<int16_t>(std::round((-pos_new.y() * GRID_SCALE_INV + delta_car_col) * 0.5));
                if(x >= 0 && y >= 0 && x < FS_ROW_OUT && y < FS_COL_OUT && grid.grid_label != GridLabel::UNOCCUPIED)
                {
                  PerceptionGrid point{};
                  point.nRowIndex   = x;
                  point.nColIndex   = y;
                  point.fHeight     = grid.top;
                  point.nId         = grid.id;
                  point.nType       = CLASS_TO_FS_LCM_CLASS.at(grid.grid_label);
                  point.nMoveStatus = MOTION_TO_LCM_MOTION.at(grid.motion_state);
                  points.emplace_back(point);
                }
              }
            }
          }
        }
        updateLabelMotionProb(grid);
      }
    }
    lidar_fs->nPointNum = lidar_fs->gstPoints.size();
  }
}
#if FS_CHECK(CFG_PUB_CONTOUR)
void fs::FreespaceManager::downSample(std::unique_ptr<PerceptionFreeSpace> &fusion_fs_ptr)
{
  if(fusion_fs_ptr == nullptr)
  {
    return;
  }
  // clang-format off
  UINFO<<"before: " << fusion_fs_ptr->nPointNum;
  memset(raw_map_,0,sizeof(raw_map_));

  std::vector<cv::Point> lut(fusion_fs_ptr->gstPoints.size());
  const auto &map_ego =fusion_fs_ptr->gstPoints;
  for (int i = 0;i<map_ego.size();++i) {
    if(map_ego[i].nRowIndex <0 || map_ego[i].nRowIndex>FS_ROW_OUT || map_ego[i].nColIndex<0 || map_ego[i].nColIndex>FS_COL_OUT){ continue; }
    raw_map_[map_ego[i].nRowIndex][map_ego[i].nColIndex] = 1;
    lut[i] = cv::Point(map_ego[i].nRowIndex,map_ego[i].nColIndex);
  }
  // -- downSample
  int expand = 2;
  for (int col = 0; col < FS_COL_OUT; ++col) {
    for (int row = 0; row < FS_ROW_OUT ; ++row) {
      if (raw_map_[row][col] == 0) { continue; }
      else{
        if(col < expand || row < expand || col >= FS_COL_OUT-expand || row >= FS_ROW_OUT-expand){ continue; }
        else {
           if      (raw_map_[row][col + 1]  == 0 && raw_map_[row][col + 2] == 0) { raw_map_[row][col] = 2; }
           else if (raw_map_[row][col - 1]  == 0 && raw_map_[row][col - 2] == 0) { raw_map_[row][col] = 2; }
           else if (raw_map_[row + 1] [col] == 0 && raw_map_[row + 2][col] == 0) { raw_map_[row][col] = 2; }
           else if (raw_map_[row - 1] [col] == 0 && raw_map_[row - 2][col] == 0) { raw_map_[row][col] = 2; }
           else {}
        }
      }
    }
  }

  int cur = 0;
  int end = lut.size()-1;
  while(cur<end) {
    if(lut[cur].x <0 || lut[cur].x>FS_ROW_OUT || lut[cur].y<0 || lut[cur].y>FS_COL_OUT){ continue; }
    if(raw_map_[lut[cur].x][lut[cur].y] == 0){continue;}
    else{
      if(raw_map_[lut[cur].x][lut[cur].y] == 2){
        ++cur;
      }
      else{
        std::swap(lut[cur],lut[end]);
        fusion_fs_ptr->gstPoints[cur] = fusion_fs_ptr->gstPoints[end];
        fusion_fs_ptr->gstPoints.pop_back();
        --end;
      }
    }
  }
  fusion_fs_ptr->nPointNum = fusion_fs_ptr->gstPoints.size();
  UINFO<<"after: " <<fusion_fs_ptr->nPointNum;
}
#endif// -- CFG_PUB_CONTOUR
#endif // -- CFG_ROS2

#if FS_CHECK(CFG_ROS2)
void fs::FreespaceManager::tptsFreespace(const int64_t fusion_timestamp,
                                         const ObjectStates *vot_object_refine_ptr,
                                         std::unique_ptr<uto::proto::PerceptionFreespace> &fusion_fs_output_ptr,
                                         std::unique_ptr<uto::proto::PerceptionFreespace> &raw_lidar_fs,
                                         bool is_reverse)
#else
void fs::FreespaceManager::tptsFreespace(const int64_t                         fusion_timestamp,
                                         const ObjectStates                   *vot_object_refine_ptr,
                                         std::unique_ptr<PerceptionFreeSpace> &fusion_fs_output_ptr,
                                         float                                 resolution,
                                         bool                                  is_reverse,
                                         const Lidar_FreeSpace_v2             *raw_lidar_fs)
#endif // -- CFG_ROS2
{
  fusion_timestamp_ = fusion_timestamp;
  auto start = std::chrono::steady_clock::now();

  assignTptsMapVotInfo(vot_object_refine_ptr);
  fusion_fs_output_ptr = tptsLidarFs(raw_lidar_fs, is_reverse);
#if FS_CHECK(CFG_PUB_CONTOUR)
  auto ds = std::chrono::steady_clock::now();
  downSample(fusion_fs_output_ptr);
  UINFO << "ds:" << std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - ds).count();
#endif // --CFG_PUB_CONTOUR

#if FS_CHECK(CFG_VIS_ENABLE)
#if FS_CHECK(!CFG_DEBUG_PERCEPTION_FREESPACES)
#if FS_CHECK(CFG_ROS2)
  Vis::drawSelfReceivePointCloud(*fusion_fs_output_ptr);
#else
  Vis::drawSelfReceivePointCloud(*fusion_fs_output_ptr);
#endif // -- CFG_ROS2
#endif
#if FS_CHECK(CFG_ROS2)
drawOther(vot_object_refine_ptr);
#else
drawOther(vot_object_refine_ptr, nullptr);
#endif // -- CFG_ROS2
#endif // -- CFG_VIS_ENABLE
  const float duration = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
  UINFO << "tpts process  time:  " << duration << " ms";
  UINFO << "=================================================================================================================================";
}
