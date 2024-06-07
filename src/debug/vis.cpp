#include "vis.h"

// #define MARGIN    45 * CV_PI / 180.
std::unordered_map<int, bool>                                mapSavedFlags;
std::unordered_map<std::string, std::pair<cv::Mat, cv::Mat>> mapCache;
cv::Mat                                                      fs::Vis::m_BEV_View(BEV_HEIGHT, BEV_WIDTH, CV_8UC3);
cv::Mat                                                      fs::Vis::m_Remap_View(216, 384, CV_8UC3);
cv::Mat                                                      fs::Vis::m_Test_Zylinder_View(216, 384, CV_8UC3);
std::array<cv::Mat, 6>                                       fs::Vis::cy_map_img_vis = {
  cv::Mat(320, 240, CV_8UC3),
  cv::Mat(320, 240, CV_8UC3),
  cv::Mat(320, 240, CV_8UC3),
  cv::Mat(320, 240, CV_8UC3),
  cv::Mat(320, 240, CV_8UC3),
  cv::Mat(320, 240, CV_8UC3)};

#if !FS_CHECK(CFG_ROS2)
cv::Mat fs::Vis::m_FW_View(216, 384, CV_8UC3);
cv::Mat fs::Vis::m_RW_View(216, 384, CV_8UC3);
cv::Mat fs::Vis::m_FN_View(216, 384, CV_8UC3);
cv::Mat fs::Vis::m_RN_View(216, 384, CV_8UC3);
cv::Mat fs::Vis::m_FL_View(216, 384, CV_8UC3);
cv::Mat fs::Vis::m_FR_View(216, 384, CV_8UC3);
cv::Mat fs::Vis::m_RL_View(216, 384, CV_8UC3);
cv::Mat fs::Vis::m_RR_View(216, 384, CV_8UC3);
#endif

std::array<cv::Mat, (uint8_t)fs::SensorId::MAX_SENSOR_NUM> fs::Vis::map_img_vis = {
#if FS_CHECK(CFG_USE_SVC)
  cv::Mat(320, 240, CV_8UC3),
  cv::Mat(320, 240, CV_8UC3),
  cv::Mat(320, 240, CV_8UC3),
  cv::Mat(320, 240, CV_8UC3),
  cv::Mat(320, 240, CV_8UC3),
  cv::Mat(320, 240, CV_8UC3),
#endif
  cv::Mat(216, 384, CV_8UC3),
  cv::Mat(216, 384, CV_8UC3),
  cv::Mat(960, 1280, CV_8UC3),
  cv::Mat(216, 384, CV_8UC3),
  cv::Mat(216, 384, CV_8UC3),
  cv::Mat(216, 384, CV_8UC3),
  cv::Mat(216, 384, CV_8UC3),
  cv::Mat(216, 384, CV_8UC3)};
#if !FS_CHECK(CFG_ROS2)
sensor_msgs::msg::PointCloud fs::Vis::m_egoBboxPointCloud{};
#endif
visualization_msgs::msg::MarkerArray fs::Vis::m_markers{};
visualization_msgs::msg::MarkerArray fs::Vis::m_blind_markers{};

sensor_msgs::msg::PointCloud fs::Vis::m_gridPointCloud{};
sensor_msgs::msg::PointCloud fs::Vis::m_outputPointCloud{};
sensor_msgs::msg::PointCloud fs::Vis::m_lidarPointCloud{};
sensor_msgs::msg::PointCloud fs::Vis::m_roadPointCloud{};
sensor_msgs::msg::PointCloud fs::Vis::m_bevPointCloud{};
#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
sensor_msgs::msg::PointCloud2 fs::Vis::m_lidarPointCloud2{};
sensor_msgs::msg::PointCloud2 fs::Vis::m_roadPointCloud2{};
#endif
sensor_msgs::msg::PointCloud fs::Vis::m_blindPointCloud{};
sensor_msgs::msg::PointCloud fs::Vis::m_colorTablePointCloud{};
sensor_msgs::msg::PointCloud fs::Vis::m_PolygonPointCloud{};
sensor_msgs::msg::PointCloud fs::Vis::m_lidarPointCloudHistory{};
sensor_msgs::msg::PointCloud fs::Vis::m_tptsBboxPointCloud{};
sensor_msgs::msg::PointCloud fs::Vis::m_outBboxPointCloud{};

sensor_msgs::msg::PointCloud fs::Vis::cld_OTHER_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_PEDESTRIAN_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_CYCLIST_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_VEHICLE_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_RTG_QC_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_LOCK_BOX_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_CONE_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_TREE_BRANCH_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_AIV_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_CONTAINER_SHELF_{};

std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud> fs::Vis::map_cld_built_in_ = {{fs::GridLabel::OTHER, fs::Vis::cld_OTHER_},
                                                                                          {fs::GridLabel::PEDESTRIAN, fs::Vis::cld_PEDESTRIAN_},
                                                                                          {fs::GridLabel::CYCLIST, fs::Vis::cld_CYCLIST_},
                                                                                          {fs::GridLabel::VEHICLE, fs::Vis::cld_VEHICLE_},
                                                                                          {fs::GridLabel::RTG_QC, fs::Vis::cld_RTG_QC_},
                                                                                          {fs::GridLabel::LOCK_BOX, fs::Vis::cld_LOCK_BOX_},
                                                                                          {fs::GridLabel::CONE, fs::Vis::cld_CONE_},
                                                                                          {fs::GridLabel::TREE_BRANCH, fs::Vis::cld_TREE_BRANCH_},
                                                                                          {fs::GridLabel::AIV, fs::Vis::cld_AIV_}};

#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
sensor_msgs::msg::PointCloud2 fs::Vis::cld_OTHER2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_PEDESTRIAN2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_CYCLIST2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_VEHICLE2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_RTG_QC2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_LOCK_BOX2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_CONE2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_AIV2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_TREE_BRANCH2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_CONTAINER_SHELF2_{};

std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud2> fs::Vis::map_cld_built_in2_ = {{fs::GridLabel::OTHER, fs::Vis::cld_OTHER2_},
                                                                                            {fs::GridLabel::PEDESTRIAN, fs::Vis::cld_PEDESTRIAN2_},
                                                                                            {fs::GridLabel::CYCLIST, fs::Vis::cld_CYCLIST2_},
                                                                                            {fs::GridLabel::VEHICLE, fs::Vis::cld_VEHICLE2_},
                                                                                            {fs::GridLabel::RTG_QC, fs::Vis::cld_RTG_QC2_},
                                                                                            {fs::GridLabel::LOCK_BOX, fs::Vis::cld_LOCK_BOX2_},
                                                                                            {fs::GridLabel::CONE, fs::Vis::cld_CONE2_},
                                                                                            {fs::GridLabel::TREE_BRANCH, fs::Vis::cld_TREE_BRANCH2_},
                                                                                            {fs::GridLabel::AIV, fs::Vis::cld_AIV2_}};

//self receive
sensor_msgs::msg::PointCloud2 fs::Vis::cld_self_receive_OTHER2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_self_receive_PEDESTRIAN2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_self_receive_CYCLIST2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_self_receive_VEHICLE2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_self_receive_RTG_QC2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_self_receive_LOCK_BOX2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_self_receive_CONE2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_self_receive_AIV2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_self_receive_TREE_BRANCH2_{};
sensor_msgs::msg::PointCloud2 fs::Vis::cld_self_receive_CONTAINER_SHELF2_{};

std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud2> fs::Vis::map_cld_self_receive2_ = {{fs::GridLabel::OTHER, fs::Vis::cld_self_receive_OTHER2_},
                                                                                                {fs::GridLabel::PEDESTRIAN, fs::Vis::cld_self_receive_PEDESTRIAN2_},
                                                                                                {fs::GridLabel::CYCLIST, fs::Vis::cld_self_receive_CYCLIST2_},
                                                                                                {fs::GridLabel::VEHICLE, fs::Vis::cld_self_receive_VEHICLE2_},
                                                                                                {fs::GridLabel::RTG_QC, fs::Vis::cld_self_receive_RTG_QC2_},
                                                                                                {fs::GridLabel::LOCK_BOX, fs::Vis::cld_self_receive_LOCK_BOX2_},
                                                                                                {fs::GridLabel::CONE, fs::Vis::cld_self_receive_CONE2_},
                                                                                                {fs::GridLabel::TREE_BRANCH, fs::Vis::cld_self_receive_TREE_BRANCH2_},
                                                                                                {fs::GridLabel::AIV, fs::Vis::cld_self_receive_AIV2_}};
#endif
sensor_msgs::msg::PointCloud fs::Vis::cld_self_receive_OTHER_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_self_receive_PEDESTRIAN_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_self_receive_CYCLIST_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_self_receive_VEHICLE_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_self_receive_RTG_QC_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_self_receive_LOCK_BOX_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_self_receive_CONE_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_self_receive_TREE_BRANCH_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_self_receive_AIV_{};
sensor_msgs::msg::PointCloud fs::Vis::cld_self_receive_CONTAINER_SHELF_{};

std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud> fs::Vis::map_cld_self_receive_ = {{fs::GridLabel::OTHER, fs::Vis::cld_self_receive_OTHER_},
                                                                                              {fs::GridLabel::PEDESTRIAN, fs::Vis::cld_self_receive_PEDESTRIAN_},
                                                                                              {fs::GridLabel::CYCLIST, fs::Vis::cld_self_receive_CYCLIST_},
                                                                                              {fs::GridLabel::VEHICLE, fs::Vis::cld_self_receive_VEHICLE_},
                                                                                              {fs::GridLabel::RTG_QC, fs::Vis::cld_self_receive_RTG_QC_},
                                                                                              {fs::GridLabel::LOCK_BOX, fs::Vis::cld_self_receive_LOCK_BOX_},
                                                                                              {fs::GridLabel::CONE, fs::Vis::cld_self_receive_CONE_},
                                                                                              {fs::GridLabel::TREE_BRANCH, fs::Vis::cld_self_receive_TREE_BRANCH_},
                                                                                              {fs::GridLabel::AIV, fs::Vis::cld_self_receive_AIV_}};

#if FS_CHECK(CFG_ROS2)

void fs::Vis::prepare(const int64_t                                                                                      timestamp,
                      std::array<std::unique_ptr<sensor_msgs::msg::CompressedImage>, (uint8_t)SensorId::MAX_SENSOR_NUM> &map_img,
                      const std::array<CamFS, (uint8_t)SensorId::MAX_SENSOR_NUM>                                        &map_fs,
                      const ObjectStates                                                                                *vot_objects_ptr,
                      const LidarFS                                                                                     *lidar_fs_ptr,
                      const std::vector<std::vector<fs::LineCoeff>>                                                     &coeff_lanes)
{
  clearMarkers();
  drawLidarPointCloud(lidar_fs_ptr);
#if FS_CHECK(CFG_USE_ROAD_MODEL)
  drawRoadModelPointCloud(road_model_ptr);
#endif
  drawVotObject(vot_objects_ptr, "raw", 0);
  drawLines(coeff_lanes);
  drawTrajectory();
  drawEgo();
  drawRemap();

  if(CameraManager::instance().isAllCameraValid())
  {
#if FS_CHECK(CFG_USE_SVC)
    decodeImg(map_img, SensorId::CAMERA_SFW, map_img_vis[(uint8_t)SensorId::CAMERA_SFW]);
    decodeImg(map_img, SensorId::CAMERA_SFL, map_img_vis[(uint8_t)SensorId::CAMERA_SFL]);
    decodeImg(map_img, SensorId::CAMERA_SFR, map_img_vis[(uint8_t)SensorId::CAMERA_SFR]);
    decodeImg(map_img, SensorId::CAMERA_SRL, map_img_vis[(uint8_t)SensorId::CAMERA_SRL]);
    decodeImg(map_img, SensorId::CAMERA_SRR, map_img_vis[(uint8_t)SensorId::CAMERA_SRR]);
    decodeImg(map_img, SensorId::CAMERA_SRW, map_img_vis[(uint8_t)SensorId::CAMERA_SRW]);
#endif
    decodeImg(map_img, SensorId::CAMERA_FW, map_img_vis[(uint8_t)SensorId::CAMERA_FW]);
    decodeImg(map_img, SensorId::CAMERA_FN, map_img_vis[(uint8_t)SensorId::CAMERA_FN]);
    decodeImg(map_img, SensorId::CAMERA_FL, map_img_vis[(uint8_t)SensorId::CAMERA_FL]);
    decodeImg(map_img, SensorId::CAMERA_FR, map_img_vis[(uint8_t)SensorId::CAMERA_FR]);
    decodeImg(map_img, SensorId::CAMERA_RW, map_img_vis[(uint8_t)SensorId::CAMERA_RW]);
    decodeImg(map_img, SensorId::CAMERA_RN, map_img_vis[(uint8_t)SensorId::CAMERA_RN]);
    decodeImg(map_img, SensorId::CAMERA_RL, map_img_vis[(uint8_t)SensorId::CAMERA_RL]);
    decodeImg(map_img, SensorId::CAMERA_RR, map_img_vis[(uint8_t)SensorId::CAMERA_RR]);

    //    auto start = std::chrono::steady_clock::now();

    for(size_t i = 0; i < map_fs.size(); ++i)
    {
      if(map_img[i] != nullptr)
      {
        const Camera *const camera = CameraManager::instance().getCamera(SensorId(i)).get();
        cv::Mat             img    = cv::imdecode(map_img[i]->data, cv::IMREAD_COLOR);
        cv::resize(img, img, cv::Size{VIS_IMAGE_VIEW_SCALING * camera->getImageWidth(), VIS_IMAGE_VIEW_SCALING * camera->getImageHeight()});

        cv::Mat     cylindrical_image;
        std::string filename = "map_" + std::to_string(i) + ".yml";           // 为每个相机生成不同的文件名
        std::string id       = "camera_" + std::to_string(i);                 // 使用相机ID作为键
        bool        saveMap  = !mapSavedFlags[static_cast<int>(SensorId(i))]; // 检查是否需要保存映射表

        fishImageToCylindrical(img, cylindrical_image, camera, id, saveMap);

        getRemoveNoiseFreespace3(img, cylindrical_image, lidar_fs_ptr, i, map_fs[i].points_ptr.get(), camera, map_fs[i].T_cam2lidar);

        drawTimestamp(timestamp, img);
        map_img_vis[i] = img.clone();
        if(saveMap)
        {
          mapSavedFlags[static_cast<int>(SensorId(i))] = true; // 更新保存标志
        }
        drawTimestamp(timestamp, cylindrical_image);

        switch(SensorId(i))
        {
        case SensorId::CAMERA_SFW:
          cy_map_img_vis[0] = cylindrical_image.clone();
          // cv::resize(cylindrical_image, cy_map_img_vis[0], cv::Size{1.0 * camera->getImageWidth(), 1.0 * camera->getImageHeight()});
          break;
        case SensorId::CAMERA_SFL:
          cy_map_img_vis[1] = cylindrical_image.clone();
          break;
        case SensorId::CAMERA_SFR:
          cy_map_img_vis[2] = cylindrical_image.clone();
          break;
        case SensorId::CAMERA_SRL:
          cy_map_img_vis[3] = cylindrical_image.clone();
          break;
        case SensorId::CAMERA_SRR:
          cy_map_img_vis[4] = cylindrical_image.clone();
          break;
        case SensorId::CAMERA_SRW:
          cy_map_img_vis[5] = cylindrical_image.clone();
          break;
        default:
          break;
        }
      }
    }
    //    const float duration = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
    //    UINFO << " 测试  time:  " << duration << " ms";
  }
}

void fs::Vis::fishImageToCylindrical(cv::Mat &fisheye_image, cv::Mat &cylindrical_image, const Camera *const camera, const std::string &id, bool saveMap)
{
  const int width  = fisheye_image.cols;
  const int height = fisheye_image.rows;
  // const float width_pi = width / CV_PI;
  // const float height_pi_margin = height / (CV_PI - MARGIN);
  // const Eigen::Matrix3f rotMat = camera->getRotMat();
  // const auto K = camera->toK();
  // const auto D = camera->toD();

  cylindrical_image.create(height, width, fisheye_image.type());
  cv::Mat map1, map2;
  if(saveMap)
  {
    static_cast<Fisheye *>(const_cast<Camera *>(camera))->convert2Map();
    auto myMap = static_cast<Fisheye *>(const_cast<Camera *>(camera))->getMap();
  }
  else
  {
    auto myMap = static_cast<Fisheye *>(const_cast<Camera *>(camera))->getMap();
    map1       = myMap.first;
    map2       = myMap.second;
    if(map1.size().area() > 0)
    {
      cv::remap(fisheye_image, cylindrical_image, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }
  }
}

// 差值函数
std::vector<cv::Point2f> interpolatePoints(const std::vector<cv::Point2f> &inputPoints)
{
    std::vector<cv::Point2f> interpolatedPoints;

    for (size_t i = 0; i < inputPoints.size() - 1; ++i)
    {
        cv::Point2f currentPoint = inputPoints[i];
        cv::Point2f nextPoint = inputPoints[i + 1];

        int numInterpolatedPoints = static_cast<int>(nextPoint.x - currentPoint.x);
        for (int k = 0; k < numInterpolatedPoints; ++k)
        {
            float interpolationFactor = static_cast<float>(k) / numInterpolatedPoints;
            cv::Point2f interpolatedPoint = currentPoint + interpolationFactor * (nextPoint - currentPoint);
            interpolatedPoints.push_back(interpolatedPoint);
        }
    }

    return interpolatedPoints;
}

void fs::Vis::getRemoveNoiseFreespace3(cv::Mat &img, cv::Mat &cy_img, const LidarFS *lidar_fs_ptr, const int &idx, const std::vector<CamFSPoint> *camera_freespace, const Camera *const camera, const FSMat4x4 &Tcl)
{
  //auto                   camera_fs_shrink = getShrinkCameraFreespace(camera_fs);
  if(lidar_fs_ptr == nullptr || camera_freespace == nullptr)
  {
    return;
  }

  std::vector<cv::Point3f> objectPoints(1);
  for(auto &&gstPoint : lidar_fs_ptr->points)
  {
    if(std::find(SENSOR_ID_TO_AREA.at(SensorId(idx)).begin(), SENSOR_ID_TO_AREA.at(SensorId(idx)).end(), gstPoint.area) == SENSOR_ID_TO_AREA.at(SensorId(idx)).end())
    {
      continue;
    }
    const float sync_x = Tcl(0, 0) * gstPoint.x + Tcl(0, 1) * gstPoint.y + Tcl(0, 3);
    const float sync_y = Tcl(1, 0) * gstPoint.x + Tcl(1, 1) * gstPoint.y + Tcl(1, 3);

    auto            pc_bottom = camera->transformEgo2Cam(FSVec3f(sync_x, sync_y, gstPoint.bottom));
    auto            px_bottom = camera->transformCam2Img(pc_bottom);
    Eigen::Vector3f pc_bottom_t;
    pc_bottom_t << pc_bottom.x(), pc_bottom.y(), pc_bottom.z();

    Eigen::Vector3f pc_ground = camera->transformEgo2Cam(FSVec3f(sync_x, sync_y, 0));
    // Eigen::Vector3f pc_ground = camera->transformEgo2Cam(FSVec3f(10, 10, 10));
    if(pc_ground[2] < 0)
    {
      continue;
    }
    // auto px_ground = camera->transformCam2Img(pc_ground);
    objectPoints[0] = cv::Point3f(pc_ground[0], pc_ground[1], pc_ground[2]);
    std::vector<cv::Point2f> imagePoints;
    cv::fisheye::projectPoints(objectPoints, imagePoints, cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), camera->toK(), camera->toD());

    cv::Point2f pt_fish = imagePoints[0];
    if(gstPoint.lidar_label == GridLabel::TREE_BRANCH)
    {
      cv::circle(img, imagePoints[0] * VIS_IMAGE_VIEW_SCALING, 10.0, cv::Scalar(0, 182, 0), -1); //淡紫色
    }

    // ------- cy ---ground-------------------------------------------------------
    Eigen::Vector3f ray   = camera->getRotMat().inverse() * pc_ground;
    cv::Point2f     pt_cy = static_cast<Fisheye *>(const_cast<Camera *>(camera))->projectToCylindrical(ray, cy_img.cols, cy_img.rows);
  //  cv::circle(cy_img, pt_cy  , 10.0,  cv::Scalar(255, 182, 193), -1);//淡紫色


    // ------- cy ---bottom-------------------------------------------------------
    Eigen::Vector3f ray_b   = camera->getRotMat().inverse() * pc_bottom_t;
    cv::Point2f     pt_cy_b = static_cast<Fisheye *>(const_cast<Camera *>(camera))->projectToCylindrical(ray_b, cy_img.cols, cy_img.rows);
    // cv::circle(cy_img, pt_cy_b  , 1.0,  cv::Scalar(255, 182, 0), -1);//浅蓝色

    //lidar -to- vis
    if((*camera_freespace)[std::round(imagePoints[0].x)].y > camera->getImageHeight())
    {
      continue;
    }
    cv::Point   pt_fish_det(imagePoints[0].x, (*camera_freespace)[std::round(imagePoints[0].x)].y);                                                //找出激光检测对应的视觉的检测点

    cv::circle(img, imagePoints[0] * VIS_IMAGE_VIEW_SCALING, 1.0, cv::Scalar(255, 182, 193), -1); //淡紫色
    cv::circle(img, pt_fish_det * VIS_IMAGE_VIEW_SCALING, 1.0, cv::Scalar(128, 0, 0), -1); //淡紫色
    cv::line(img, imagePoints[0] * VIS_IMAGE_VIEW_SCALING, pt_fish_det * VIS_IMAGE_VIEW_SCALING, cv::Scalar(255, 182, 0), 1.0);

    cv::Point2f pt_cy_vis = static_cast<Fisheye *>(const_cast<Camera *>(camera))->unprojectFisheye(pt_fish_det, camera, cy_img.cols, cy_img.rows); // 将对应的鱼眼检测点转为柱面
    // cv::circle(cy_img, pt_cy_vis, 1.0, cv::Scalar(255, 182, 0), -1);   
    cv::circle(cy_img, pt_cy * VIS_IMAGE_VIEW_SCALING, 1.0, cv::Scalar(255, 182, 193), -1); //淡紫色
    cv::circle(cy_img, pt_cy_vis * VIS_IMAGE_VIEW_SCALING, 1.0, cv::Scalar(128, 0, 0), -1); //淡紫色                                                                             //浅蓝色
    cv::line(cy_img, pt_cy, pt_cy_vis, cv::Scalar(255, 182, 0), 1.0);

    // if((pt_fish_det.y - imagePoints[0].y) > 100 ){
    //   UINFO << " 激光鱼眼 " <<  imagePoints[0].x << " " << imagePoints[0].y << " 激光fs柱面 " << pt_cy.x << " " << pt_cy.y;
    //   UINFO <<  "鱼眼找到的视觉 " << pt_fish_det.x << " " << pt_fish_det.y << " 视觉柱面 " << pt_cy_vis.x << " " << pt_cy_vis.y;
    // }

    auto roi_fish = static_cast<Fisheye *>(const_cast<Camera *>(camera))->fov2Pixel(90.0 * M_PI / 180, 60.0 * M_PI / 180);
    cv::rectangle(img, cv::Point(roi_fish[0] * VIS_IMAGE_VIEW_SCALING, roi_fish[1] * VIS_IMAGE_VIEW_SCALING), cv::Point(roi_fish[2] * VIS_IMAGE_VIEW_SCALING, roi_fish[3] * VIS_IMAGE_VIEW_SCALING), cv::Scalar(255, 182, 193), 2);
    cv::rectangle(cy_img, cv::Point(roi_fish[0] * VIS_IMAGE_VIEW_SCALING, roi_fish[1] * VIS_IMAGE_VIEW_SCALING), cv::Point(roi_fish[2] * VIS_IMAGE_VIEW_SCALING, roi_fish[3] * VIS_IMAGE_VIEW_SCALING), cv::Scalar(255, 182, 193), 2);

    Eigen::Vector2f lidar_px_ground;
    lidar_px_ground << imagePoints[0].x / 4.0, imagePoints[0].y / 4.0;

    if(lidar_px_ground.x() >= 0 && lidar_px_ground.x() < camera->getImageWidth() / 4.0)
    {
      if(lidar_px_ground.y() > camera->getImageHeight() / 4.0)
      {
        // cv::line(cy_img, cv::Point2f(pt_cy.x, pt_cy.y), cv::Point2f(pt_cy_b.x, pt_cy_b.y), cv::Scalar(0, 0, 128), 1.0);
      } // -- pixel of lidar-fs ground point out of 2160
      else if((*camera_freespace)[std::round(imagePoints[0].x)].type == CameraFSType::INVALID)
      {
        // cv::circle(cy_img, pt_cy, 0.5,  cv::Scalar(0, 0, 128), -1);
        // cv::line(cy_img, cv::Point2f(pt_cy.x, pt_cy.y), cv::Point2f(pt_cy_b.x, pt_cy_b.y), cv::Scalar(0, 0, 128), 1.0);
      } // -- INVALID == wipers
      // else if(pt_cy_vis.y > pt_cy.y)
      else if(pt_cy_vis.y + OFFSET_FISHFS > pt_cy.y)
      {
        // cv::circle(cy_img, pt_cy, 1.0,cv::Scalar(0, 0, 128), -1);
        // cv::line(cy_img, cv::Point2f(pt_cy.x, pt_cy.y), cv::Point2f(pt_cy_b.x, pt_cy_b.y), cv::Scalar(0, 0, 100), 1.0); //红色
      }                                                                                                                 // --extrinsic parameters fluctuation
      else
      {
        // cv::circle(cy_img, pt_cy, 1.0,cv::Scalar(128, 0, 0), -1);
        // cv::line(cy_img, cv::Point2f(pt_cy.x, pt_cy.y), cv::Point2f(pt_cy_b.x, pt_cy_b.y), cv::Scalar(128, 0, 0), 1.0);

      } // -- noise
    }
  } // -- iteration for each freespace point
#if FS_CHECK(CFG_VIS_ENABLE)
  if(camera_freespace != nullptr)
  {
    //      cv::line(img, cv::Point2f(0, CAMERA_FRONT_CENTER_HEIGHT - 30) ,
    //                cv::Point2f(CAMERA_FRONT_CENTER_WIDTH - 1, CAMERA_FRONT_CENTER_HEIGHT - 30), cv::Scalar(0, 0, 0), 2);
    std::vector<cv::Point2f> undistortedPoints;
    for(size_t j = 0; j < (*camera_freespace).size() - 1; ++j)
    {
      cv::Point pt1(j, (*camera_freespace)[j].y);

      if((*camera_freespace)[j].y > camera->getImageHeight())
      {
        continue;
      }

      cv::circle(img, pt1 * VIS_IMAGE_VIEW_SCALING, 0.5, getFreespaceColor((*camera_freespace)[j].type), 1);
      cv::Point2f pt_cy1 = static_cast<Fisheye *>(const_cast<Camera *>(camera))->unprojectFisheye(pt1, camera, cy_img.cols, cy_img.rows);

// // -----------------------------------------------------------------------------------------------------------------------------------------
//       // 差值处理
//       std::vector<cv::Point2f> interpolatedPoints = interpolatePoints({pt1});
//       undistortedPoints.insert(undistortedPoints.end(), interpolatedPoints.begin(), interpolatedPoints.end());

//       // 显示差值后的点
//       for (const auto &point : interpolatedPoints)
//       {
//           cv::circle(img, point * VIS_IMAGE_VIEW_SCALING, 5, getFreespaceColor((*camera_freespace)[j].type), 1);

//           cv::Point2f pt_cy = static_cast<Fisheye *>(const_cast<Camera *>(camera))->unprojectFisheye(point, camera, cy_img.cols, cy_img.rows);
//           cv::circle(cy_img, point, 5, getFreespaceColor((*camera_freespace)[j].type), -1);
//       }
// // -----------------------------------------------------------------------------------------------------------------------------------------



      // UINFO << " j " << j << " 视觉鱼眼fspt1 " << pt1.x << " " << pt1.y << " 视觉柱面pt_cy1 " << pt_cy1.x << " " << pt_cy1.y;
      cv::circle(cy_img, pt_cy1, 0.5, getFreespaceColor((*camera_freespace)[j].type), -1);
    }
    int       fontFace  = cv::FONT_HERSHEY_SIMPLEX;
    double    fontScale = 0.3;
    int       v_step    = 10;
    int       start     = 5;
    int       wid       = start + 20;
    cv::Point p_inva(wid, 1 * v_step), p_inva1(start, 0 * v_step + 0.7 * v_step),
              p_curb(wid, 3 * v_step), p_curb1(start, 2 * v_step + 0.7 * v_step),
              p_vpcy(wid, 5 * v_step), p_vpcy1(start, 4 * v_step + 0.7 * v_step),
              p_cone(wid, 7 * v_step), p_cone1(start, 6 * v_step + 0.7 * v_step),
              p_barr(wid, 9 * v_step), p_barr1(start, 8 * v_step + 0.7 * v_step),
              p_othe(wid, 11 * v_step), p_othe1(start, 10 * v_step + 0.7 * v_step),
              p_wipper(wid, 13 * v_step), p_wipper1(start, 12 * v_step + 0.7 * v_step),
              p_rain_fog(wid, 15 * v_step), p_rain_fog1(start, 14 * v_step + 0.7 * v_step),
              p_dust(wid, 17 * v_step), p_dust1(start, 16 * v_step + 0.7 * v_step),
              p_uninit(wid, 19 * v_step), p_uninit1(start, 18 * v_step + 0.7 * v_step);
    cv::putText(img, "Invalid               ", p_inva, fontFace, fontScale, getFreespaceColor(CameraFSType::INVALID), 1);
    cv::putText(img, "Curb                  ", p_curb, fontFace, fontScale, getFreespaceColor(CameraFSType::CURB), 1);
    cv::putText(img, "People/Vehicle/Cycling", p_vpcy, fontFace, fontScale, getFreespaceColor(CameraFSType::VPR), 1);
    cv::putText(img, "Cone                  ", p_cone, fontFace, fontScale, getFreespaceColor(CameraFSType::CONE), 1);
    cv::putText(img, "Barricade             ", p_barr, fontFace, fontScale, getFreespaceColor(CameraFSType::BARRICADE), 1);
    cv::putText(img, "other                 ", p_othe, fontFace, fontScale, getFreespaceColor(CameraFSType::OTHER), 1);
    cv::putText(img, "wipper                ", p_wipper, fontFace, fontScale, getFreespaceColor(CameraFSType::WIPPER), 1);
    cv::putText(img, "rain_fog              ", p_rain_fog, fontFace, fontScale, getFreespaceColor(CameraFSType::RAIN_FOG), 1);
    cv::putText(img, "dust                  ", p_dust, fontFace, fontScale, getFreespaceColor(CameraFSType::DUST), 1);
    cv::putText(img, "uninit                ", p_uninit, fontFace, fontScale, getFreespaceColor(CameraFSType::UNINIT), 1);

    cv::circle(img, p_inva1, 0.1, getFreespaceColor(CameraFSType::INVALID), -1);
    cv::circle(img, p_curb1, 0.1, getFreespaceColor(CameraFSType::CURB), -1);
    cv::circle(img, p_vpcy1, 0.1, getFreespaceColor(CameraFSType::VPR), -1);
    cv::circle(img, p_cone1, 0.1, getFreespaceColor(CameraFSType::CONE), -1);
    cv::circle(img, p_barr1, 0.1, getFreespaceColor(CameraFSType::BARRICADE), -1);
    cv::circle(img, p_othe1, 0.1, getFreespaceColor(CameraFSType::OTHER), -1);
    cv::circle(img, p_wipper1, 0.1, getFreespaceColor(CameraFSType::WIPPER), -1);
    cv::circle(img, p_rain_fog1, 0.1, getFreespaceColor(CameraFSType::RAIN_FOG), -1);
    cv::circle(img, p_dust1, 0.1, getFreespaceColor(CameraFSType::DUST), -1);
    cv::circle(img, p_uninit1, 0.1, getFreespaceColor(CameraFSType::UNINIT), -1);
    //
    cv::putText(cy_img, "Invalid               ", p_inva, fontFace, fontScale, getFreespaceColor(CameraFSType::INVALID), 1);
    cv::putText(cy_img, "Curb                  ", p_curb, fontFace, fontScale, getFreespaceColor(CameraFSType::CURB), 1);
    cv::putText(cy_img, "People/Vehicle/Cycling", p_vpcy, fontFace, fontScale, getFreespaceColor(CameraFSType::VPR), 1);
    cv::putText(cy_img, "Cone                  ", p_cone, fontFace, fontScale, getFreespaceColor(CameraFSType::CONE), 1);
    cv::putText(cy_img, "Barricade             ", p_barr, fontFace, fontScale, getFreespaceColor(CameraFSType::BARRICADE), 1);
    cv::putText(cy_img, "other                 ", p_othe, fontFace, fontScale, getFreespaceColor(CameraFSType::OTHER), 1);
    cv::putText(cy_img, "wipper                ", p_wipper, fontFace, fontScale, getFreespaceColor(CameraFSType::WIPPER), 1);
    cv::putText(cy_img, "rain_fog              ", p_rain_fog, fontFace, fontScale, getFreespaceColor(CameraFSType::RAIN_FOG), 1);
    cv::putText(cy_img, "dust                  ", p_dust, fontFace, fontScale, getFreespaceColor(CameraFSType::DUST), 1);
    cv::putText(cy_img, "uninit                ", p_uninit, fontFace, fontScale, getFreespaceColor(CameraFSType::UNINIT), 1);
    cv::circle(cy_img, p_inva1, 0.1, getFreespaceColor(CameraFSType::INVALID), -1);
    cv::circle(cy_img, p_curb1, 0.1, getFreespaceColor(CameraFSType::CURB), -1);
    cv::circle(cy_img, p_vpcy1, 0.1, getFreespaceColor(CameraFSType::VPR), -1);
    cv::circle(cy_img, p_cone1, 0.1, getFreespaceColor(CameraFSType::CONE), -1);
    cv::circle(cy_img, p_barr1, 0.1, getFreespaceColor(CameraFSType::BARRICADE), -1);
    cv::circle(cy_img, p_othe1, 0.1, getFreespaceColor(CameraFSType::OTHER), -1);
    cv::circle(cy_img, p_wipper1, 0.1, getFreespaceColor(CameraFSType::WIPPER), -1);
    cv::circle(cy_img, p_rain_fog1, 0.1, getFreespaceColor(CameraFSType::RAIN_FOG), -1);
    cv::circle(cy_img, p_dust1, 0.1, getFreespaceColor(CameraFSType::DUST), -1);
    cv::circle(cy_img, p_uninit1, 0.1, getFreespaceColor(CameraFSType::UNINIT), -1);
  }
#endif
}

#else
void fs::Vis::prepare(const int64_t                   timestamp,
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
                      const LOGITECH_IMAGE           *image_rr_Ptr)
{
  static cv::Mat bev_view_bkp = cv::Mat::zeros(BEV_HEIGHT, BEV_WIDTH, CV_8UC3);

  // clang-format off
  if(nullptr != image_fw_Ptr){m_FW_View   = cv::imdecode(image_fw_Ptr->data, cv::IMREAD_COLOR);}
  if(nullptr != image_fn_Ptr){m_FN_View   = cv::imdecode(image_fn_Ptr->data, cv::IMREAD_COLOR);}
  if(nullptr != image_rw_Ptr){m_RW_View   = cv::imdecode(image_rw_Ptr->data, cv::IMREAD_COLOR);}
  if(nullptr != image_rn_Ptr){m_RN_View   = cv::imdecode(image_rn_Ptr->data, cv::IMREAD_COLOR);}
  if(nullptr != image_fl_Ptr){m_FL_View   = cv::imdecode(image_fl_Ptr->data, cv::IMREAD_COLOR);}
  if(nullptr != image_fr_Ptr){m_FR_View   = cv::imdecode(image_fr_Ptr->data, cv::IMREAD_COLOR);}
  if(nullptr != image_rl_Ptr){m_RL_View   = cv::imdecode(image_rl_Ptr->data, cv::IMREAD_COLOR);}
  if(nullptr != image_rr_Ptr){m_RR_View   = cv::imdecode(image_rr_Ptr->data, cv::IMREAD_COLOR);}
  // clang-format on

  if(nullptr != img_bev_ptr)
  {
    parseDmAivImage(*img_bev_ptr, SensorId::CAMERA_BEV, m_BEV_View);
    m_BEV_View.copyTo(bev_view_bkp);
  }
  else
  {
    bev_view_bkp.copyTo(m_BEV_View);
  }

  if(camera_fs_bev_ptr != nullptr)
  {
    int    fontFace  = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.5;
    int    v_step    = 40;
    int    start     = 0;
    auto   citer     = BEV_FS_COLOR.cbegin();
    //    for(int i = 0; i < BEV_FS_COLOR.size(); ++i)
    //    {
    //      cv::Point txt_pos(start, (1 + (i + 1) * 2) * v_step);
    //      cv::putText(m_BEV_View, CLASS_STRINGS.at(citer->first), txt_pos, fontFace, fontScale, citer->second, 2);
    //      citer++;
    //    }
    //
    //    for(auto &p : camera_fs_bev_ptr->camera_freespace_points)
    //    {
    //      cv::circle(m_BEV_View, cv::Point2f((-p.y * 2 + BEV_WIDTH / 2 * 0.02) * FACTOR + 120, (-p.x * 2 + BEV_HEIGHT / 2 * 0.02) * FACTOR), 10, BEV_FS_COLOR.at(BEVCLASS_TO_CLASS.at(p.type)), -1);
    //    }
  } // -- Vis raw freespace
  drawLidarPointCloud(lidar_ptr);
  drawVotObject(vot_object_raw_ptr, "raw", 0);
  drawPolygonLane(std::vector<std::vector<cv::Point2f>>{polygonm_line});
  drawTimestamp(timestamp, m_BEV_View);
  drawTrajectory();
  drawEgo();
  drawRemap();
}

void fs::Vis::parseDmAivImage(const DMAIV::DmAivBatch &img_bev_ptr, const fs::SensorId sensorId, cv::Mat &dst)
{
  if(img_bev_ptr.dm_aiv_batch[0].debug_image_height * img_bev_ptr.dm_aiv_batch[0].debug_image_width != 0)
  {
    cv::Mat img(img_bev_ptr.dm_aiv_batch[0].debug_image_height, img_bev_ptr.dm_aiv_batch[0].debug_image_width, CV_8UC3, (uint8_t *)img_bev_ptr.dm_aiv_batch[0].debug_image_data.data());
    cv::resize(img, dst, cv::Size(BEV_WIDTH, BEV_HEIGHT));
  }
  else
  {
    dst = cv::Mat::zeros(BEV_HEIGHT, BEV_WIDTH, CV_8UC3);
  }
}
#endif

void fs::Vis::drawRemap()
{
  int     img_w = 3840;
  int     img_h = 2160;
  cv::Mat img   = cv::Mat::zeros(img_h, img_w, CV_8UC3);
  img.setTo(cv::Scalar(255, 255, 255));

#if FS_CHECK(CFG_ROS2)
  // clang-format off
    int max_length = INT_MIN;
    for (auto &c: VOT_PROTO_CLASS_TO_STRING) { max_length = std::max((int) c.second.size(), max_length); }// -- mot输入
    for (auto &c: CLASS_STRINGS) { max_length = std::max((int) c.second.size(), max_length); }// --fs 内置
    for (auto &c: FS_PROTO_CLASS_TO_STRING) { max_length = std::max((int) c.second.size(), max_length); }// --fs 输出
    int margin_w = 300;
    int box_h = 60;
    int box_w = (img_w - margin_w * 6) / 3;

    const auto box_string_mot = getMapInfo(VOT_PROTO_CLASS_TO_STRING, img_h, box_h, box_w, margin_w);
    const auto box_string_inr = getMapInfo(CLASS_STRINGS, img_h, box_h, box_w, margin_w + box_w + margin_w + margin_w);
    const auto box_string_out = getMapInfo(FS_PROTO_CLASS_TO_STRING, img_h, box_h, box_w,
                                           margin_w + box_w + margin_w + margin_w + box_w + margin_w + margin_w);

    for (auto &c: box_string_mot) {
        cv::rectangle(img, c.second.rect, cv::Scalar(0, 0, 255), 4);
        cv::putText(img, c.second.str, cv::Point(c.second.rect.x, c.second.rect.y + box_h), cv::FONT_HERSHEY_SIMPLEX, 2,
                    cv::Scalar(0, 0, 0), 4, 3);
        cv::Point p1{c.second.rect.x + box_w, c.second.rect.y + box_h / 2};
        cv::Point p2{box_string_inr.at(perceptionClass2ObjectClass(c.first)).rect.x,
                     box_string_inr.at(perceptionClass2ObjectClass(c.first)).rect.y + box_h / 2};
        cv::line(img, p1, p2, CLASS_COLOR.at(perceptionClass2ObjectClass(c.first)), 3);
    }
    for (auto &c: box_string_inr) {
        cv::rectangle(img, c.second.rect, cv::Scalar(0, 255, 0), 4);
        cv::putText(img, c.second.str, cv::Point(c.second.rect.x, c.second.rect.y + box_h), cv::FONT_HERSHEY_SIMPLEX, 2,
                    cv::Scalar(0, 0, 0), 4, 3);
        cv::Point p1{c.second.rect.x + box_w, c.second.rect.y + box_h / 2};
        cv::Point p2{box_string_out.at(CLASS_TO_FS_PROTO_CLASS.at(c.first)).rect.x,
                     box_string_out.at(CLASS_TO_FS_PROTO_CLASS.at(c.first)).rect.y + box_h / 2};
        cv::line(img, p1, p2, FS_PROTO_CLASS_TO_COLOR.at(CLASS_TO_FS_PROTO_CLASS.at(c.first)), 3);
    }
    for (auto &c: box_string_out) {
        cv::rectangle(img, c.second.rect, cv::Scalar(255, 0, 0), 4);
        cv::putText(img, c.second.str, cv::Point(c.second.rect.x, c.second.rect.y + box_h), cv::FONT_HERSHEY_SIMPLEX, 2,
                    cv::Scalar(0, 0, 0), 4, 3);
    }
    m_Remap_View = img;
  // clang-format on
#else
  // clang-format off
    int max_length = INT_MIN;
    for(auto & c:VOT_LCM_CLASS_TO_STRING){ max_length = std::max((int)c.second.size(), max_length); }// -- mot输入
    for(auto & c:CLASS_STRINGS){ max_length = std::max((int)c.second.size(), max_length); }// --fs 内置
    for(auto & c:FS_LCM_CLASS_TO_STRING){ max_length = std::max((int)c.second.size(), max_length); }// --fs 输出
    int margin_w   = 300;
    int box_h      = 60;
    int box_w      = (img_w - margin_w * 6) / 3;
  
    const auto box_string_mot = getMapInfo(VOT_LCM_CLASS_TO_STRING,img_h, box_h, box_w,  margin_w);
    const auto box_string_inr = getMapInfo(CLASS_STRINGS            ,img_h, box_h, box_w,  margin_w + box_w + margin_w + margin_w);
    const auto box_string_out = getMapInfo(FS_LCM_CLASS_TO_STRING ,img_h, box_h, box_w,  margin_w + box_w + margin_w + margin_w + box_w + margin_w + margin_w);
  
    for (auto & c: box_string_mot) {
      cv::rectangle(img,c.second.rect,cv::Scalar(0,0,255),4);
      cv::putText(img,c.second.str,cv::Point(c.second.rect.x, c.second.rect.y+box_h), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,0), 4, 3);
      cv::Point p1 {c.second.rect.x+box_w, c.second.rect.y+box_h/2};
      cv::Point p2 {box_string_inr.at(VOT_LCM_CLASS_TO_CLASS.at(c.first)).rect.x,box_string_inr.at(VOT_LCM_CLASS_TO_CLASS.at(c.first)).rect.y+box_h/2};
      cv::line(img, p1,p2,CLASS_COLOR.at(VOT_LCM_CLASS_TO_CLASS.at(c.first)),3);
    }
    for (auto & c: box_string_inr) {
      cv::rectangle(img,c.second.rect,cv::Scalar(0,255,0),4);
      cv::putText(img,c.second.str,cv::Point(c.second.rect.x, c.second.rect.y+box_h), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,0), 4, 3);
      if(c.first == GridLabel::UNOCCUPIED){ continue; }
      if(!CLASS_TO_FS_LCM_CLASS.count(c.first)){continue;}
      cv::Point p1 {c.second.rect.x+box_w, c.second.rect.y+box_h/2};
      cv::Point p2 {box_string_out.at(CLASS_TO_FS_LCM_CLASS.at(c.first)).rect.x,box_string_out.at(CLASS_TO_FS_LCM_CLASS.at(c.first)).rect.y+box_h/2};
      cv::line(img, p1,p2,FS_LCM_CLASS_TO_COLOR.at(CLASS_TO_FS_LCM_CLASS.at(c.first)),3);
    }
    for (auto & c: box_string_out) {
      cv::rectangle(img,c.second.rect,cv::Scalar(255,0,0),4);
      cv::putText(img,c.second.str,cv::Point(c.second.rect.x, c.second.rect.y+box_h), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,0), 4, 3);
    }
    m_Remap_View = img;
  // clang-format on

#endif
}

void fs::Vis::drawTimestamp(const int64_t timestamp, cv::Mat &img)
{
  std::stringstream str;
  str << timestamp;
  cv::putText(img, str.str(), cv::Point2i(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_R, 2);
}

void fs::Vis::drawBlindMakers(const std::vector<cv::Point2f> &l, const std::vector<cv::Point2f> &r)
{
  std::vector<FSVec3f> polygonLeft;
  for(const auto &p : l)
  {
    polygonLeft.emplace_back((p.x - HALF_GRID_MAP_SIZE) * GRID_SCALE, (p.y - HALF_GRID_MAP_SIZE) * GRID_SCALE, 0.f);
  }
  m_blind_markers.markers.push_back(
    VisHelper::createPolygon(polygonLeft, "left blind", 0, VisHelper::getColor(RvizColor::WHITE), VisHelper::getScale(RvizScale::XSMALL)));

  std::vector<FSVec3f> polygonRight;
  for(const auto &p : r)
  {
    polygonRight.emplace_back((p.x - HALF_GRID_MAP_SIZE) * GRID_SCALE, (p.y - HALF_GRID_MAP_SIZE) * GRID_SCALE, 0.f);
  }
  m_blind_markers.markers.push_back(
    VisHelper::createPolygon(polygonRight, "right blind", 0, VisHelper::getColor(fs::RvizColor::WHITE), VisHelper::getScale(RvizScale::XSMALL)));
}

void fs::Vis::drawBlindPointCloud(bool blind[GRID_MAP_SIZE][GRID_MAP_SIZE])
{
  m_blindPointCloud.header.frame_id = "ego";
  m_blindPointCloud.points.clear();
  m_blindPointCloud.points.reserve(1000);

  for(int row = 0; row < GRID_MAP_SIZE; ++row)
  {
    for(int col = 0; col < GRID_MAP_SIZE; ++col)
    {
      if(blind[row][col])
      {
        geometry_msgs::msg::Point32 &point = m_blindPointCloud.points.emplace_back();

        point.x = (row - HALF_GRID_MAP_SIZE) * GRID_SCALE;
        point.y = (col - HALF_GRID_MAP_SIZE) * GRID_SCALE;
        point.z = 0;
      }
    }
  }
}

void fs::Vis::drawEgo()
{
  constexpr float length = 5.f;

  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "ego vehicle";
    marker.id              = 0;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.type            = visualization_msgs::msg::Marker::CYLINDER;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.color           = VisHelper::getColor(RvizColor::RED);
    marker.pose            = VisHelper::convertPose(
      Eigen::Translation3d(0.5 * length, 0, 0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));

    marker.scale.x = 0.1f * length;
    marker.scale.y = 0.1f * length;
    marker.scale.z = length;

    m_markers.markers.push_back(marker);
  }

  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "ego vehicle";
    marker.id              = 1;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.type            = visualization_msgs::msg::Marker::CYLINDER;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.color           = VisHelper::getColor(RvizColor::GREEN);
    marker.pose            = VisHelper::convertPose(
      Eigen::Translation3d(0, 0.5 * length, 0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));

    marker.scale.x = 0.1f * length;
    marker.scale.y = 0.1f * length;
    marker.scale.z = length;

    m_markers.markers.push_back(marker);
  }

  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "ego vehicle";
    marker.id              = 2;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.type            = visualization_msgs::msg::Marker::CYLINDER;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.color           = VisHelper::getColor(RvizColor::BLUE);
    marker.pose            = VisHelper::convertPose(
      Eigen::Translation3d(0, 0, 0.5 * length) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));

    marker.scale.x = 0.1f * length;
    marker.scale.y = 0.1f * length;
    marker.scale.z = length;

    m_markers.markers.push_back(marker);
  }
}

void fs::Vis::drawTrajectory()
{
  const Vehicle &vehicle = Vehicle::getVehicle();

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "ego";
  marker.ns              = "trajectory";
  marker.id              = 0;
  marker.type            = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action          = visualization_msgs::msg::Marker::ADD;
  marker.lifetime        = rclcpp::Duration(0, 0);
  marker.pose            = VisHelper::getIdentityPose();

  marker.scale   = VisHelper::getScale(RvizScale::MEDIUM);
  marker.scale.y = 0.f;
  marker.scale.z = 0.f;
  marker.color   = VisHelper::getColor(RvizColor::BROWN);
  marker.points.reserve(vehicle.trajectory_world_.size());
  marker.colors.reserve(vehicle.trajectory_world_.size());

  for(auto iter = vehicle.trajectory_world_.cbegin(); iter != vehicle.trajectory_world_.cend(); ++iter)
  {
    const auto pointEgo = vehicle.transformWrd2Ego(*iter);

    if(iter != vehicle.trajectory_world_.cbegin() && iter < vehicle.trajectory_world_.cend() - 1)
    {
      marker.points.push_back(VisHelper::convertPoint(fs::FSVec3f{pointEgo.x(), pointEgo.y(), pointEgo.z()}));
    }

    marker.points.push_back(VisHelper::convertPoint(fs::FSVec3f{pointEgo.x(), pointEgo.y(), pointEgo.z()}));
    marker.colors.push_back(VisHelper::getColor(RvizColor::BROWN));
  }

  m_markers.markers.push_back(marker);
}

void fs::Vis::clearMarkers()
{
  {
    auto markers = m_markers.markers;
    m_markers.markers.clear();
    for(auto &marker : markers)
    {
      if(marker.action != visualization_msgs::msg::Marker::DELETE)
      {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        m_markers.markers.push_back(marker);
      }
    }
  }
  {
    auto markers = m_blind_markers.markers;
    m_blind_markers.markers.clear();
    for(auto &marker : markers)
    {
      if(marker.action != visualization_msgs::msg::Marker::DELETE)
      {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        m_blind_markers.markers.push_back(marker);
      }
    }
  }
}

void fs::Vis::drawGridMap(const fs::GridMap &gridMap)
{
  m_gridPointCloud.header.frame_id = "ego";
  m_gridPointCloud.points.clear();
  m_gridPointCloud.points.reserve(GRID_MAP_SIZE * GRID_MAP_SIZE);
  m_gridPointCloud.channels.resize(3);
  m_gridPointCloud.channels.front().name = "logit";
  m_gridPointCloud.channels.front().values.clear();
  m_gridPointCloud.channels.front().values.reserve(GRID_MAP_SIZE * GRID_MAP_SIZE);
  m_gridPointCloud.channels[1].name = "type";
  m_gridPointCloud.channels[1].values.clear();
  m_gridPointCloud.channels[1].values.reserve(GRID_MAP_SIZE * GRID_MAP_SIZE);
  m_gridPointCloud.channels.back().name = "motion";
  m_gridPointCloud.channels.back().values.clear();
  m_gridPointCloud.channels.back().values.reserve(GRID_MAP_SIZE * GRID_MAP_SIZE);

  constexpr int min_row = -HALF_TPTS_LENGTH;
  constexpr int max_row = +HALF_TPTS_LENGTH;
  constexpr int min_col = -HALF_TPTS_WIDTH;
  constexpr int max_col = +HALF_TPTS_WIDTH;

  for(int col = 0; col < GRID_MAP_SIZE; ++col)
  {
    for(int row = 0; row < GRID_MAP_SIZE; ++row)
    {
      const auto &ego_center_idx = gridMap.modIdx2EgoCenterIdxMap(GridMap::Index{row, col});

      if(ego_center_idx.x() < min_row || ego_center_idx.x() >= max_row || ego_center_idx.y() < min_col ||
         ego_center_idx.y() >= max_col)
      {
        if(gridMap.m_grids[row][col].logit >= 2)
        {
          const auto pos   = gridMap.egoCenterIdx2Pos(ego_center_idx);
          auto      &point = m_gridPointCloud.points.emplace_back();
          point.x          = (static_cast<int>(pos.x() * GRID_SCALE_INV)) * GRID_SCALE;
          point.y          = (static_cast<int>(pos.y() * GRID_SCALE_INV)) * GRID_SCALE;
          point.z          = 0.f;
          m_gridPointCloud.channels.front().values.emplace_back(gridMap.m_grids[row][col].logit);
          m_gridPointCloud.channels[1].values.emplace_back((int)gridMap.m_grids[row][col].grid_label);
          switch(gridMap.m_grids[row][col].motion_state)
          {
          case MotionState::STOPPED:
            m_gridPointCloud.channels.back().values.emplace_back(0);
            break;
          case MotionState::MOVING:
            m_gridPointCloud.channels.back().values.emplace_back(1);
            break;
          default:
            break;
          }
        }
      }
      else
      {
        if(gridMap.m_grids[row][col].logit >= 1)
        {
          const auto pos   = gridMap.egoCenterIdx2Pos(ego_center_idx);
          auto      &point = m_gridPointCloud.points.emplace_back();
          point.x          = (static_cast<int>(pos.x() * GRID_SCALE_INV)) * GRID_SCALE;
          point.y          = (static_cast<int>(pos.y() * GRID_SCALE_INV)) * GRID_SCALE;
          point.z          = 0.f;
          m_gridPointCloud.channels.front().values.emplace_back(gridMap.m_grids[row][col].logit);
          m_gridPointCloud.channels[1].values.emplace_back((int)gridMap.m_grids[row][col].grid_label);
          switch(gridMap.m_grids[row][col].motion_state)
          {
          case MotionState::STOPPED:
            m_gridPointCloud.channels.back().values.emplace_back(0);
            break;
          case MotionState::MOVING:
            m_gridPointCloud.channels.back().values.emplace_back(1);
            break;
          default:
            break;
          }
        } // -- tpts
      }
    }
  }
}

void fs::Vis::drawGridMapsplit(const fs::GridMap &gridMap)
{
  for(auto &cld : map_cld_built_in_)
  {
    cld.second.header.frame_id = "ego";
    cld.second.points.clear();
    cld.second.channels.resize(1);
    cld.second.channels.front().name = "logit/motion";
    cld.second.channels.front().values.clear();
  }

  constexpr int min_row_tpts = -HALF_TPTS_LENGTH;
  constexpr int max_row_tpts = +HALF_TPTS_LENGTH;
  constexpr int min_col_tpts = -HALF_TPTS_WIDTH;
  constexpr int max_col_tpts = +HALF_TPTS_WIDTH;

  for(int col = 0; col < GRID_MAP_SIZE; ++col)
  {
    for(int row = 0; row < GRID_MAP_SIZE; ++row)
    {
      if(gridMap.m_grids[row][col].logit >= 2 || gridMap.m_grids[row][col].tpts)
      {
        const auto &ego_idx_cur_in = gridMap.modIdx2EgoCenterIdxCurrent(GridMap::Index{row, col});

        if(gridMap.m_grids[row][col].tpts)
        {
          const FSVec2f               pos = ego_idx_cur_in * GRID_SCALE;
          geometry_msgs::msg::Point32 point;
          point.x            = pos.x();
          point.y            = pos.y();
          point.z            = gridMap.m_grids[row][col].top;
          float logit_motion = 0.f;
#if FS_CHECK(CFG_RVIZ_LOGIT_OR_MOTION)
          logit_motion = (float)gridMap.m_grids[row][col].motion_state;
#else
          logit_motion = (float)gridMap.m_grids[row][col].logit;
#endif
          emplaceBack(gridMap.m_grids[row][col].grid_label, point, logit_motion, map_cld_built_in_);
        } // -- tpts
        else
        {
          if(gridMap.m_grids[row][col].logit >= 2)
          {
            const FSVec2f               pos = ego_idx_cur_in * GRID_SCALE;
            geometry_msgs::msg::Point32 point;
            point.x            = pos.x();
            point.y            = pos.y();
            point.z            = gridMap.m_grids[row][col].top;
            float logit_motion = 0.f;
#if FS_CHECK(CFG_RVIZ_LOGIT_OR_MOTION)
            logit_motion = (float)gridMap.m_grids[row][col].motion_state;
#else
            logit_motion = (float)gridMap.m_grids[row][col].logit;
#endif
            emplaceBack(gridMap.m_grids[row][col].grid_label, point, logit_motion, map_cld_built_in_);
          }
        } // -- normal logit
      }
    }
  }
#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
  emplaceBack2(map_cld_built_in_, map_cld_built_in2_);
#endif
}

#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
void fs::Vis::emplaceBack2(std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud> &map_cld, std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud2> &map_cld2)
{
  sensor_msgs::convertPointCloudToPointCloud2(map_cld.at(GridLabel::PEDESTRIAN), map_cld2.at(GridLabel::PEDESTRIAN));

  sensor_msgs::convertPointCloudToPointCloud2(map_cld.at(GridLabel::CYCLIST), map_cld2.at(GridLabel::CYCLIST));

  sensor_msgs::convertPointCloudToPointCloud2(map_cld.at(GridLabel::VEHICLE), map_cld2.at(GridLabel::VEHICLE));

  sensor_msgs::convertPointCloudToPointCloud2(map_cld.at(GridLabel::RTG_QC), map_cld2.at(GridLabel::RTG_QC));

  sensor_msgs::convertPointCloudToPointCloud2(map_cld.at(GridLabel::LOCK_BOX), map_cld2.at(GridLabel::LOCK_BOX));

  sensor_msgs::convertPointCloudToPointCloud2(map_cld.at(GridLabel::CONE), map_cld2.at(GridLabel::CONE));

  sensor_msgs::convertPointCloudToPointCloud2(map_cld.at(GridLabel::AIV), map_cld2.at(GridLabel::AIV));

  sensor_msgs::convertPointCloudToPointCloud2(map_cld.at(GridLabel::TREE_BRANCH), map_cld2.at(GridLabel::TREE_BRANCH));

  sensor_msgs::convertPointCloudToPointCloud2(map_cld.at(GridLabel::OTHER), map_cld2.at(GridLabel::OTHER));
}
#endif

void fs::Vis::emplaceBack(const GridLabel grid_label, const geometry_msgs::msg::Point32 &point, const float logit_motion, std::map<const fs::GridLabel, sensor_msgs::msg::PointCloud> &map_cld)
{
  switch(grid_label)
  {
  case GridLabel::PEDESTRIAN:
    map_cld.at(GridLabel::PEDESTRIAN).points.emplace_back(point);
    map_cld.at(GridLabel::PEDESTRIAN).channels.back().values.emplace_back(logit_motion);
    break;
  case GridLabel::CYCLIST:
    map_cld.at(GridLabel::CYCLIST).points.emplace_back(point);
    map_cld.at(GridLabel::CYCLIST).channels.back().values.emplace_back(logit_motion);
    break;
  case GridLabel::VEHICLE:
    map_cld.at(GridLabel::VEHICLE).points.emplace_back(point);
    map_cld.at(GridLabel::VEHICLE).channels.back().values.emplace_back(logit_motion);
    break;
  case GridLabel::RTG_QC:
    map_cld.at(GridLabel::RTG_QC).points.emplace_back(point);
    map_cld.at(GridLabel::RTG_QC).channels.back().values.emplace_back(logit_motion);
    break;
  case GridLabel::LOCK_BOX:
    map_cld.at(GridLabel::LOCK_BOX).points.emplace_back(point);
    map_cld.at(GridLabel::LOCK_BOX).channels.back().values.emplace_back(logit_motion);
    break;
  case GridLabel::CONE:
    map_cld.at(GridLabel::CONE).points.emplace_back(point);
    map_cld.at(GridLabel::CONE).channels.back().values.emplace_back(logit_motion);
    break;
  case GridLabel::AIV:
    map_cld.at(GridLabel::AIV).points.emplace_back(point);
    map_cld.at(GridLabel::AIV).channels.back().values.emplace_back(logit_motion);
    break;
  case GridLabel::TREE_BRANCH:
    map_cld.at(GridLabel::TREE_BRANCH).points.emplace_back(point);
    map_cld.at(GridLabel::TREE_BRANCH).channels.back().values.emplace_back(logit_motion);
    break;
  default:
    map_cld.at(GridLabel::OTHER).points.emplace_back(point);
    map_cld.at(GridLabel::OTHER).channels.back().values.emplace_back(logit_motion);
    break;
  }
}

void fs::Vis::drawBevFs(const BevSeg *fsBev)
{
  if(fsBev == nullptr)
  {
    return;
  }
  m_bevPointCloud.header.frame_id = "ego";
  m_bevPointCloud.points.clear();
  m_bevPointCloud.points.reserve(fsBev->points.size());

  for(const auto &pt : fsBev->points)
  {
    if(pt.label == 0 && pt.score > 0.5)
    {
      geometry_msgs::msg::Point32 &point = m_bevPointCloud.points.emplace_back();

      point.x = pt.x;
      point.y = pt.y;
    }
  }
#if FS_CHECK(CFG_USE_FOXGLOV_VIS)
  sensor_msgs::convertPointCloudToPointCloud2(m_lidarPointCloud, m_lidarPointCloud2);
#endif
}
#if FS_CHECK(CFG_USE_ROAD_MODEL)
void fs::Vis::drawRoadModelPointCloud(const RoadModelFS *fsLidar)
{
  if(fsLidar == nullptr)
  {
    return;
  }
  m_roadPointCloud.header.frame_id = "ego";
  m_roadPointCloud.points.clear();
  m_roadPointCloud.points.reserve(fsLidar->points.size());

  for(const auto &grid : fsLidar->points)
  {
    geometry_msgs::msg::Point32 &point = m_roadPointCloud.points.emplace_back();

    point.x = grid.first.first;
    point.y = grid.first.second;
    point.z = grid.second;
  }

#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
  sensor_msgs::convertPointCloudToPointCloud2(m_roadPointCloud, m_roadPointCloud2);
#endif
}
#endif

void fs::Vis::drawLidarPointCloud(const LidarFS *fsLidar)
{
  if(fsLidar == nullptr)
  {
    UERROR << "nullptr";
    return;
  }
  m_lidarPointCloud.header.frame_id = "ego";
  m_lidarPointCloud.points.clear();
  m_lidarPointCloud.points.reserve(fsLidar->points.size());

  for(const auto &grid : fsLidar->points)
  {
    geometry_msgs::msg::Point32 &point = m_lidarPointCloud.points.emplace_back();

    point.x = grid.x;
    point.y = grid.y;
    point.z = grid.top;
  }

#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
  sensor_msgs::convertPointCloudToPointCloud2(m_lidarPointCloud, m_lidarPointCloud2);
#endif
}

void fs::Vis::drawRvizColorTable()
{
  m_colorTablePointCloud.header.frame_id = "ego";
  m_colorTablePointCloud.points.clear();
  m_colorTablePointCloud.points.reserve(2000);

  m_colorTablePointCloud.channels.resize(1);
  m_colorTablePointCloud.channels.front().name = "color table";
  m_colorTablePointCloud.channels.front().values.clear();
  m_colorTablePointCloud.channels.front().values.reserve(2000);

  for(int i = -1000; i < 1000; ++i)
  {
    geometry_msgs::msg::Point32 &point = m_colorTablePointCloud.points.emplace_back();

    point.x = i * 0.1;
    point.y = 51;
    point.z = 0;
    m_colorTablePointCloud.channels.front().values.emplace_back(i + 1000.0);
  }
}

void fs::Vis::drawPolygonPointCloud(const int polygon[GRID_MAP_SIZE][GRID_MAP_SIZE])
{
  if(polygon == nullptr)
  {
    return;
  }
  m_PolygonPointCloud.header.frame_id = "ego";
  m_PolygonPointCloud.points.clear();
  m_PolygonPointCloud.points.reserve(GRID_MAP_SIZE * GRID_MAP_SIZE);

  m_PolygonPointCloud.channels.resize(1);
  m_PolygonPointCloud.channels.front().name = "logit";
  m_PolygonPointCloud.channels.front().values.clear();
  m_PolygonPointCloud.channels.front().values.reserve(GRID_MAP_SIZE * GRID_MAP_SIZE);

  for(int row = 0; row < GRID_MAP_SIZE; ++row)
  {
    for(int col = 0; col < GRID_MAP_SIZE; ++col)
    {
      if(polygon[row][col] == 2)
      {
        geometry_msgs::msg::Point32 &point = m_PolygonPointCloud.points.emplace_back();

        point.x = (row - HALF_GRID_MAP_SIZE) * GRID_SCALE;
        point.y = -(col - HALF_GRID_MAP_SIZE) * GRID_SCALE;
        point.z = 0;
        m_PolygonPointCloud.channels.front().values.emplace_back(1000.0);
      } // -- side
    }
  }
  for(int row = 0; row < GRID_MAP_SIZE; row += 8)
  {
    for(int col = 0; col < GRID_MAP_SIZE; col += 8)
    {
      if(polygon[row][col] == 1)
      {
        geometry_msgs::msg::Point32 &point = m_PolygonPointCloud.points.emplace_back();

        point.x = (row - HALF_GRID_MAP_SIZE) * GRID_SCALE;
        point.y = -(col - HALF_GRID_MAP_SIZE) * GRID_SCALE;
        point.z = 0;
        m_PolygonPointCloud.channels.front().values.emplace_back(0.1);
      } // -- region
    }
  }
}

void fs::Vis::drawLidarPointCloudHistory(const std::vector<FSVec3f> &pointsWrd)
{
  m_lidarPointCloudHistory.header.frame_id = "ego";
  m_lidarPointCloudHistory.points.clear();
  m_lidarPointCloudHistory.points.reserve(pointsWrd.size());

  for(const auto &pointWrd : pointsWrd)
  {
    const Vehicle &vehicle  = Vehicle::getVehicle();
    const auto     pointEgo = vehicle.transformWrd2Ego(pointWrd);

    geometry_msgs::msg::Point32 &point = m_lidarPointCloudHistory.points.emplace_back();

    point.x = pointEgo.x();
    point.y = pointEgo.y();
    point.z = pointEgo.z();
  }
}

void fs::Vis::drawPolygonLane(const std::vector<std::vector<cv::Point2f>> &polygon_line)
{
  if(polygon_line.empty())
  {
    return;
  }
  for(int i = 0; i < polygon_line.size(); ++i)
  {
    if(polygon_line[i].empty())
    {
      continue;
    }
    std::vector<FSVec3f> polygonPointsEgo(4);
    polygonPointsEgo[0] = FSVec3f{polygon_line[i][0].x, polygon_line[i][0].y, 0};
    polygonPointsEgo[1] = FSVec3f{polygon_line[i][1].x, polygon_line[i][1].y, 0};
    polygonPointsEgo[2] = FSVec3f{polygon_line[i][2].x, polygon_line[i][2].y, 0};
    polygonPointsEgo[3] = FSVec3f{polygon_line[i][3].x, polygon_line[i][3].y, 0};
    m_markers.markers.push_back(
      VisHelper::createPolygon(polygonPointsEgo, "lane roi", i, VisHelper::getColor(RvizColor::CYAN), VisHelper::getScale(RvizScale::MEDIUM)));
  }
}

void fs::Vis::drawLines(const std::vector<std::vector<fs::LineCoeff>> &lines)
{
  if(lines.empty())
  {
    return;
  }
  for(int k = 0; k < lines[0].size(); ++k)
  {
    std::vector<FSVec3f> polygonPointsEgo;
    polygonPointsEgo.clear();
    for(int j = REMOVE_REGION_BACK; j <= REMOVE_REGION_FRONT; j += 2)
    {
      polygonPointsEgo.emplace_back(j, Utils::getPolynomialValue(float(j), lines[0][k].coeff), 0.f);
    }
    m_markers.markers.push_back(
      VisHelper::createPolygon(polygonPointsEgo, "fc_lane", k, VisHelper::getColor(RvizColor::CYAN), VisHelper::getScale(RvizScale::MEDIUM), false));
  }
  // -- AIV2.0  fc_lane and rc_lane is same
  //  if(lines.size() < 2)
  //  {
  //    return;
  //  }
  //
  //  for(int k = 0; k < lines[0].size(); ++k)
  //  {
  //    std::vector<FSVec3f> polygonPointsEgo;
  //    polygonPointsEgo.clear();
  //    for(int j = REMOVE_REGION_BACK; j <= 0; j += 2)
  //    {
  //      polygonPointsEgo.emplace_back(j, Utils::getPolynomialValue(float(j), lines[0][k].coeff), 0.f);
  //    }
  //    m_markers.markers.push_back(VisHelper::createPolygon(polygonPointsEgo, "rc_lane", k, VisHelper::getColor(RvizColor::CYAN), VisHelper::getScale(RvizScale::MEDIUM), false));
  //  }
}

void fs::Vis::drawVotObject(const ObjectStates *vot_object_ptr, std::string type, const float dt)
{
  if(vot_object_ptr == nullptr)
  {
    return;
  }
  for(const auto &object : vot_object_ptr->objects)
  {
    drawObjectBoxRviz(object, type, dt);
  }
}

void fs::Vis::drawGates(const std::vector<fs::Rect> gates_bbox)
{
  std::vector<FSVec3f> polygonPointsEgo(4);
  float                min_x = 0.f;
  float                max_x = 0.f;
  float                min_y = 0.f;
  float                max_y = 0.f;

  for(auto &ob : gates_bbox)
  {
    // arrow
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "ego";
      //      marker.ns              = type + "fused object velocity";
      //      marker.id              = ob.id;
      marker.type     = visualization_msgs::msg::Marker::ARROW;
      marker.action   = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration(0, 0);
      marker.points.resize(2);

      //      FSMat3x3 R          = Eigen::AngleAxisf(ob.yaw, FSVec3f::UnitZ()).matrix();
      polygonPointsEgo[0] = FSVec3f{ob.min_x, ob.min_y, 0.f};
      polygonPointsEgo[1] = FSVec3f{ob.min_x, ob.max_y, 0.f};
      polygonPointsEgo[2] = FSVec3f{ob.max_x, ob.max_y, 0.f};
      polygonPointsEgo[3] = FSVec3f{ob.max_x, ob.min_y, 0.f};

      const FSVec3f startPoint = 0.5f * (polygonPointsEgo[1] + polygonPointsEgo[2]);
      const FSVec3f stopPoint  = 0.5f * (polygonPointsEgo[0] + polygonPointsEgo[3]);
      marker.points[0]         = VisHelper::convertPoint(startPoint);
      marker.points[1]         = VisHelper::convertPoint(stopPoint);
      marker.pose              = VisHelper::getIdentityPose();
      //      marker.color             = color;
      marker.scale.x = 0.2;
      marker.scale.y = 0.4;
      marker.scale.z = 0.f;

      m_markers.markers.push_back(
        VisHelper::createPolygon(polygonPointsEgo, "fused object", 0, VisHelper::getColor(RvizColor::GREEN), VisHelper::getScale(RvizScale::MEDIUM)));

      m_markers.markers.push_back(marker);
    }
  }
}

void fs::Vis::drawZone(const std::vector<std::vector<fs::FSVec3f>> &boxes)
{
  if(boxes.size() == 2)
  {
    m_markers.markers.push_back(
      VisHelper::createPolygon(boxes[0], "Zone-Safe", 0, VisHelper::getColor(RvizColor::RED), VisHelper::getScale(RvizScale::MEDIUM)));
    m_markers.markers.push_back(
      VisHelper::createPolygon(boxes[1], "Zone-caution", 0, VisHelper::getColor(RvizColor::PINK), VisHelper::getScale(RvizScale::MEDIUM)));
  }
}

void fs::Vis::drawObjectBoxRviz(const ObjectState &ob, std::string type, const float dt)
{
  std::vector<FSVec3f> polygonPointsEgo;

  std::string expand{};
  if(type == "expand")
  {
    const auto &approx_polygon = Expand::instance().getPolygonByOval(ob, dt, expand);
    for(const auto &p : approx_polygon)
    {
      polygonPointsEgo.emplace_back(FSVec3f{p.x, p.y, 0.f});
    }
  }
  else
  {
    const auto &box_polygon = Expand::instance().assembleEgoBox(ob);
    for(const auto &p : box_polygon)
    {
      polygonPointsEgo.emplace_back(FSVec3f{p.x(), p.y(), 0.f});
    }
  }

  std_msgs::msg::ColorRGBA color;
  if(type == "raw")
  {
    color = VisHelper::getColor(RvizColor::WHITE);
  }
  else if(type == "expand")
  {
    color = VisHelper::getColor(RvizColor::BLUE);
  }
  else
  {
    color = VisHelper::getColor(
      std::hypot(ob.real_vx, ob.real_vy) > STOPPED_THRE ? RvizColor::YELLOW : RvizColor::RED);
    if(ob.type == GridLabel::RTG_QC)
    {
      color = VisHelper::getColor(ob.motion_state == MotionState::MOVING ? RvizColor::YELLOW : RvizColor::RED);
    }
  }

  // cuboid
  m_markers.markers.push_back(VisHelper::createPolygon(polygonPointsEgo, type + "fused object", ob.id, color, VisHelper::getScale(RvizScale::MEDIUM)));

  //MOVING/STOPPED
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "MOVING";
    marker.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.pose.position   = VisHelper::convertPoint(FSVec3f{20, 30, 0});
    marker.scale           = VisHelper::getScale(RvizScale::XLARGE, 3.f);
    marker.color           = VisHelper::getColor(RvizColor::YELLOW);
    std::stringstream ss;
    ss << "MOVING";
    marker.text = ss.str();

    m_markers.markers.push_back(marker);
  }

  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "STOPPED";
    marker.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.pose.position   = VisHelper::convertPoint(FSVec3f{25, 30, 0});
    marker.scale           = VisHelper::getScale(RvizScale::XLARGE, 3.f);
    marker.color           = VisHelper::getColor(RvizColor::RED);
    std::stringstream ss;
    ss << "STOPPED";
    marker.text = ss.str();

    m_markers.markers.push_back(marker);
  }

  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = "RAW LIDAR";
    marker.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.pose.position   = VisHelper::convertPoint(FSVec3f{30, 30, 0});
    marker.scale           = VisHelper::getScale(RvizScale::XLARGE, 3.f);
    marker.color           = VisHelper::getColor(RvizColor::WHITE);
    std::stringstream ss;
    ss << "RAW LIDAR";
    marker.text = ss.str();

    m_markers.markers.push_back(marker);
  }

  // arrow
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego";
    marker.ns              = type + "fused object velocity";
    marker.id              = ob.id * polygonPointsEgo[0].x();
    marker.type            = visualization_msgs::msg::Marker::ARROW;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration(0, 0);
    marker.points.resize(2);

    const FSVec3f startPoint = 0.5f * (polygonPointsEgo[1] + polygonPointsEgo[2]);
    const FSVec3f stopPoint  = startPoint + 0.5f * FSVec3f{ob.abs_vx, ob.abs_vy, 0.f};
    marker.points[0]         = VisHelper::convertPoint(startPoint);
    marker.points[1]         = VisHelper::convertPoint(stopPoint);
    marker.pose              = VisHelper::getIdentityPose();
    marker.color             = color;
    marker.scale.x           = 0.2;
    marker.scale.y           = 0.4;
    marker.scale.z           = 0.f;

    m_markers.markers.push_back(marker);
  }

  if(type == "expand")
  {
    // text
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "ego";
      marker.ns              = type + "fused object id";
      marker.id              = ob.id * polygonPointsEgo[0].x();
      marker.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action          = visualization_msgs::msg::Marker::ADD;
      marker.lifetime        = rclcpp::Duration(0, 0);
      marker.pose.position   = VisHelper::convertPoint(polygonPointsEgo[3]);
      marker.scale           = VisHelper::getScale(RvizScale::XLARGE, 3.f);
      marker.color           = color;
      std::stringstream ss;
      ss << "#" << ob.id;
      marker.text = ss.str();

      m_markers.markers.push_back(marker);
    }

    // text info
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "ego";
      marker.ns              = type + "fused object info";
      marker.id              = ob.id * polygonPointsEgo[0].x();
      ;
      marker.type          = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action        = visualization_msgs::msg::Marker::ADD;
      marker.lifetime      = rclcpp::Duration(0, 0);
      marker.pose.position = VisHelper::convertPoint(polygonPointsEgo[3]);
      marker.scale         = VisHelper::getScale(RvizScale::XLARGE, 3.f);
      marker.color         = color;
      std::stringstream ss;
      ss << "#" << ob.id << ": " << std::fixed << std::setprecision(2)
         << "[" << CLASS_STRINGS.at(ob.type)
         //       << "][pos: " << ob.x
         //       << "," << ob.y
         << "][vel: " << ob.abs_vx
         << "," << ob.abs_vy
         << "]"
         << "[" << expand << "]";
      marker.text = ss.str();

      m_markers.markers.push_back(marker);
    }
  }
}

void fs::Vis::drawTptsBboxRviz()
{
  constexpr float ROW = HALF_TPTS_LENGTH;
  constexpr float COL = HALF_TPTS_WIDTH;
  if(m_tptsBboxPointCloud.points.empty())
  {
    m_tptsBboxPointCloud.header.frame_id = "ego";
    geometry_msgs::msg::Point32 point;
    for(int i = -ROW; i < ROW; ++i)
    {
      point.x = i * GRID_SCALE;
      point.y = COL * GRID_SCALE;
      point.z = 0;
      m_tptsBboxPointCloud.points.emplace_back(point);
    } // -- left
    for(int i = -ROW; i < ROW; ++i)
    {
      point.x = i * GRID_SCALE;
      point.y = -COL * GRID_SCALE;
      point.z = 0;
      m_tptsBboxPointCloud.points.emplace_back(point);
    } // -- right
    for(int j = -COL; j < COL; ++j)
    {
      point.x = -ROW * GRID_SCALE;
      point.y = j * GRID_SCALE;
      point.z = 0;
      m_tptsBboxPointCloud.points.emplace_back(point);
    } // -- down
    for(int j = -COL; j < COL; ++j)
    {
      point.x = ROW * GRID_SCALE;
      point.y = j * GRID_SCALE;
      point.z = 0;
      m_tptsBboxPointCloud.points.emplace_back(point);
    } // -- up
  }
}

void fs::Vis::drawOutBboxRviz(float resolution)
{
  const float ROW_START = -FS_CAR_ROW_OUT * (resolution * GRID_SCALE_INV);
  const float ROW_END   = (FS_ROW_OUT - FS_CAR_ROW_OUT) * (resolution * GRID_SCALE_INV);
  const float COL_START = -FS_CAR_COL_OUT * (resolution * GRID_SCALE_INV);
  const float COL_END   = (FS_COL_OUT - FS_CAR_COL_OUT) * (resolution * GRID_SCALE_INV);
  if(m_outBboxPointCloud.points.empty())
  {
    m_outBboxPointCloud.header.frame_id = "ego";
    geometry_msgs::msg::Point32 point;
    for(int i = ROW_START; i < ROW_END; ++i)
    {
      point.x = i * GRID_SCALE;
      point.y = COL_START * GRID_SCALE;
      point.z = 0;
      m_outBboxPointCloud.points.emplace_back(point);
    } // -- left
    for(int i = ROW_START; i < ROW_END; ++i)
    {
      point.x = i * GRID_SCALE;
      point.y = COL_END * GRID_SCALE;
      point.z = 0;
      m_outBboxPointCloud.points.emplace_back(point);
    } // -- right
    for(int j = COL_START; j < COL_END; ++j)
    {
      point.x = ROW_START * GRID_SCALE;
      point.y = j * GRID_SCALE;
      point.z = 0;
      m_outBboxPointCloud.points.emplace_back(point);
    } // -- down
    for(int j = COL_START; j < COL_END; ++j)
    {
      point.x = ROW_END * GRID_SCALE;
      point.y = j * GRID_SCALE;
      point.z = 0;
      m_outBboxPointCloud.points.emplace_back(point);
    } // -- up
  }
}

#if !FS_CHECK(CFG_ROS2)
void fs::Vis::drawEgoBboxRviz()
{
  constexpr float ROW = HALF_EGO_LENGTH;
  constexpr float COL = HALF_EGO_WIDTH;
  if(m_egoBboxPointCloud.points.empty())
  {
    m_egoBboxPointCloud.header.frame_id = "ego";
    geometry_msgs::msg::Point32 point;
    for(int i = -ROW; i < ROW; ++i)
    {
      point.x = i * GRID_SCALE;
      point.y = COL * GRID_SCALE;
      point.z = 0;
      m_egoBboxPointCloud.points.emplace_back(point);
    } // -- left
    for(int i = -ROW; i < ROW; ++i)
    {
      point.x = i * GRID_SCALE;
      point.y = -COL * GRID_SCALE;
      point.z = 0;
      m_egoBboxPointCloud.points.emplace_back(point);
    } // -- right
    for(int j = -COL; j < COL; ++j)
    {
      point.x = -ROW * GRID_SCALE;
      point.y = j * GRID_SCALE;
      point.z = 0;
      m_egoBboxPointCloud.points.emplace_back(point);
    } // -- down
    for(int j = -COL; j < COL; ++j)
    {
      point.x = ROW * GRID_SCALE;
      point.y = j * GRID_SCALE;
      point.z = 0;
      m_egoBboxPointCloud.points.emplace_back(point);
    } // -- up
  }
}
void fs::Vis::drawSelfReceivePointCloud(const Lidar_FreeSpace_v2 &lidar_fs)
{
  for(auto &cld : map_cld_self_receive_)
  {
    cld.second.header.frame_id = "ego";
    cld.second.points.clear();
    cld.second.channels.resize(1);
    cld.second.channels.front().name = "logit/motion";
    cld.second.channels.front().values.clear();
  }

  for(auto &&p : lidar_fs.gstPoints)
  {
    geometry_msgs::msg::Point32 point;
    point.x = (p.nRowIndex - lidar_fs.nVehicleOriginRows) * lidar_fs.fResolution;
    point.y = (-p.nColIndex + lidar_fs.nVehicleOriginCols) * lidar_fs.fResolution;
    point.z = p.nHight * 0.1;
    emplaceBack(FS_LCM_CLASS_TO_CLASS.at(p.nType), point, 0, map_cld_self_receive_);
  }
}
void fs::Vis::drawSelfReceivePointCloud(const PerceptionFreeSpace &lidar_fs)
{
  for(auto &cld : map_cld_self_receive_)
  {
    cld.second.header.frame_id = "ego";
    cld.second.points.clear();
    cld.second.channels.resize(1);
    cld.second.channels.front().name = "logit/motion";
    cld.second.channels.front().values.clear();
  }
  for(auto &&p : lidar_fs.gstPoints)
  {
    geometry_msgs::msg::Point32 point;
    point.x            = (p.nRowIndex - lidar_fs.nVehicleOriginRows) * lidar_fs.fResolution;
    point.y            = (-p.nColIndex + lidar_fs.nVehicleOriginCols) * lidar_fs.fResolution;
    point.z            = p.fHeight;
    float logit_motion = 0.f;
#if FS_CHECK(CFG_RVIZ_LOGIT_OR_MOTION)
    logit_motion = (float)p.nMoveStatus;
#else
    logit_motion = (float)p.nMoveStatus;
#endif
    emplaceBack(FS_LCM_CLASS_TO_CLASS.at(p.nType), point, logit_motion, map_cld_self_receive_);
  }
#if FS_CHECK(CFG_USE_FOXGLOV_VIS) || FS_CHECK(CFG_USE_MCAP_FUSION)
  emplaceBack2(map_cld_self_receive_, map_cld_self_receive2_);
#endif
}
#else

void fs::Vis::drawSelfReceivePointCloud(const uto::proto::PerceptionFreespace &lidar_fs)
{
#if FS_CHECK(CFG_USE_LIGHT_COMMON)
  for(auto &cld : map_cld_self_receive_)
  {
    cld.second.header.frame_id = "ego";
    cld.second.points.clear();
    cld.second.channels.resize(1);
    cld.second.channels.front().name = "logit/motion";
    cld.second.channels.front().values.clear();
  }
  for(int i = 0; i < lidar_fs.grid_positions().size(); i += 2)
  {
    geometry_msgs::msg::Point32 point;

    point.x            = lidar_fs.grid_positions(i);
    point.y            = lidar_fs.grid_positions(i + 1);
    point.z            = lidar_fs.altitudes(i) * 0.001f;
    float logit_motion = 0.f;

#if FS_CHECK(CFG_RVIZ_LOGIT_OR_MOTION)
    logit_motion       = (float)HDT2_LIDAR_CLASS_TO_MOTION_STATE.at(lidar_fs.labels(i / 2));
#else
    logit_motion = (float)GT_CLASS_TO_CLASS.at(lidar_fs->labels(i / 2));
#endif
    emplaceBack(FS_PROTO_CLASS_TO_CLASS.at(lidar_fs.labels(i / 2)), point, logit_motion, map_cld_self_receive_);
  }
#else

  for(auto &cld : map_cld_self_receive_)
  {
    cld.second.header.frame_id = "ego";
    cld.second.points.clear();
    cld.second.channels.resize(1);
    cld.second.channels.front().name = "logit/motion";
    cld.second.channels.front().values.clear();
  }

  for(int i = 0; i < lidar_fs.perception_gridmap().size(); ++i)
  {
    geometry_msgs::msg::Point32 point;
    point.x            = lidar_fs.perception_gridmap().data()[i]->position().x();
    point.y            = lidar_fs.perception_gridmap().data()[i]->position().y();
    point.z            = lidar_fs.perception_gridmap().data()[i]->top();
    float logit_motion = 0.f;
#if FS_CHECK(CFG_RVIZ_LOGIT_OR_MOTION)
    logit_motion       = (float)HDT2_MOVESTATUS_TO_MOVESTATUS.at(lidar_fs.perception_gridmap().data()[i]->move_status());
#else
    logit_motion = (float)GT_CLASS_TO_CLASS.at(lidar_fs->perception_gridmap().data()[i]->lidar_label());
#endif
    emplaceBack(FS_PROTO_CLASS_TO_CLASS.at(lidar_fs.perception_gridmap().data()[i]->label()), point, logit_motion, map_cld_self_receive_);
  }
#endif

#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
  emplaceBack2(map_cld_self_receive_, map_cld_self_receive2_);
#endif
}

void fs::Vis::drawSelfReceivePointCloud(const uto::proto::PerceptionFreespace &lidar_fs, const EgoMotion *ego_motion)
{
#if FS_CHECK(CFG_USE_LIGHT_COMMON)
  for(auto &cld : map_cld_self_receive_)
  {
    cld.second.header.frame_id = "ego";
    cld.second.points.clear();
    cld.second.channels.resize(1);
    cld.second.channels.front().name = "logit/motion";
    cld.second.channels.front().values.clear();
  }
  for(int i = 0; i < lidar_fs.grid_positions().size(); i += 2)
  {
    geometry_msgs::msg::Point32 point;

    point.x            = lidar_fs.grid_positions(i);
    point.y            = lidar_fs.grid_positions(i + 1);
    point.z            = lidar_fs.altitudes(i) * 0.001f;
    float logit_motion = 0.f;

#if FS_CHECK(CFG_RVIZ_LOGIT_OR_MOTION)
    logit_motion       = (float)HDT2_LIDAR_CLASS_TO_MOTION_STATE.at(lidar_fs.labels(i / 2));
#else
    logit_motion = (float)GT_CLASS_TO_CLASS.at(lidar_fs->labels(i / 2));
#endif
    emplaceBack(FS_PROTO_CLASS_TO_CLASS.at(lidar_fs.labels(i / 2)), point, logit_motion, map_cld_self_receive_);
  }
#else

  auto Tcw = Utils::inverse(Utils::makeTFrom6Dof(lidar_fs.origin_point_theta(), 0., 0., lidar_fs.origin_point().x() - ego_motion->init_x, lidar_fs.origin_point().y() - ego_motion->init_y, 0.));
  for(auto &cld : map_cld_self_receive_)
  {
    cld.second.header.frame_id = "ego";
    cld.second.points.clear();
    cld.second.channels.resize(1);
    cld.second.channels.front().name = "logit/motion";
    cld.second.channels.front().values.clear();
  }

  for(int i = 0; i < lidar_fs.perception_gridmap().size(); ++i)
  {
    geometry_msgs::msg::Point32 point;
    const Eigen::Vector3f       pos_cur_ego = (Tcw * HOMO4(Eigen::Vector3f(lidar_fs.perception_gridmap().data()[i]->position().x() - ego_motion->init_x, lidar_fs.perception_gridmap().data()[i]->position().y() - ego_motion->init_y, 0.0))).topRows(3);
    point.x                                 = pos_cur_ego.x();
    point.y                                 = pos_cur_ego.y();
    point.z                                 = lidar_fs.perception_gridmap().data()[i]->top();
    float logit_motion                      = 0.f;
#if FS_CHECK(CFG_RVIZ_LOGIT_OR_MOTION)
    logit_motion                            = (float)HDT2_MOVESTATUS_TO_MOVESTATUS.at(lidar_fs.perception_gridmap().data()[i]->move_status());
#else
    logit_motion = (float)GT_CLASS_TO_CLASS.at(lidar_fs->perception_gridmap().data()[i]->lidar_label());
#endif
    emplaceBack(FS_PROTO_CLASS_TO_CLASS.at(lidar_fs.perception_gridmap().data()[i]->label()), point, logit_motion, map_cld_self_receive_);
  }
#endif

#if FS_CHECK(CFG_USE_MCAP_RAW) || FS_CHECK(CFG_USE_MCAP_FUSION) || FS_CHECK(CFG_USE_FOXGLOV_VIS)
  emplaceBack2(map_cld_self_receive_, map_cld_self_receive2_);
#endif
}
#endif

#if FS_CHECK(CFG_ROS2)

void fs::Vis::prepareRawImage(const sensor_msgs::msg::CompressedImage &src, const SensorId sensorId, cv::Mat &dst)
{
  cv::Mat img    = cv::imdecode(src.data, cv::IMREAD_COLOR);
  auto    camera = CameraManager::getCamera(sensorId);
  if(camera == nullptr)
    return;
  cv::resize(img, dst, cv::Size{VIS_IMAGE_VIEW_SCALING * camera->getImageWidth(), VIS_IMAGE_VIEW_SCALING * camera->getImageHeight()});
}

#else
//void fs::Vis::prepareRawImage(const LOGITECH_IMAGE &src, const fs::SensorId sensorId, cv::Mat &dst)
//{
//  cv::Mat       img    = cv::imdecode(src.data, cv::IMREAD_COLOR);
//  const Camera &camera = CameraManager::getCamera(sensorId);
//  cv::resize(img, img, cv::Size{camera.image_width_, camera.image_height_});
//  cv::remap(img, dst, camera.m_map1, camera.m_map2, cv::INTER_LINEAR);
//  cv::resize(dst, dst, cv::Size{VIS_IMAGE_VIEW_SCALING * camera.image_width_, VIS_IMAGE_VIEW_SCALING * camera.image_height_});
//}

void fs::Vis::prepareRawImage(const LOGITECH_IMAGE &src, const fs::SensorId sensorId, cv::Mat &dst)
{
  cv::Mat        img    = cv::imdecode(src.data, cv::IMREAD_COLOR);
  const Pinhole *camera = static_cast<Pinhole *>(const_cast<Camera *>(CameraManager::getCamera(sensorId).get()));
  cv::resize(img, img, cv::Size{camera->getImageWidth(), camera->getImageHeight()});
  cv::remap(img, dst, camera->getmap1(), camera->getmap2(), cv::INTER_LINEAR);
  cv::resize(dst, dst, cv::Size{VIS_IMAGE_VIEW_SCALING * camera->getImageWidth(), VIS_IMAGE_VIEW_SCALING * camera->getImageHeight()});
}
#endif
