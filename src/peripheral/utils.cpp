#include "utils.h"
bool fs::Utils::pointInPolygonQuick(const float px, const float py, const std::vector<cv::Point2f>& poly)
{
  size_t j    = poly.size() - 1;
  bool   flag = false;
  for(size_t i = 0; i < poly.size(); i++)
  {
    if(((poly[i].y < py && poly[j].y >= py) || (poly[j].y < py && poly[i].y >= py)) && (poly[i].x <= px || poly[j].x <= px))
    {
      flag ^= (poly[i].x + (py - poly[i].y) / (poly[j].y - poly[i].y) * (poly[j].x - poly[i].x) < px);
    }
    j = i;
  }
  return flag;
}

bool fs::Utils::pointInPolygonQuick(const int px, const int py, const std::vector<cv::Point>& poly)
{
  size_t j    = poly.size() - 1;
  bool   flag = false;
  for(size_t i = 0; i < poly.size(); i++)
  {
    if(((poly[i].y < py && poly[j].y >= py) || (poly[j].y < py && poly[i].y >= py)) && (poly[i].x <= px || poly[j].x <= px))
    {
      flag ^= (poly[i].x + (py - poly[i].y) / (poly[j].y - poly[i].y) * (poly[j].x - poly[i].x) < px);
    }
    j = i;
  }
  return flag;
}

bool fs::Utils::pointInPoly(float px, float py, const std::vector<cv::Point2f>& poly)
{
  // clang-format off
  bool flag = false;
  for(size_t i = 0, l = poly.size(), j = l - 1; i < l; j = i, i++) {
    float sx = poly[i].x;
    float sy = poly[i].y;
    float tx = poly[j].x;
    float ty = poly[j].y;
    if((sx == px && sy == py) || (tx == px && ty == py)) { return true; }
    if((sy < py && ty >= py) || (sy >= py && ty < py)) {
      float x = sx + (py - sy) * (tx - sx) / (ty - sy);
      if(x == px) { return true; }
      if(x > px) { flag = !flag;}
    }
  }
  return flag;
  // clang-format on
}

bool fs::Utils::pointInPoly(float px, float py, const std::vector<Eigen::Vector2f>& poly)
{
  // clang-format off
  bool flag = false;
  for(size_t i = 0, l = poly.size(), j = l - 1; i < l; j = i, i++) {
    float sx = poly[i].x();
    float sy = poly[i].y();
    float tx = poly[j].x();
    float ty = poly[j].y();
    if((sx == px && sy == py) || (tx == px && ty == py)) { return true; }
    if((sy < py && ty >= py) || (sy >= py && ty < py)) {
      float x = sx + (py - sy) * (tx - sx) / (ty - sy);
      if(x == px) { return true; }
      if(x > px) { flag = !flag;}
    }
  }
  return flag;
  // clang-format on
}

bool fs::Utils::pointInPolys(float px, float py, const std::vector<std::vector<Eigen::Vector2f>>& polys)
{
  for(const auto& poly : polys)
  {
    if(pointInPoly(px, py, poly))
    {
      return true;
    }
  }
  return false;
}

bool fs::Utils::pointInPolys(float px, float py, const std::vector<std::vector<cv::Point2f>>& polys)
{
  for(const auto& poly : polys)
  {
    if(pointInPoly(px, py, poly))
    {
      return true;
    }
  }
  return false;
}

void fs::Utils::getBresenhamLine(int x0, int y0, int x1, int y1, int polygon[GRID_MAP_SIZE][GRID_MAP_SIZE])
{
  // clang-format off
  const int dx = abs(x1-x0),   dy = abs(y1-y0);// -- 变化
  const int sx = x0<x1 ? 1 : -1,  sy = y0<y1 ? 1 : -1;// -- 象限
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
    polygon[x0][y0] = 2;
    if (x0==x1 && y0==y1) { break; }
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
  // clang-format on
}
void fs::Utils::getBresenhamLine(int x0, int y0, int x1, int y1, std::vector<cv::Point>& lines)
{
  // clang-format off
  const int dx = abs(x1-x0),   dy = abs(y1-y0);// -- 变化
  const int sx = x0<x1 ? 1 : -1,  sy = y0<y1 ? 1 : -1;// -- 象限
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
    lines.emplace_back(x0,y0);
    if (x0==x1 && y0==y1) { break; }
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
  // clang-format on
}
void fs::Utils::getBresenhamLine(int x0, int y0, int x1, int y1, std::vector<FSVec2>& lines)
{
  // clang-format off
  const int dx = abs(x1-x0),   dy = abs(y1-y0);// -- 变化
  const int sx = x0<x1 ? 1 : -1,  sy = y0<y1 ? 1 : -1;// -- 象限
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
    lines.emplace_back(x0,y0);
    if (x0==x1 && y0==y1) { break; }
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
  // clang-format on
}

fs::FSMat4x4 fs::Utils::makeTFrom6Dof(float yaw, float pitch, float roll, float tx, float ty, float tz)
{
  FSMat4x4 Twb = FSMat4x4::Identity();
  T2R(Twb)     = FSMat3x3(Eigen::AngleAxis<float>(yaw, FSVec3f::UnitZ()) * Eigen::AngleAxis<float>(pitch, FSVec3f::UnitY()) * Eigen::AngleAxis<float>(roll, FSVec3f::UnitX()));
  T2t(Twb)     = FSVec3f(tx, ty, tz);
  return Twb;
}

fs::FSMat4x4 fs::Utils::makeTFrom6Dof(const fs::EgoMotion& ego_motion)
{
  FSMat4x4 Twb = FSMat4x4::Identity();
  T2R(Twb)     = FSMat3x3(Eigen::AngleAxis<float>(ego_motion.yaw, FSVec3f::UnitZ()) * Eigen::AngleAxis<float>(0.f, FSVec3f::UnitY()) * Eigen::AngleAxis<float>(0.f, FSVec3f::UnitX()));
  T2t(Twb)     = FSVec3f(ego_motion.translation.x(), ego_motion.translation.y(), 0.f);
  return Twb;
}

fs::FSMat4x4 fs::Utils::inverse(const fs::FSMat4x4& Twb)
{
  FSMat4x4 Tbw               = FSMat4x4::Identity();
  Tbw.topLeftCorner<3, 3>()  = T2R_inv(Twb);
  Tbw.topRightCorner<3, 1>() = -Tbw.topLeftCorner<3, 3>() * Twb.topRightCorner<3, 1>();
  return Tbw;
}

Eigen::Vector2d fs::Utils::convertLLA2UTM(const double longitude, const double latitude)
{
  int    zone    = 0;
  bool   isNorth = false;
  double x = 0.0, y = 0.0, angleDiff = 0.0, scaleFactor = 0.0;
  GeographicLib::UTMUPS::Forward(latitude, longitude, zone, isNorth, x, y, angleDiff, scaleFactor);
  return Eigen::Vector2d{x, y};
}

Eigen::Vector2d fs::Utils::convertLLA2LTM(const double longitude, const double latitude, const Eigen::Vector3d& originPoint)
{
  uto::PoseTransformer pose;
  Eigen::Vector3d      positionInLtm{0.0, 0.0, 0.0};
  pose.setLtmOrigin(originPoint.x(), originPoint.y(), originPoint.z());
  pose.llaToLtm(Eigen::Vector3d{longitude, latitude, 0.0}, Eigen::Vector3d::Zero(), &positionInLtm, nullptr);
  return Eigen::Vector2d{positionInLtm.x(), positionInLtm.y()};
}

double fs::Utils::convertHeadingLLA2UTM(const double longitude, const double latitude, const double heading)
{
  int    zone        = 0;
  bool   isNorth     = false;
  double x           = 0.0;
  double y           = 0.0;
  double angleDiff   = 0.0;
  double scaleFactor = 0.0;

  GeographicLib::UTMUPS::Forward(latitude, longitude, zone, isNorth, x, y, angleDiff, scaleFactor);
  return heading + angleDiff / RAD2DEG;
}

cv::Point fs::Utils::findB(const cv::Point& C, const cv::Point& D, float x)
{
  // 计算 A 点坐标
  cv::Point A(C.x - x, C.y);

  // 计算 CD 和 BD 的斜率
  float slopeCD = static_cast<float>(C.y - D.y) / static_cast<float>(C.x - D.x);
  float slopeBD = -static_cast<float>(D.x - C.x) / static_cast<float>(C.y - D.y);

  // 计算 B 点到 D 点的距离
  float distBD = std::sqrt(x * x + x * x / (slopeBD * slopeBD));

  // 计算 B 点坐标
  float Bx = D.x + distBD / std::sqrt(1 + slopeBD * slopeBD);
  float By = D.x + slopeBD * (Bx - D.x);

  return cv::Point(Bx, By);
}

float fs::Utils::interpolate(const std::map<const float, const float>& expand_map, const float x)
{
  float ret = 0.f;

  auto iter = expand_map.lower_bound(x);
  if(expand_map.cend() == iter)
  {
    ret = expand_map.crbegin()->second;
  }
  else if(expand_map.cbegin() == iter)
  {
    ret = expand_map.cbegin()->second;
  }
  else
  {
    const float rhsX = iter->first;
    const float rhsY = iter->second;
    --iter;
    const float lhsX     = iter->first;
    const float lhsY     = iter->second;
    const float lhsRatio = (rhsX - x) / (rhsX - lhsX);
    const float rhsRatio = 1.f - lhsRatio;

    ret = lhsY * lhsRatio + rhsY * rhsRatio;
  }

  return ret;
}

fs::CloudPose::Ptr fs::Utils::tranfromFreespace2CloudPose(const LidarFS* fs_lidar_ptr, const EgoMotion* ego_motion)
{
  CloudPose::Ptr                 cld_ptr = std::make_shared<CloudPose>();
  pcl::PointCloud<pcl::PointXYZ> cld;
  cld_ptr->cloud     = cld.makeShared();
  cld_ptr->timestamp = fs_lidar_ptr->timestamp;
  cld_ptr->pose      = *ego_motion;
  for(auto&& p : fs_lidar_ptr->points)
  {
    cld_ptr->cloud->points.emplace_back(p.x, p.y, p.top);
    cld_ptr->cloud->points.emplace_back(p.x, p.y, p.bottom);
  }
  return cld_ptr;
}

std::string fs::Utils::makeDirSimple(const std::string& _output_dir)
{
  std::string save_dir = _output_dir;
  if(save_dir.back() != '/')
  {
    save_dir = save_dir + "/";
  }
  if(access(save_dir.c_str(), 0) != 0)
  {
    std::string mkdir_cmd = "mkdir -p " + save_dir;
    int         res       = system(mkdir_cmd.c_str());
  }
  return save_dir;
}

std::string fs::Utils::getCurrentTimeStamp(int time_stamp_type)
{
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

  std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
  std::tm*    now_tm     = std::localtime(&now_time_t);

  char buffer[128];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H:%M:%S", now_tm);

  std::ostringstream ss;
  ss.fill('0');

  std::chrono::milliseconds ms;
  std::chrono::microseconds cs;
  std::chrono::nanoseconds  ns;

  switch(time_stamp_type)
  {
  case 0:
    ss << buffer;
    break;
  case 1:
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    ss << buffer << ":" << ms.count();
    break;
  case 2:
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    cs = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
    ss << buffer << "" << ms.count() << ":" << cs.count() % 1000;
    break;
  case 3:
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    cs = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
    ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()) % 1000000000;
    ss << buffer << ":" << ms.count() << ":" << cs.count() % 1000 << ":" << ns.count() % 1000;
    break;
  default:
    ss << buffer;
    break;
  }

  return ss.str();
}
