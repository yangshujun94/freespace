//
// Created by uto_zzq on 23-8-15.
//

#include "expand.h"
fs::Expand& fs::Expand::instance()
{
  static Expand instance;
  return instance;
}

std::vector<cv::Point2f> fs::Expand::getPolygon(const std::vector<FSVec2f>& box_polygon_ego, float x, float y) const
{
  std::vector<cv::Point2f> candian_polygon;
  const float              theta_step = M_PI / 8;
  float                    theta      = 0;
  while(theta < 2 * M_PI)
  {
    theta += theta_step;
    const float delta_x = x * sin(theta);
    const float delta_y = y * cos(theta);
    for(auto p : box_polygon_ego)
    {
      candian_polygon.emplace_back(p.x() + delta_x, p.y() + delta_y);
    }
  }
  const auto               polygon = Utils::findConvexHull(candian_polygon);
  std::vector<cv::Point2f> approx_polygon;
  cv::approxPolyDP(polygon, approx_polygon, 0.1f, true);

  return approx_polygon;
}

std::vector<fs::FSVec2f> fs::Expand::assembleEgoBox(const ObjectState& ob) const
{
  float max_x = ob.length / 2.0f;
  float min_x = -max_x;
  float max_y = ob.width / 2.0f;
  float min_y = -max_y;

  if(ob.type == GridLabel::AIV)
  {
    max_x = (ob.length + 2.0f) / 2.0f;
    min_x = -max_x;
  }
#if !FS_CHECK(CFG_EXPAND_TABLE)
  else if(ob.type == GridLabel::PEDESTRIAN || ob.type == GridLabel::CONE || ob.type == GridLabel::CYCLIST)
  {
    max_x = (ob.length + 0.4f) / 2.0f;
    min_x = -max_x;
    max_y = (ob.width + 0.4F) / 2.0f;
    min_y = -max_y;
  }
  else
  {
    max_x = (ob.length + 1.0f) / 2.0f;
    min_x = -max_x;
    max_y = (ob.width + 0.6F) / 2.0f;
    min_y = -max_y;
  }
#endif

  FSMat3x3             R = Eigen::AngleAxisf(ob.yaw, FSVec3f::UnitZ()).matrix();
  FSVec2f              p1{min_x * R(0, 0) + min_y * R(0, 1) + ob.x, min_x * R(1, 0) + min_y * R(1, 1) + ob.y}; // -- p3---p4
  FSVec2f              p2{min_x * R(0, 0) + max_y * R(0, 1) + ob.x, min_x * R(1, 0) + max_y * R(1, 1) + ob.y}; // -- |     |
  FSVec2f              p3{max_x * R(0, 0) + max_y * R(0, 1) + ob.x, max_x * R(1, 0) + max_y * R(1, 1) + ob.y}; // -- |     |
  FSVec2f              p4{max_x * R(0, 0) + min_y * R(0, 1) + ob.x, max_x * R(1, 0) + min_y * R(1, 1) + ob.y}; // -- p2---p1
  std::vector<FSVec2f> box_polygon{p1, p2, p3, p4};
  return box_polygon;
}

#if FS_CHECK(CFG_EXPAND_TABLE)
fs::Expand::Expand()
{
  // clang-format off
  memset(y_expand_map_,0.,sizeof(y_expand_map_));
  y_expand_map_[0][0] = 0.5f; y_expand_map_[0][1] = 1.0f;  y_expand_map_[0][2] = 2.0f;
  y_expand_map_[1][0] = 0.5f; y_expand_map_[1][1] = 2.0f;  y_expand_map_[1][2] = 2.0f;
  y_expand_map_[2][0] = 0.5f; y_expand_map_[2][1] = 1.0f;  y_expand_map_[2][2] = 2.0f;
  y_expand_map_[3][0] = 0.1f; y_expand_map_[3][1] = 0.2f;  y_expand_map_[3][2] = 0.2f;
  y_expand_map_[4][0] = 0.2f; y_expand_map_[4][1] = 0.35f; y_expand_map_[4][2] = 0.35f;

  memset(x_expand_map_,0.,sizeof(x_expand_map_));// --[..][2] is ration, must * distance(abs)
  x_expand_map_[0][0] = 1.0f; x_expand_map_[0][1] = 1.5f;  x_expand_map_[0][2] = 0.05f;
  x_expand_map_[1][0] = 1.0f; x_expand_map_[1][1] = 2.0f;  x_expand_map_[1][2] = 0.07f;
  x_expand_map_[2][0] = 1.0f; x_expand_map_[2][1] = 1.5f;  x_expand_map_[2][2] = 0.05f;
  x_expand_map_[3][0] = 0.2f; x_expand_map_[3][1] = 0.2f;  x_expand_map_[3][2] = 0.03f;
  x_expand_map_[4][0] = 1.0f; x_expand_map_[4][1] = 1.5f;  x_expand_map_[4][2] = 0.05f;

  zone_boxes_.resize(2);
  zone_boxes_[0].emplace_back(-safe_x_, -safe_y_,0.f);
  zone_boxes_[0].emplace_back(-safe_x_,  safe_y_,0.f);
  zone_boxes_[0].emplace_back( safe_x_,  safe_y_,0.f);
  zone_boxes_[0].emplace_back( safe_x_, -safe_y_,0.f);

  zone_boxes_[1].emplace_back(-caution_x_, -caution_y_,0.f);
  zone_boxes_[1].emplace_back(-caution_x_,  caution_y_,0.f);
  zone_boxes_[1].emplace_back( caution_x_,  caution_y_,0.f);
  zone_boxes_[1].emplace_back( caution_x_, -caution_y_,0.f);

  safe_boxes_.emplace_back(-safe_x_, -safe_y_);
  safe_boxes_.emplace_back(-safe_x_,  safe_y_);
  safe_boxes_.emplace_back( safe_x_,  safe_y_);
  safe_boxes_.emplace_back( safe_x_, -safe_y_);

  caution_boxes_.emplace_back(-caution_x_, -caution_y_);
  caution_boxes_.emplace_back(-caution_x_,  caution_y_);
  caution_boxes_.emplace_back( caution_x_,  caution_y_);
  caution_boxes_.emplace_back( caution_x_, -caution_y_);
  // clang-format on
}

std::pair<float, float> fs::Expand::at(int zone_x, int zone_y, int label) const
{
  if(zone_x < 0 || zone_x > X_ || zone_y < 0 || zone_y > Y_)
  {
    UERROR << "index out of range";
  }
  return std::make_pair(x_expand_map_[label][zone_x], y_expand_map_[label][zone_y]);
}

std::vector<cv::Point2f> fs::Expand::getPolygonByOval(const ObjectState& ob, float dt, std::string& expand) const
{
  const auto& box_polygon_ego = assembleEgoBox(ob);
  if(box_polygon_ego.empty())
  {
    return {};
  }

  Zone zone_x, zone_y;
  if(isBoxIntersect(getSafeBox(), box_polygon_ego))
  {
    zone_x = Zone::SAFETY;
    zone_y = Zone::SAFETY;
  }
  else if(isBoxIntersect(getCautionBox(), box_polygon_ego))
  {
    zone_x = Zone::CAUTION;
    zone_y = Zone::CAUTION;
  }
  else
  {
    zone_x = Zone::IGNORANCE;
    zone_y = Zone::IGNORANCE;
  }

  const auto& oval_xy = getOvalXY(zone_x, zone_y, ob);
  expand              = Utils::to_string_with_precision(oval_xy.first, 2) + "," + Utils::to_string_with_precision(oval_xy.second, 2);

  return getPolygon(box_polygon_ego, oval_xy.first, oval_xy.second);
}

std::pair<float, float> fs::Expand::getOvalXY(Zone zone_x, Zone zone_y, const ObjectState& ob) const
{
  const auto oval_xy = at(static_cast<int>(zone_x), static_cast<int>(zone_y), static_cast<int>(CLASS_TO_SUBCLASS.at(ob.type)));
  float      oval_x  = oval_xy.first;
  float      oval_y  = oval_xy.second;

  if(zone_x == Zone::IGNORANCE)
  {
    //std::cout << "[" << static_cast<int>(zone_x) << "," << static_cast<int>(zone_y) << "]: " << oval_x << " * " << std::fabs(ob.x) << " = " << oval_x * std::fabs(ob.x) << std::endl;
    oval_x *= std::fabs(ob.x);
    switch(CLASS_TO_SUBCLASS.at(ob.type))
    {
    case SubLabel::BARRICADE:
      oval_x = Utils::clamp(oval_x, 1.0f, 2.0f);
      break;
    default:
      oval_x = Utils::clamp(oval_x, 2.0f, 5.0f);
      break;
    }
  }
  else
  {
    //std::cout << "[" << static_cast<int>(zone_x) << "," << static_cast<int>(zone_y) << "]: " << oval_x << std::endl;
  }

  return std::make_pair(oval_x, oval_y);
}

#else
std::vector<cv::Point2f> fs::Expand::getPolygonByOval(const ObjectState& ob, float dt, std::string& expand) const
{
  const auto& box_polygon_ego = assembleEgoBox(ob);
  if(box_polygon_ego.empty())
  {
    return {};
  }
  float oval_x = 0.f;
  float oval_y = 0.f;
  if(ob.type == GridLabel::CONE || ob.type == GridLabel::PEDESTRIAN || ob.type == GridLabel::CYCLIST)
  {
    oval_x = Utils::interpolate(EXPAND_SMALL_STOPPED, std::fabs(ob.x)) + dt * Utils::interpolate(EXPAND_SMALL_MOVING, std::fabs(ob.x));
    oval_y = Utils::interpolate(EXPAND_SMALL_STOPPED, std::fabs(ob.y)) + dt * Utils::interpolate(EXPAND_SMALL_MOVING, std::fabs(ob.y));
  }
  else
  {
    oval_x = Utils::interpolate(EXPAND_BIG_STOPPED, std::fabs(ob.x)) + dt * Utils::interpolate(EXPAND_BIG_MOVING, std::fabs(ob.x));
    oval_y = Utils::interpolate(EXPAND_BIG_STOPPED, std::fabs(ob.y)) + dt * Utils::interpolate(EXPAND_BIG_MOVING, std::fabs(ob.y));
  }
  expand = Utils::to_string_with_precision(oval_x, 2) + "," + Utils::to_string_with_precision(oval_y, 2);

  return getPolygon(box_polygon_ego, oval_x, oval_y);
}

#endif