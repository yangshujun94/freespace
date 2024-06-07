#include "object.h"
#include "peripheral/vehicle.h"
#include "peripheral/fs_math.h"
#include "peripheral/utils.h"
#include "debug/fs_log.h"

fs::Object::Object(const uto::proto::PerceptionObstacle& fusedObj, const float deltaTime, const EMatrix2& rotTitoT0, const EVector2& vecTitoT0)
{
  assert(!fusedObj.sub_obstacles().empty());
  m_class = perceptionClass2ObjectClass(fusedObj.sub_obstacles(0).obstacle_type());
  m_id    = fusedObj.id();

  if(ObstacleClass::CRANE == m_class && uto::proto::PerceptionObstacle_ObstacleMoveStatus_MOVING != fusedObj.move_status())
  {
    m_class = ObstacleClass::STATIC_OBJECT;
  }

  m_contour.reserve(4 * fusedObj.sub_obstacles_size());
  for(const auto& subObj : fusedObj.sub_obstacles())
  {
    // predict object to current timestamp
    const float xCenterEgo1 = subObj.position().x() + subObj.velocity().x() * deltaTime;
    const float yCenterEgo1 = subObj.position().y() + subObj.velocity().y() * deltaTime;

    const std::array<EVector2, 4> shapePoints = Utils::createCuboid(xCenterEgo1, yCenterEgo1, subObj.length(), subObj.width(), subObj.heading_angle());
    m_contour.insert(m_contour.cend(), shapePoints.cbegin(), shapePoints.cend());
  }

  // transform to current ego coordinate system
  for(auto& point : m_contour)
  {
    point = rotTitoT0 * point + vecTitoT0;

    m_minX = std::min(m_minX, point.x());
    m_maxX = std::max(m_maxX, point.x());
    m_minY = std::min(m_minY, point.y());
    m_maxY = std::max(m_maxY, point.y());
  }

  const EVector2 dilationSize = calcDilationSize();
  dilateContour(dilationSize);
}

void fs::Object::dilateContour(const EVector2& dilationSize)
{
  std::vector<EVector2> expandedPolygon{};
  expandedPolygon.reserve(m_contour.size() * 8);

  constexpr float thetaStep = M_PIf32 / 4.f;
  float           theta     = 0.f;

  while(theta < 2.f * M_PIf32)
  {
    theta += thetaStep;
    const float deltaX = dilationSize.x() * std::sin(theta);
    const float deltaY = dilationSize.y() * std::cos(theta);
    for(const auto& p : m_contour)
    {
      expandedPolygon.emplace_back(p.x() + deltaX, p.y() + deltaY);
    }
  }

  m_dilatedContour = convexHull(expandedPolygon);
}

bool fs::Object::isInsidePoint(const EVector2& point) const
{
  return pointInPolygon(EVector2{point.x(), point.y()}, m_dilatedContour);
}

fs::SafetyZone fs::Object::determineSafetyZone() const
{
  SafetyZone zone = SafetyZone::MAX_NUM;

  const Vehicle& vehicle     = Vehicle::getVehicle();
  const Rect&    dangerZone  = vehicle.getDangerZone();
  const Rect&    cautionZone = vehicle.getCautionZone();

  if(isInRect(dangerZone))
  {
    zone = SafetyZone::DANGER;
  }
  else if(isInRect(cautionZone))
  {
    zone = SafetyZone::CAUTION;
  }
  else
  {
    zone = SafetyZone::IGNORANCE;
  }

  return zone;
}

bool fs::Object::isInRect(const fs::Rect& rect) const
{
  return std::min(rect.maxX, m_maxX) > std::max(rect.minX, m_minX) &&
         std::min(rect.maxY, m_maxY) > std::max(rect.minY, m_minY);
}

fs::EVector2 fs::Object::calcDilationSize() const
{
  EVector2 dilationSize{};

  const SafetyZone safetyZone = determineSafetyZone();
  switch(safetyZone)
  {
  case SafetyZone::DANGER:
    dilationSize.x() = DANGER_ZONE_DILATION_X.at(m_class);
    dilationSize.y() = DANGER_ZONE_DILATION_Y.at(m_class);
    break;
  case SafetyZone::CAUTION:
    dilationSize.x() = CAUTION_ZONE_DILATION_X.at(m_class);
    dilationSize.y() = CAUTION_ZONE_DILATION_Y.at(m_class);
    break;
  case SafetyZone::IGNORANCE:
    dilationSize.x() = std::clamp(std::min(std::abs(m_minX), std::abs(m_maxX)) * IGNORANCE_ZONE_DILATION_X_FACTOR.at(m_class), IGNORANCE_ZONE_DILATION_X_LOW.at(m_class), IGNORANCE_ZONE_DILATION_X_HIGH.at(m_class));
    dilationSize.y() = IGNORANCE_ZONE_DILATION_Y.at(m_class);
    break;
  default:
    assert(false && "unknown zone");
  }

  LOG_DEBUG("[id: %d][safety zone: %s]", m_id, SAFETY_ZONE_TO_STRING.at(safetyZone).c_str());

  return dilationSize;
}

std::tuple<fs::EVector2, fs::EVector2> fs::Object::calcMinBoundingRect() const
{
  float minX = FLT_MAX;
  float maxX = -FLT_MAX;
  float minY = FLT_MAX;
  float maxY = -FLT_MAX;
  for(const auto& point : m_dilatedContour)
  {
    minX = std::min(minX, point.x());
    maxX = std::max(maxX, point.x());
    minY = std::min(minY, point.y());
    maxY = std::max(maxY, point.y());
  }
  return {EVector2{maxX, maxY}, EVector2{minX, minY}};
}
