#ifndef OBJECT_H_
#define OBJECT_H_

#include <common/macros.h>
#include <perception_obstacles.pb.h>
#include "peripheral/types.h"

namespace fs
{
  class Object
  {
  public:
    Object(const uto::proto::PerceptionObstacle& fusedObj, const float deltaTime, const EMatrix2& rotTitoT0, const EVector2& vecTitoT0);

    ObstacleClass getClass() const { return m_class; }
    bool          isInsidePoint(const EVector2& point) const;

    std::tuple<EVector2, EVector2> calcMinBoundingRect() const;

  private:
    void dilateContour(const EVector2& dilationSize);

    bool       isInRect(const Rect& rect) const;
    SafetyZone determineSafetyZone() const;
    EVector2   calcDilationSize() const;

    int           m_id    = 0;
    ObstacleClass m_class = ObstacleClass::MAX_NUM;
    float         m_minX  = FLT_MAX;
    float         m_maxX  = -FLT_MAX;
    float         m_minY  = FLT_MAX;
    float         m_maxY  = -FLT_MAX;

    std::vector<EVector2> m_contour{};
    std::vector<EVector2> m_dilatedContour{};
  };

  using ObjectList = std::vector<Object>;
} // namespace fs

#endif //OBJECT_H_
