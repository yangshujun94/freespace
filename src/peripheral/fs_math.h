#ifndef FS_MATH_H_
#define FS_MATH_H_

namespace fs
{
  template<typename T, template<typename U> class CONT>
  inline constexpr bool pointInPolygon(const Eigen::Matrix<T, 2, 1>& point, const CONT<Eigen::Matrix<T, 2, 1>>& polygon)
  {
    bool oddNodes = false;

    for(std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
    {
      if(((polygon[i].x() > point.x()) != (polygon[j].x() > point.x())) &&
         (point.y() < (polygon[j].y() - polygon[i].y()) * (point.x() - polygon[i].x()) / (polygon[j].x() - polygon[i].x()) + polygon[i].y()))
      {
        oddNodes = !oddNodes;
      }
    }

    return oddNodes;
  }

  inline std::vector<Eigen::Vector2f> convexHull(std::vector<Eigen::Vector2f>& points)
  {
    std::vector<Eigen::Vector2f> hull;
    hull.clear();
    hull.reserve(points.size());

    if(!points.empty())
    {
      const int n = points.size();
      std::sort(points.begin(), points.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) { return a.x() == b.x() ? a.y() < b.y() : a.x() < b.x(); });

      for(int i = 0; i < n; i++)
      {
        const Eigen::Vector3f point(points[i].x(), points[i].y(), 0.0f);
        while(hull.size() >= 2 && (Eigen::Vector3f{hull[hull.size() - 2].x(), hull[hull.size() - 2].y(), 0.0f} - Eigen::Vector3f{hull[hull.size() - 1].x(), hull[hull.size() - 1].y(), 0.0f}).cross(point - Eigen::Vector3f{hull[hull.size() - 1].x(), hull[hull.size() - 1].y(), 0.0f}).z() <= 0.f)
        {
          hull.pop_back();
        }
        hull.push_back(points[i]);
      }

      const int t = hull.size();
      for(int i = n - 2; i >= 0; i--)
      {
        const Eigen::Vector3f point(points[i].x(), points[i].y(), 0.0f);
        while(hull.size() > t && (Eigen::Vector3f{hull[hull.size() - 2].x(), hull[hull.size() - 2].y(), 0.0f} - Eigen::Vector3f{hull[hull.size() - 1].x(), hull[hull.size() - 1].y(), 0.0f}).cross(point - Eigen::Vector3f{hull[hull.size() - 1].x(), hull[hull.size() - 1].y(), 0.0f}).z() <= 0.f)
        {
          hull.pop_back();
        }
        hull.push_back(points[i]);
      }
      hull.pop_back();
    }

    return hull;
  }
} // namespace fs

#endif //FS_MATH_H_
