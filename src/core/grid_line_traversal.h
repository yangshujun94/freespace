#ifndef GRID_LINE_TRAVERSAL_H_
#define GRID_LINE_TRAVERSAL_H_

#include <Eigen/Dense>

namespace fs
{
  class GridLineTraversal
  {
  public:
    static std::vector<Eigen::Vector2i> gridLine(const Eigen::Vector2i& start, const Eigen::Vector2i& end);

  private:
    static std::vector<Eigen::Vector2i> gridLineCore(const Eigen::Vector2i& start, const Eigen::Vector2i& end);
  };
} // namespace fs

#endif //GRID_LINE_TRAVERSAL_H_
