#include "grid_line_traversal.h"

std::vector<Eigen::Vector2i> fs::GridLineTraversal::gridLine(const Eigen::Vector2i& start, const Eigen::Vector2i& end)
{
  std::vector<Eigen::Vector2i> points = gridLineCore(start, end);

  if(start.x() != points[0].x() || start.y() != points[0].y())
  {
    for(size_t i = 0, j = points.size() - 1; i < points.size() / 2; i++, j--)
    {
      points[i].swap(points[j]);
    }
  }

  return points;
}

std::vector<Eigen::Vector2i> fs::GridLineTraversal::gridLineCore(const Eigen::Vector2i& start, const Eigen::Vector2i& end)
{
  const int dx = std::abs(end.x() - start.x());
  const int dy = std::abs(end.y() - start.y());

  std::vector<Eigen::Vector2i> points;
  points.reserve(2 * std::max(dx, dy));

  int incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;

  if(dy <= dx)
  {
    d     = 2 * dy - dx;
    incr1 = 2 * dy;
    incr2 = 2 * (dy - dx);
    if(start.x() > end.x())
    {
      x        = end.x();
      y        = end.y();
      ydirflag = -1;
      xend     = start.x();
    }
    else
    {
      x        = start.x();
      y        = start.y();
      ydirflag = 1;
      xend     = end.x();
    }
    points.emplace_back(x, y);
    if((end.y() - start.y()) * ydirflag > 0)
    {
      while(x < xend)
      {
        x++;
        if(d < 0)
        {
          d += incr1;
        }
        else
        {
          y++;
          d += incr2;
        }
        points.emplace_back(x, y);
      }
    }
    else
    {
      while(x < xend)
      {
        x++;
        if(d < 0)
        {
          d += incr1;
        }
        else
        {
          y--;
          d += incr2;
        }
        points.emplace_back(x, y);
      }
    }
  }
  else
  {
    d     = 2 * dx - dy;
    incr1 = 2 * dx;
    incr2 = 2 * (dx - dy);
    if(start.y() > end.y())
    {
      y        = end.y();
      x        = end.x();
      yend     = start.y();
      xdirflag = -1;
    }
    else
    {
      y        = start.y();
      x        = start.x();
      yend     = end.y();
      xdirflag = 1;
    }
    points.emplace_back(x, y);
    if((end.x() - start.x()) * xdirflag > 0)
    {
      while(y < yend)
      {
        y++;
        if(d < 0)
        {
          d += incr1;
        }
        else
        {
          x++;
          d += incr2;
        }
        points.emplace_back(x, y);
      }
    }
    else
    {
      while(y < yend)
      {
        y++;
        if(d < 0)
        {
          d += incr1;
        }
        else
        {
          x--;
          d += incr2;
        }
        points.emplace_back(x, y);
      }
    }
  }

  return points;
}
