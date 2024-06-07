//
// Created by uto_zzq on 23-8-21.
//

#include "quads_intersect.h"
namespace fs
{
  bool QuadsIntersect::onSegment(FSVec2f p, FSVec2f q, FSVec2f r)const
  {
    return (q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
            q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y()));
  }

  int QuadsIntersect::orientation(FSVec2f p, FSVec2f q, FSVec2f r)const
  {
    double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
    if(val == 0)
      return 0;
    return (val > 0) ? 1 : 2;
  }

  bool QuadsIntersect::doIntersect(FSVec2f p1, FSVec2f q1, FSVec2f p2, FSVec2f q2)const
  {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if(o1 != o2 && o3 != o4)
      return true;
    if(o1 == 0 && onSegment(p1, p2, q1))
      return true;
    if(o2 == 0 && onSegment(p1, q2, q1))
      return true;
    if(o3 == 0 && onSegment(p2, p1, q2))
      return true;
    if(o4 == 0 && onSegment(p2, q1, q2))
      return true;

    return false;
  }

  bool QuadsIntersect::isInsideQuad(FSVec2f p, const std::vector<FSVec2f>& quad)const
  {
    int n = quad.size();
    if(n < 3)
      return false;

    FSVec2f extreme = {1e9, p.y()};

    int count = 0, i = 0;
    do
    {
      int next = (i + 1) % n;

      if(doIntersect(quad[i], quad[next], p, extreme))
      {
        if(orientation(quad[i], p, quad[next]) == 0)
          return onSegment(quad[i], p, quad[next]);

        count++;
      }
      i = next;
    } while(i != 0);

    return (count & 1); // 如果计数为奇数，返回true
  }

  bool QuadsIntersect::quadsIntersect(const std::vector<FSVec2f>& quad1, const std::vector<FSVec2f>& quad2)const
  {
    for(int i = 0; i < 4; i++)
    {
      for(int j = 0; j < 4; j++)
      {
        if(doIntersect(quad1[i], quad1[(i + 1) % 4], quad2[j], quad2[(j + 1) % 4]))
          return true;
      }
    }

    if(isInsideQuad(quad1[0], quad2) || isInsideQuad(quad2[0], quad1))
      return true;

    return false;
  }
}
