//
// Created by uto_zzq on 23-8-21.
//

#ifndef UTO_PER_FS_SRC_PERIPHERAL_QUADSINTERSECT_H_
#define UTO_PER_FS_SRC_PERIPHERAL_QUADSINTERSECT_H_
#include "peripheral/defs.h"
namespace fs
{
  class QuadsIntersect
  {
    struct LineSegment
    {
      FSVec2f start, end;

      LineSegment(FSVec2f s, FSVec2f e):
        start(s),
        end(e) {}
    };
    bool onSegment(FSVec2f p, FSVec2f q, FSVec2f r) const;
    int  orientation(FSVec2f p, FSVec2f q, FSVec2f r) const;
    bool doIntersect(FSVec2f p1, FSVec2f q1, FSVec2f p2, FSVec2f q2) const;
    bool isInsideQuad(FSVec2f p, const std::vector<FSVec2f>& quad) const;

  public:
    bool quadsIntersect(const std::vector<FSVec2f>& quad1, const std::vector<FSVec2f>& quad2) const;
  };
} // namespace fs

#endif //UTO_PER_FS_SRC_PERIPHERAL_QUADSINTERSECT_H_
