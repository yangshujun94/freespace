#ifndef GRID_MAP_H_
#define GRID_MAP_H_

#include <common/math/math.h>
#include <peripheral/ring_buffer.h>
#include "peripheral/macro.h"
#include "peripheral/defs.h"
#include "peripheral/types.h"

namespace fs
{
  class GridMap
  {
  public:
    struct Index
    {
      int row = 0U;
      int col = 0U;
    };

    DISALLOW_COPY_MOVE_AND_ASSIGN(GridMap);

    GridMap() { reset(); }

    void reset();
    void updateEgo();
    void applyPrior();
    void extractBorders();
    bool isToPublish() const { return m_isToPublish; }
    void setNoPublish() { m_isToPublish = false; }

    static Index pos2idx(const EVector2& pointEgo)
    {
      return {EGO_INDEX_ROW - static_cast<int>(std::round(pointEgo.x() * GRID_SCALE_INV)),
              EGO_INDEX_COL - static_cast<int>(std::round(pointEgo.y() * GRID_SCALE_INV))};
    }
    static EVector2 idx2pos(const int row, const int col)
    {
      return EVector2{GRID_SCALE * static_cast<float>(EGO_INDEX_ROW - row),
                      GRID_SCALE * static_cast<float>(EGO_INDEX_COL - col)};
    }

    Grid&       at(const int row, const int col) { return m_gridMapBuffer.front()[row][col]; }
    const Grid& at(const int row, const int col) const { return m_gridMapBuffer.front()[row][col]; }

  private:
    using Grids = Grid[GRID_MAP_SIZE_ROWS][GRID_MAP_SIZE_COLS];
    using Cells = BorderType[CELL_ROWS][CELL_COLS];

    fs::RingBuffer<Grids, 4> m_gridMapBuffer{};
    Cells                    m_cells{};
    bool                     m_isToPublish = true;
  };
} // namespace fs

#endif //GRID_MAP_H_
