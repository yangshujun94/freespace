#ifndef GRID_MAP_H_
#define GRID_MAP_H_

#include <common/math/math.h>
#include "peripheral/macro.h"
#include "peripheral/defs.h"
#include "peripheral/utils.h"

namespace fs
{
  struct Grid
  {
    float       logit;      // --  0.0f;
    GridLabel   grid_label; // --  GridLabel::UNKNOWN;
    bool        is_noise;
    MotionState motion_state; // --  MotionState::STOPPED;
    int16_t     id;           // --  -1;
    float       top;          // --  -1;
    float       bottom;       // --  -1;
    float       ground;       // --  -1;
    bool        tpts;         // --  false;
    int         row_test;
    int         col_test;
    float       distance_x;
    float       distance_y;
    int64_t     vot_timestamp;
    Grid() = default;
    Grid(float _logit):
      logit(_logit),
      grid_label(GridLabel::UNKNOWN),
      is_noise(false),
      motion_state(MotionState::STOPPED),
      id(ID_DEFAULT),
      top(0.f),
      bottom(0.f),
      ground(0.f),
      tpts(false),
      row_test(0),
      col_test(0),
      distance_x(0.f),
      distance_y(0.f),
      vot_timestamp(0) {}
    Grid(GridLabel label):
      logit(0),
      grid_label(label),
      is_noise(false),
      motion_state(MotionState::STOPPED),
      id(ID_DEFAULT),
      top(0.f),
      bottom(0.f),
      ground(0.f),
      tpts(false),
      row_test(0),
      col_test(0),
      distance_x(0.f),
      distance_y(0.f),
      vot_timestamp(0) {}
  };

  class GridMap
  {
  public:
    struct Index
    {
      int row = 0;
      int col = 0;
      Index() = default;

      Index(int _row, int _col):
        row(_row),
        col(_col) {}

      Index operator+(const Index& idx)
      {
        this->row += idx.row;
        this->col += idx.col;
        return *this;
      }
      Index operator+=(const Index& idx)
      {
        this->row += idx.row;
        this->col += idx.col;
        return *this;
      }
      Index operator-(const Index& idx)
      {
        this->row -= idx.row;
        this->col -= idx.col;
        return *this;
      }
      Index operator-=(const Index& idx)
      {
        this->row -= idx.row;
        this->col -= idx.col;
        return *this;
      }
      friend std::ostream& operator<<(std::ostream& out, const Index& idx)
      {
        out << " [" << idx.row << "," << idx.col << "] ";
        return out;
      }
    };

    DISALLOW_COPY_MOVE_AND_ASSIGN(GridMap);

    GridMap() { reset(); }

    void updateEgo(const FSVec2f& deltaPos, const FSVec2f& vecEgo2Map, const FSMat2x2& rotEgo2Map);

    Index   pos2ModIdx(const FSVec2f& pointEgo) const;
    FSVec2f pos2World(const FSVec2f& pointEgo) const;
    FSVec2f modIdx2PosCurrent(const Index& idx) const;
    FSVec2f egoCenterIdx2Pos(const fs::FSVec2f& idx_ego_center) const;
    FSVec2f modIdx2EgoCenterIdxMap(const fs::GridMap::Index& idx) const;
    FSVec2f modIdx2EgoCenterIdxCurrent(const fs::GridMap::Index& idx) const;

    Grid&       at(const Index& idx) { return m_grids[idx.row][idx.col]; }
    const Grid& at(const Index& idx) const { return m_grids[idx.row][idx.col]; }
    Grid&       at(const int& row, const int& col) { return m_grids[row][col]; }
    const Grid& at(const int& row, const int& col) const { return m_grids[row][col]; }

  private:
    using Grids = Grid[GRID_MAP_SIZE][GRID_MAP_SIZE];

    void reset();

    [[nodiscard]] FSVec2f mod(const FSVec2f& value) const
    {
      float x = value.x() + mod_flag_x_ * (mod_num_x_ * GRID_MAP_M);
      float y = value.y() + mod_flag_y_ * (mod_num_y_ * GRID_MAP_M);
      while(x >= GRID_MAP_M)
      {
        x -= GRID_MAP_M;
      }
      while(y >= GRID_MAP_M)
      {
        y -= GRID_MAP_M;
      }
      while(x < 0)
      {
        x += GRID_MAP_M;
      }
      while(y < 0)
      {
        y += GRID_MAP_M;
      }
      return FSVec2f{x, y};
    }
    [[nodiscard]] Index modN(const Index& idx) const
    {
      int x = idx.row + mod_flag_x_ * (mod_num_x_ * GRID_MAP_SIZE);
      int y = idx.col + mod_flag_y_ * (mod_num_y_ * GRID_MAP_SIZE);
      while(x >= GRID_MAP_SIZE)
      {
        x -= GRID_MAP_SIZE;
      }
      while(y >= GRID_MAP_SIZE)
      {
        y -= GRID_MAP_SIZE;
      }
      while(x < 0)
      {
        x += GRID_MAP_SIZE;
      }
      while(y < 0)
      {
        y += GRID_MAP_SIZE;
      }
      return Index{x, y};
    }

    Grids    m_grids;
    float    m_yaw{};
    FSVec2f  m_vecEgo2MapLast{};
    FSVec2f  m_vecEgo2MapLastIdx{};
    FSMat2x2 m_rotEgo2MapLast{};
    FSMat2x2 m_rotMapLast2Ego{};
    int      mod_num_x_  = 0;
    int      mod_num_y_  = 0;
    int      mod_flag_x_ = 0;
    int      mod_flag_y_ = 0;
  };
} // namespace fs

#endif //GRID_MAP_H_
