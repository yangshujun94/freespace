#include "grid_map.h"

void fs::GridMap::reset()
{
  m_yaw = 0.f;
  m_vecEgo2MapLast << HALF_GRID_MAP_SIZE * GRID_SCALE, HALF_GRID_MAP_SIZE * GRID_SCALE;
  m_rotEgo2MapLast << std::cos(m_yaw), -std::sin(m_yaw), std::sin(m_yaw), std::cos(m_yaw);

  for(int j = 0; j < GRID_MAP_SIZE; ++j)
  {
    for(int i = 0; i < GRID_MAP_SIZE; ++i)
    {
      m_grids[j][i] = Grid(0.0f);
    }
  }
}

void fs::GridMap::updateEgo(const FSVec2f& deltaPos, const FSVec2f& vecEgo2Map, const FSMat2x2& rotEgo2Map)
{
  if(std::fabs(deltaPos.x()) > GRID_MAP_SIZE * GRID_SCALE || fabs(deltaPos.y()) > GRID_MAP_SIZE * GRID_SCALE)
  {
    reset();
  }
  else
  {
    FSVec2f       delta_pos_map   = m_rotEgo2MapLast * deltaPos;
    const int     signX           = uto::sgn(delta_pos_map.x());
    const int     signY           = uto::sgn(delta_pos_map.y());
    const int32_t new_indexBeginX = std::floor((m_vecEgo2MapLast.x() + 0. + signX * HALF_GRID_MAP_SIZE * GRID_SCALE) * GRID_SCALE_INV);
    const int32_t new_indexBeginY = std::floor((m_vecEgo2MapLast.y() + 0. + signY * HALF_GRID_MAP_SIZE * GRID_SCALE) * GRID_SCALE_INV);
    const int32_t new_indexEndX   = std::floor((m_vecEgo2MapLast.x() + delta_pos_map.x() + signX * HALF_GRID_MAP_SIZE * GRID_SCALE) * GRID_SCALE_INV);
    const int32_t new_indexEndY   = std::floor((m_vecEgo2MapLast.y() + delta_pos_map.y() + signY * HALF_GRID_MAP_SIZE * GRID_SCALE) * GRID_SCALE_INV);

    if(uto::sgn(new_indexEndX - new_indexBeginX) == signX)
    {
      for(int i = new_indexBeginX; i != new_indexEndX; i += signX)
      {
        const int modI = i & 2047;
        for(int j = 0; j < GRID_MAP_SIZE; ++j)
        {
          m_grids[modI][j] = Grid(0.0f);
        } // -- Grid(0.0f),类内提供初始值
      }
    }
    if(uto::sgn(new_indexEndY - new_indexBeginY) == signY)
    {
      for(int j = new_indexBeginY; j != new_indexEndY; j += signY)
      {
        const int modJ = j & 2047;
        for(int i = 0; i < GRID_MAP_SIZE; ++i)
        {
          m_grids[i][modJ] = Grid(0.0f);
        }
      }
    }
  }
  m_vecEgo2MapLast.x() = Utils::mod(vecEgo2Map.x());
  m_vecEgo2MapLast.y() = Utils::mod(vecEgo2Map.y());

  m_vecEgo2MapLastIdx = m_vecEgo2MapLast * GRID_SCALE_INV;

  // -- simplify mod, which cover +%+  -%+
  mod_flag_x_ = vecEgo2Map.x() > 0 ? -1 : 1;
  mod_flag_y_ = vecEgo2Map.y() > 0 ? -1 : 1;
  mod_num_x_  = int(std::fabs(vecEgo2Map.x() - m_vecEgo2MapLast.x())) / (GRID_MAP_SIZE * GRID_SCALE);
  mod_num_y_  = int(std::fabs(vecEgo2Map.y() - m_vecEgo2MapLast.y())) / (GRID_MAP_SIZE * GRID_SCALE);

  m_rotEgo2MapLast = rotEgo2Map;
  m_rotMapLast2Ego = m_rotEgo2MapLast.transpose();
}

fs::GridMap::Index fs::GridMap::pos2ModIdx(const fs::FSVec2f& pointEgo) const
{
  const auto pointMap = m_rotEgo2MapLast * pointEgo + m_vecEgo2MapLast;

  return Index{static_cast<int>(std::round(pointMap.x() * GRID_SCALE_INV)) & 2047,
               static_cast<int>(std::round(pointMap.y() * GRID_SCALE_INV)) & 2047};
  //  return modN(Index{static_cast<int>(std::round(pointMap.x() * GRID_SCALE_INV)),
  //                    static_cast<int>(std::round(pointMap.y() * GRID_SCALE_INV))});
}
fs::FSVec2f fs::GridMap::pos2World(const fs::FSVec2f& pointEgo) const
{
  return m_rotEgo2MapLast * pointEgo + m_vecEgo2MapLast;
}

fs::FSVec2f fs::GridMap::modIdx2PosCurrent(const fs::GridMap::Index& idx) const
{
  const auto& idx_ego_center = modIdx2EgoCenterIdxMap(idx);
  return m_rotMapLast2Ego * (idx_ego_center * GRID_SCALE);
}

fs::FSVec2f fs::GridMap::egoCenterIdx2Pos(const fs::FSVec2f& idx_ego_center) const
{
  return m_rotMapLast2Ego * (idx_ego_center * GRID_SCALE);
}

fs::FSVec2f fs::GridMap::modIdx2EgoCenterIdxMap(const fs::GridMap::Index& idx) const
{
  FSVec2f grid{idx.row, idx.col};
  FSVec2f idx_new = grid - m_vecEgo2MapLastIdx;

  if(idx_new.x() > HALF_GRID_MAP_SIZE)
  {
    idx_new.x() -= GRID_MAP_SIZE;
  }
  else if(idx_new.x() < -HALF_GRID_MAP_SIZE)
  {
    idx_new.x() += GRID_MAP_SIZE;
  }
  else {} // remain the value

  if(idx_new.y() > HALF_GRID_MAP_SIZE)
  {
    idx_new.y() -= GRID_MAP_SIZE;
  }
  else if(idx_new.y() < -HALF_GRID_MAP_SIZE)
  {
    idx_new.y() += GRID_MAP_SIZE;
  }
  else {} // remain the value

  // -- current grid is restored，without mod
  return idx_new;
}

fs::FSVec2f fs::GridMap::modIdx2EgoCenterIdxCurrent(const fs::GridMap::Index& idx) const
{
  FSVec2f grid{idx.row, idx.col};
  FSVec2f idx_new = grid - m_vecEgo2MapLastIdx;

  if(idx_new.x() > HALF_GRID_MAP_SIZE)
  {
    idx_new.x() -= GRID_MAP_SIZE;
  }
  else if(idx_new.x() < -HALF_GRID_MAP_SIZE)
  {
    idx_new.x() += GRID_MAP_SIZE;
  }
  else {} // remain the value

  if(idx_new.y() > HALF_GRID_MAP_SIZE)
  {
    idx_new.y() -= GRID_MAP_SIZE;
  }
  else if(idx_new.y() < -HALF_GRID_MAP_SIZE)
  {
    idx_new.y() += GRID_MAP_SIZE;
  }
  else {} // remain the value

  // -- current grid is restored，without mod
  return m_rotMapLast2Ego * idx_new;
}
