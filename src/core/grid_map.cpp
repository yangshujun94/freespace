#include "grid_map.h"
#include "peripheral/vehicle.h"

void fs::GridMap::reset()
{
  m_isToPublish = true;
  m_gridMapBuffer.pushFrontForce();

  auto& newGrid = m_gridMapBuffer.front();
  for(int j = 0; j < GRID_MAP_SIZE_ROWS; ++j)
  {
    for(int i = 0; i < GRID_MAP_SIZE_COLS; ++i)
    {
      auto& grid       = newGrid[j][i];
      grid.logit       = PRIOR_LOGIT;
      grid.point       = idx2pos(j, i);
      grid.locked      = false;
      grid.isFreeInCam = false;
      grid.isBorder    = true;
      grid.top         = 0.0f;
      grid.ground      = 0.0f;
      grid.bottom      = 0.0f;
#if FS_CHECK(CFG_DEBUG_GRID)
      grid.history    = 0;
      grid.logitPrev  = 0;
      grid.logitLidar = 0;
      grid.rowPrev    = 0;
      grid.colPrev    = 0;
#endif
    }
  }
}

void fs::GridMap::updateEgo()
{
  m_gridMapBuffer.pushFrontForce();
  auto& oldGrids = m_gridMapBuffer[1];
  auto& newGrids = m_gridMapBuffer[0];

  for(int j = 0; j < GRID_MAP_SIZE_ROWS; ++j)
  {
    for(int i = 0; i < GRID_MAP_SIZE_COLS; ++i)
    {
      auto& newGrid = newGrids[j][i];
      newGrid.logit = MIN_LOGIT;
      newGrid.point = idx2pos(j, i);
    }
  }

  const Vehicle& vehicle = Vehicle::getVehicle();
  const EMatrix2 rot     = vehicle.getRotT1toT0().topLeftCorner<2, 2>();
  const EVector2 vec     = vehicle.getVecT1toT0().head<2>();

  for(int j = 0; j < GRID_MAP_SIZE_ROWS; ++j)
  {
    for(int i = 0; i < GRID_MAP_SIZE_COLS; ++i)
    {
      const Grid&          oldGrid  = oldGrids[j][i];
      const EVector2       pointNew = rot * oldGrid.point + vec;
      const GridMap::Index idxNew   = pos2idx(pointNew);

      if(idxNew.row < GRID_MAP_SIZE_ROWS && idxNew.col < GRID_MAP_SIZE_COLS && idxNew.row >= 0 && idxNew.col >= 0 && newGrids[idxNew.row][idxNew.col].logit < oldGrid.logit)
      {
        auto& newGrid = newGrids[idxNew.row][idxNew.col];
        newGrid       = oldGrid;
        newGrid.point = pointNew;
#if FS_CHECK(CFG_DEBUG_GRID)
        newGrid.rowPrev = j;
        newGrid.colPrev = i;
#endif
      }
    }
  }
}

void fs::GridMap::applyPrior()
{
  m_isToPublish = true;
  for(auto& row : m_gridMapBuffer.front())
  {
    for(auto& col : row)
    {
#if FS_CHECK(CFG_DEBUG_GRID)
      col.history   = (col.history * 10) % 100000000UL;
      col.logitPrev = col.logit;
#endif
      col.logit       = std::max(col.logit + PRIOR_LOGIT, MIN_LOGIT);
      col.locked      = false;
      col.isFreeInCam = false;
      col.isBorder    = true;
    }
  }
}

void fs::GridMap::extractBorders()
{
  Grids& grids = m_gridMapBuffer.front();

#if FS_CHECK(CFG_PUB_BORDER)
  memset(m_cells, BorderType::BORDER_TYPE_FREE, sizeof(m_cells));

  for(int j = 0; j < GRID_MAP_SIZE_ROWS; ++j)
  {
    const int jCell = j * CELL_SCALE_INV;
    for(int i = 0; i < GRID_MAP_SIZE_COLS; ++i)
    {
      const int iCell = i * CELL_SCALE_INV;
      if(BorderType::BORDER_TYPE_FREE == m_cells[jCell][iCell] && grids[j][i].logit > LOGIT_OCCUPIED_THRESH)
      {
        m_cells[jCell][iCell] = BorderType::BORDER_TYPE_OCCUPIED;
      }
    }
  }

  for(int j = 2; j < CELL_ROWS - 2; ++j)
  {
    for(int i = 2; i < CELL_COLS - 2; ++i)
    {
      if(BorderType::BORDER_TYPE_OCCUPIED == m_cells[j][i])
      {
        if((BorderType::BORDER_TYPE_FREE == m_cells[j + 0][i - 2] && BorderType::BORDER_TYPE_FREE == m_cells[j + 0][i - 1]) ||
           (BorderType::BORDER_TYPE_FREE == m_cells[j + 0][i + 2] && BorderType::BORDER_TYPE_FREE == m_cells[j + 0][i + 1]) ||
           (BorderType::BORDER_TYPE_FREE == m_cells[j - 2][i + 0] && BorderType::BORDER_TYPE_FREE == m_cells[j - 1][i + 0]) ||
           (BorderType::BORDER_TYPE_FREE == m_cells[j + 2][i + 0] && BorderType::BORDER_TYPE_FREE == m_cells[j + 1][i + 0]))
        {
          m_cells[j][i] = BorderType::BORDER_TYPE_CONTOUR;
        }
      }
    }
  }
#endif

  for(int j = 0; j < GRID_MAP_SIZE_ROWS; ++j)
  {
    const int jCell = j * CELL_SCALE_INV;
    for(int i = 0; i < GRID_MAP_SIZE_COLS; ++i)
    {
      const int iCell = i * CELL_SCALE_INV;
#if FS_CHECK(CFG_PUB_BORDER)
      grids[j][i].isBorder = BorderType::BORDER_TYPE_CONTOUR == m_cells[jCell][iCell]
#else
      grids[j][i].isBorder = true;
#endif
        ;
    }
  }
}
