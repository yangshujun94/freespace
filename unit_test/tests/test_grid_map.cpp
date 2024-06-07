#include <gtest/gtest.h>

#define private public
#include "core/grid_map.h"

using namespace fs;

// set GRID_MAP_SIZE = 8
// set GRID_SCALE = 1.F
class TestSuite_GridMap : public testing::Test
{
protected:
  void SetUp() override {}

  void TearDown() override {}
};

TEST_F(TestSuite_GridMap, Testcase_pos2idx_1)
{
  GridMap gridMap;

  const auto idx = gridMap.pos2idx(EVector2{3.5f, -3.5f});
  EXPECT_EQ(idx.col, 7);
  EXPECT_EQ(idx.row, 0);
}

TEST_F(TestSuite_GridMap, Testcase_pos2idx_2)
{
  GridMap gridMap;

  const auto idx = gridMap.pos2idx(EVector2{-0.5f, 0.5f});
  EXPECT_EQ(idx.col, 3);
  EXPECT_EQ(idx.row, 4);
}

TEST_F(TestSuite_GridMap, Testcase_pos2idx_3)
{
  GridMap gridMap;
  gridMap.updateEgo(EVector2{-2.f, 2.f}, 0.f);

  const auto idx = gridMap.pos2idx(EVector2{1.5f, -1.5f});
  EXPECT_EQ(idx.col, 3);
  EXPECT_EQ(idx.row, 4);
}

TEST_F(TestSuite_GridMap, Testcase_pos2idx_4)
{
  GridMap gridMap;
  gridMap.updateEgo(EVector2{-2.f, 2.f}, 0.f);

  const auto idx = gridMap.pos2idx(EVector2{-3.5f, 3.5f});
  EXPECT_EQ(idx.col, 6);
  EXPECT_EQ(idx.row, 1);
}

TEST_F(TestSuite_GridMap, Testcase_pos2idx_5)
{
  GridMap gridMap;
  gridMap.updateEgo(EVector2{-2.f, 2.f}, M_PI_2f32);

  const auto idx = gridMap.pos2idx(EVector2{-3.5f, 3.5f});
  EXPECT_EQ(idx.col, 6);
  EXPECT_EQ(idx.row, 2);
}

TEST_F(TestSuite_GridMap, Testcase_idx2pos_1)
{
  GridMap gridMap;

  EVector2 pos = gridMap.idx2pos(GridMap::Index{70, 10});
  EXPECT_FLOAT_EQ(pos.x(), 3.5f);
  EXPECT_FLOAT_EQ(pos.y(), -2.5f);
}

TEST_F(TestSuite_GridMap, Testcase_idx2pos_2)
{
  GridMap gridMap;
  gridMap.updateEgo(EVector2{-2.f, 2.f}, 0.f);

  EVector2 pos = gridMap.idx2pos(GridMap::Index{7, 1});
  EXPECT_FLOAT_EQ(pos.x(), -2.5f);
  EXPECT_FLOAT_EQ(pos.y(), 3.5f);
}

TEST_F(TestSuite_GridMap, Testcase_idx2pos_3)
{
  GridMap gridMap;
  gridMap.updateEgo(EVector2{-2.f, 2.f}, M_PI_2f32);

  EVector2 pos = gridMap.idx2pos(GridMap::Index{7, 1});
  EXPECT_FLOAT_EQ(pos.y(), 2.5f);
  EXPECT_FLOAT_EQ(pos.x(), 3.5f);
}

TEST_F(TestSuite_GridMap, Testcase_updateEgo)
{
  GridMap gridMap;
  for(int i = 0; i < GRID_MAP_SIZE; ++i)
  {
    for(int j = 0; j < GRID_MAP_SIZE; ++j)
    {
      gridMap.m_grids[j][i].logit = 1.f;
    }
  }

  gridMap.updateEgo(EVector2{-2.f, 2.f}, 0.f);

  for(int i = 0; i < GRID_MAP_SIZE; ++i)
  {
    for(int j = 0; j < GRID_MAP_SIZE; ++j)
    {
      if(i >= 7 || i <= 0 || j <= 1)
      {
        EXPECT_FLOAT_EQ(gridMap.m_grids[j][i].logit, 0.f);
      }
      else
      {
        EXPECT_FLOAT_EQ(gridMap.m_grids[j][i].logit, 1.f);
      }
    }
  }
}
