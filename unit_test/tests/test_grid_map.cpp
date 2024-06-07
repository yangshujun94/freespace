#include <gtest/gtest.h>

#define private public
#include "core/grid_map.h"
#include "peripheral/fs_math.h"

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
  for(int j = 0; j < GRID_MAP_SIZE_ROWS; ++j)
  {
    const auto pos = GridMap::idx2pos(1, j);
    const auto idx = GridMap::pos2idx(pos);
    EXPECT_EQ(idx.row, j);
  }

  for(int i = 0; i < GRID_MAP_SIZE_COLS; ++i)
  {
    const auto pos = GridMap::idx2pos(i, 1);
    const auto idx = GridMap::pos2idx(pos);
    EXPECT_EQ(idx.col, i);
  }
}

TEST_F(TestSuite_GridMap, Testcase_pos2idx_2)
{
  GridMap gridMap;
}

TEST_F(TestSuite_GridMap, Testcase_pos2idx_3)
{
  GridMap gridMap;
  gridMap.updateEgo(EVector2{-2.f, 2.f}, Eigen::Matrix2f::Identity());

  const auto idx = gridMap.pos2idx(EVector2{1.5f, -1.5f});
  EXPECT_EQ(idx.col, 3);
  EXPECT_EQ(idx.row, 4);
}

TEST_F(TestSuite_GridMap, Testcase_pos2idx_4)
{
  GridMap gridMap;
  gridMap.updateEgo(EVector2{-2.f, 2.f}, Eigen::Matrix2f::Identity());

  const auto idx = gridMap.pos2idx(EVector2{-3.5f, 3.5f});
  EXPECT_EQ(idx.col, 6);
  EXPECT_EQ(idx.row, 1);
}

TEST_F(TestSuite_GridMap, Testcase_pos2idx_5)
{
  GridMap gridMap;
  gridMap.updateEgo(EVector2{-2.f, 2.f}, Eigen::Rotation2Df{M_PI_2f32}.toRotationMatrix());

  const auto idx = gridMap.pos2idx(EVector2{-3.5f, 3.5f});
  EXPECT_EQ(idx.col, 6);
  EXPECT_EQ(idx.row, 2);
}

TEST_F(TestSuite_GridMap, Testcase_idx2pos_1)
{
  GridMap gridMap;

  EVector2 pos = gridMap.idx2pos(70, 10);
  EXPECT_FLOAT_EQ(pos.x(), 3.5f);
  EXPECT_FLOAT_EQ(pos.y(), -2.5f);
}

TEST_F(TestSuite_GridMap, Testcase_idx2pos_2)
{
  GridMap gridMap;
  gridMap.updateEgo(EVector2{-2.f, 2.f}, Eigen::Rotation2Df{0.f}.toRotationMatrix());

  EVector2 pos = gridMap.idx2pos(7, 1);
  EXPECT_FLOAT_EQ(pos.x(), -2.5f);
  EXPECT_FLOAT_EQ(pos.y(), 3.5f);
}

TEST_F(TestSuite_GridMap, Testcase_idx2pos_3)
{
  GridMap gridMap;
  gridMap.updateEgo(EVector2{-2.f, 2.f}, Eigen::Rotation2Df{M_PI_2f32}.toRotationMatrix());

  EVector2 pos = gridMap.idx2pos(7, 1);
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
      //      gridMap.m_grids[j][i].logit = 1.f;
    }
  }

  gridMap.updateEgo(EVector2{-2.f, 2.f}, Eigen::Rotation2Df{0.f}.toRotationMatrix());

  for(int i = 0; i < GRID_MAP_SIZE; ++i)
  {
    for(int j = 0; j < GRID_MAP_SIZE; ++j)
    {
      if(i >= 7 || i <= 0 || j <= 1)
      {
        //        EXPECT_FLOAT_EQ(gridMap.m_grids[j][i].logit, 0.f);
      }
      else
      {
        //        EXPECT_FLOAT_EQ(gridMap.m_grids[j][i].logit, 1.f);
      }
    }
  }
}

TEST_F(TestSuite_GridMap, Testcase_cycleToMax)
{
  float logit = PRIOR_LOGIT;

  int i = 0;
  while(true)
  {
    logit += LOGIT_OBSTACLE_LIDAR;
    ++i;

    std::cout << "logit: " << logit << ", prob: " << sigmoid(logit) << std::endl;

    if(logit > MAX_LOGIT)
    {
      break;
    }
  }

  std::cout << "cycles from init to max: " << i << std::endl;
}

// about 3 cycles
TEST_F(TestSuite_GridMap, Testcase_cycleToOccupied)
{
  float logit = PRIOR_LOGIT;

  int i = 0;
  while(true)
  {
    logit += LOGIT_OBSTACLE_LIDAR;
    ++i;

    std::cout << "logit: " << logit << ", prob: " << sigmoid(logit) << std::endl;

    if(sigmoid(logit) > PROB_OCCUPIED_THRESH)
    {
      break;
    }
  }

  std::cout << "cycles from init to occupied: " << i << std::endl;
}

TEST_F(TestSuite_GridMap, Testcase_cycleToOccupied_lidarNoise)
{
  float logit = PRIOR_LOGIT;

  int i = 0;
  while(true)
  {
    logit += LOGIT_NOISE_LIDAR;
    ++i;

    std::cout << "logit: " << logit << ", prob: " << sigmoid(logit) << std::endl;

    if(sigmoid(logit) > PROB_OCCUPIED_THRESH)
    {
      break;
    }
  }

  std::cout << "cycles from init to occupied: " << i << std::endl;
}

// expected 10 cycles
TEST_F(TestSuite_GridMap, Testcase_cycleToOccupied_lidarObstacle_cameraNoise)
{
  float logit = PRIOR_LOGIT;

  int i = 0;
  while(true)
  {
    logit += LOGIT_OBSTACLE_LIDAR + LOGIT_FREE_CAMERA;
    ++i;

    std::cout << "logit: " << logit << ", prob: " << sigmoid(logit) << std::endl;

    if(sigmoid(logit) > PROB_OCCUPIED_THRESH)
    {
      break;
    }
  }

  std::cout << "cycles from init to occupied: " << i << std::endl;
}

TEST_F(TestSuite_GridMap, Testcase_cycleToOccupied_lidarNoise_cameraNoise)
{
  float logit = PRIOR_LOGIT;

  int i = 0;
  while(true)
  {
    logit += LOGIT_NOISE_LIDAR + LOGIT_FREE_CAMERA;
    ++i;

    std::cout << "logit: " << logit << ", prob: " << sigmoid(logit) << std::endl;

    if(logit < MIN_LOGIT)
    {
      break;
    }
  }

  std::cout << "cycles from init to min logit: " << i << std::endl;
}

// expected: 6 cycles
TEST_F(TestSuite_GridMap, Testcase_cycleToOccupied_101)
{
  float logit = PRIOR_LOGIT;

  int i = 0;
  while(true)
  {
    logit += PRIOR_LOGIT;

    if(0 == i % 2)
    {
      logit += LOGIT_OBSTACLE_LIDAR - PRIOR_LOGIT;
    }
    ++i;

    std::cout << "logit: " << logit << ", prob: " << sigmoid(logit) << std::endl;

    if(sigmoid(logit) > PROB_OCCUPIED_THRESH)
    {
      break;
    }
  }

  std::cout << "cycles from init to occupied: " << i << std::endl;
}
