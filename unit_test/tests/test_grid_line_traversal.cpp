#include <gtest/gtest.h>

#define private public
#include "core/grid_line_traversal.h"

using namespace fs;

class TestSuite_GridLineTraversal : public testing::Test
{
protected:
  void SetUp() override {}

  void TearDown() override {}
};

// Input  : A(0,0), B(4,4)
// Output : (0,0), (1,1), (2,2), (3,3), (4,4)
TEST_F(TestSuite_GridLineTraversal, Testcase_1)
{
  const Eigen::Vector2i        start{0, 0};
  const Eigen::Vector2i        end{4, 4};
  std::vector<Eigen::Vector2i> points = GridLineTraversal::gridLine(start, end);

  for(const auto& point : points)
  {
    std::cout << point.x() << ", " << point.y() << std::endl;
  }
}

// Input  : A(0,0), B(4,2)
// Output : (0,0), (1,0), (2,1), (3,1), (4,2)
TEST_F(TestSuite_GridLineTraversal, Testcase_2)
{
  const Eigen::Vector2i        start{0, 0};
  const Eigen::Vector2i        end{4, 2};
  std::vector<Eigen::Vector2i> points = GridLineTraversal::gridLine(start, end);

  for(const auto& point : points)
  {
    std::cout << point.x() << ", " << point.y() << std::endl;
  }
}