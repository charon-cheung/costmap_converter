#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <costmap_converter/costmap_to_lines_convex_hull.h>

namespace
{
class LineConverter : public costmap_converter::CostmapToLinesDBSMCCH
{
public:
  LineConverter()
  {
    support_pts_max_dist_ = 0.08;
    support_pts_max_dist_inbetween_ = 0.25;
    min_support_pts_ = 3;
  }

  using CostmapToLinesDBSMCCH::extractPointsAndLines;
};

class CostmapLinesTest : public ::testing::Test
{
protected:
  nav2_costmap_2d::Costmap2D map_{40, 40, 0.05, 0.0, 0.0};
  LineConverter converter_;

  void SetUp() override
  {
    converter_.setCostmap2D(&map_);
  }

  void fill(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
            unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    for (unsigned int x = x0; x <= x1; ++x)
      for (unsigned int y = y0; y <= y1; ++y)
        map_.setCost(x, y, cost);
  }

  std::vector<geometry_msgs::msg::Polygon> extractSegment(int x0, int y0, int x1, int y1,
                                                        int support_offset = 0)
  {
    // Axis-aligned or 45-degree segments with one support point per cell.
    std::vector<LineConverter::KeyPoint> cluster;
    const int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
    for (int i = 0; i <= steps; ++i)
    {
      double x, y;
      map_.mapToWorld(x0 + (x1 - x0) * i / steps,
                      y0 + (y1 - y0) * i / steps, x, y);
      cluster.emplace_back(x, y - support_offset * map_.getResolution());
      if (support_offset != 0)
        cluster.emplace_back(x, y + support_offset * map_.getResolution());
    }
    geometry_msgs::msg::Polygon candidate;
    candidate.points.resize(2);
    double x, y;
    map_.mapToWorld(x0, y0, x, y);
    LineConverter::KeyPoint(x, y).toPointMsg(candidate.points.front());
    map_.mapToWorld(x1, y1, x, y);
    LineConverter::KeyPoint(x, y).toPointMsg(candidate.points.back());
    std::vector<geometry_msgs::msg::Polygon> result;
    converter_.extractPointsAndLines(cluster, candidate, std::back_inserter(result));
    return result;
  }

  void expectLine(int x0, int y0, int x1, int y1)
  {
    const auto result = extractSegment(x0, y0, x1, y1);
    ASSERT_EQ(1u, result.size());
    ASSERT_EQ(2u, result.front().points.size());
    double x, y;
    map_.mapToWorld(x0, y0, x, y);
    EXPECT_NEAR(x, result.front().points.front().x, 1e-6);
    EXPECT_NEAR(y, result.front().points.front().y, 1e-6);
    map_.mapToWorld(x1, y1, x, y);
    EXPECT_NEAR(x, result.front().points.back().x, 1e-6);
    EXPECT_NEAR(y, result.front().points.back().y, 1e-6);
  }
};

TEST_F(CostmapLinesTest, SolidRectangleInteriorDiagonalIsDroppedWithoutPoints)
{
  fill(4, 4, 30, 30);
  const auto result = extractSegment(7, 7, 27, 27);
  EXPECT_TRUE(result.empty());
}

TEST_F(CostmapLinesTest, RejectsMostlySolidInteriorWithSomeFreeSideSamples)
{
  fill(4, 4, 30, 30);
  fill(10, 19, 15, 19, nav2_costmap_2d::FREE_SPACE);
  EXPECT_TRUE(extractSegment(7, 18, 27, 18).empty());
}

TEST_F(CostmapLinesTest, RejectedDiagonalPreservesLaterBoundariesAndUnrelatedPoints)
{
  fill(4, 4, 30, 30);
  std::vector<LineConverter::KeyPoint> cluster;
  for (unsigned int x = 4; x <= 30; ++x)
    for (unsigned int y = 4; y <= 30; ++y)
    {
      double wx, wy;
      map_.mapToWorld(x, y, wx, wy);
      cluster.emplace_back(wx, wy);
    }
  geometry_msgs::msg::Polygon candidate;
  for (const auto &cell : std::vector<std::pair<unsigned int, unsigned int>>{
      {4, 4}, {30, 30}, {4, 30}, {4, 4}})
  {
    double wx, wy;
    map_.mapToWorld(cell.first, cell.second, wx, wy);
    geometry_msgs::msg::Point32 point;
    point.x = wx;
    point.y = wy;
    candidate.points.push_back(point);
  }
  std::vector<geometry_msgs::msg::Polygon> result;
  converter_.extractPointsAndLines(cluster, candidate, std::back_inserter(result));
  unsigned int boundary_lines = 0;
  bool unrelated_point_preserved = false;
  for (const auto &polygon : result)
  {
    if (polygon.points.size() == 2)
    {
      ++boundary_lines;
      const auto &a = polygon.points.front();
      const auto &b = polygon.points.back();
      EXPECT_TRUE((std::abs(a.y - 1.525) < 1e-6 && std::abs(b.y - 1.525) < 1e-6) ||
                  (std::abs(a.x - 0.225) < 1e-6 && std::abs(b.x - 0.225) < 1e-6));
    }
    else if (polygon.points.size() == 1)
    {
      const auto &point = polygon.points.front();
      // Real rectangle corners may remain as points; only interior diagonal points are forbidden.
      EXPECT_FALSE(std::abs(point.x - point.y) < 1e-6 &&
                   point.x > 0.225 + 1e-6 && point.x < 1.525 - 1e-6)
        << "Unexpected interior point: " << point.x << ", " << point.y;
      if (std::abs(point.x - 1.525) < 1e-6 && std::abs(point.y - 0.225) < 1e-6)
        unrelated_point_preserved = true;
    }
  }
  EXPECT_EQ(2u, boundary_lines);
  EXPECT_TRUE(unrelated_point_preserved);
}

TEST_F(CostmapLinesTest, PreservesAllFourSolidRectangleBoundaries)
{
  fill(4, 4, 30, 30);
  expectLine(7, 4, 27, 4);
  expectLine(7, 30, 27, 30);
  expectLine(4, 7, 4, 27);
  expectLine(30, 7, 30, 27);
}

TEST_F(CostmapLinesTest, PreservesBothSidesOfOneCellWideFreeCorridor)
{
  fill(4, 4, 18, 30);
  fill(20, 4, 34, 30);
  expectLine(18, 7, 18, 27);
  expectLine(20, 7, 20, 27);
}

TEST_F(CostmapLinesTest, UnknownAndInflatedCellsDoNotProveSolidInterior)
{
  for (const unsigned char cost : {nav2_costmap_2d::NO_INFORMATION,
                                  nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE})
  {
    fill(0, 0, 39, 39, cost);
    fill(7, 18, 27, 18);
    expectLine(7, 18, 27, 18);
  }
}

TEST_F(CostmapLinesTest, PreservesObstacleAtMapBoundary)
{
  fill(0, 4, 10, 30);
  expectLine(0, 7, 0, 27);
}

TEST_F(CostmapLinesTest, FreeCenterBetweenOccupiedSidesIsNotSolidInterior)
{
  fill(7, 17, 27, 17);
  fill(7, 19, 27, 19);
  const auto result = extractSegment(7, 18, 27, 18, 1);
  ASSERT_EQ(1u, result.size());
  ASSERT_EQ(2u, result.front().points.size());
}
}  // namespace
