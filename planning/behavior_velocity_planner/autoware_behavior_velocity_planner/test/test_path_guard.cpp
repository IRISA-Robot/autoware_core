// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/behavior_velocity_planner/experimental/path_guard.hpp"

#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware::behavior_velocity_planner::experimental::is_degenerate_path;
using Point = autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using Points = std::vector<Point>;

namespace
{
Point make(const double x, const double y)
{
  Point p;
  p.point.pose.position.x = x;
  p.point.pose.position.y = y;
  p.point.pose.orientation.w = 1.0;
  p.lane_ids = {81};
  return p;
}

// What onTrigger does after the guard: build + restore must not crash.
void expect_builds_and_restores(const Points & points)
{
  const auto trajectory = autoware::experimental::trajectory::pretty_build(points);
  ASSERT_TRUE(trajectory.has_value());
  EXPECT_GE(trajectory->restore().size(), 2u);
}
}  // namespace

// The 2026-11-11 crash: every point at one position. pretty_build() accepted these and restore()
// segfaulted; the guard must reject them before they reach the scene modules.
TEST(PathGuard, RejectsPathsWithoutGeometry)
{
  EXPECT_TRUE(is_degenerate_path({}));
  EXPECT_TRUE(is_degenerate_path({make(5, 5)}));
  EXPECT_TRUE(is_degenerate_path({make(5, 5), make(5, 5)}));
  EXPECT_TRUE(is_degenerate_path({make(5, 5), make(5 + 1e-9, 5)}));
  EXPECT_TRUE(is_degenerate_path({make(5, 5), make(5, 5), make(5, 5)}));
  EXPECT_TRUE(is_degenerate_path({make(2, 2), make(2, 2), make(2, 2), make(2, 2), make(2, 2)}));
  EXPECT_TRUE(is_degenerate_path({make(2, 2), make(2.003, 2), make(2.006, 2)}));  // 6 mm
}

// Normal paths (and short or partly duplicated ones) still go through, and are restorable.
TEST(PathGuard, KeepsNormalPaths)
{
  const std::vector<Points> cases = {
    {make(0, 0), make(1, 0)},
    {make(0, 0), make(0.02, 0)},                           // 2 cm: short but real
    {make(0, 0), make(1, 0), make(1, 0), make(1, 0)},      // duplicated tail
    {make(0, 0), make(1, 0), make(0.5, 0), make(0, 0)},    // out-and-back fold
    {make(0, 0), make(10, 0), make(20, 1), make(30, 3), make(40, 6)},
  };
  for (const auto & points : cases) {
    EXPECT_FALSE(is_degenerate_path(points));
    expect_builds_and_restores(points);
  }
}
