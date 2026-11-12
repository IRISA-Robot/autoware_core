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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__EXPERIMENTAL__PATH_GUARD_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__EXPERIMENTAL__PATH_GUARD_HPP_

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <cmath>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

/// Total xy arc length of the path points [m].
inline double path_arc_length(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points)
{
  double length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & a = points[i - 1].point.pose.position;
    const auto & b = points[i].point.pose.position;
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length;
}

/// IRISA 2026-11-12: true when the path has no usable geometry (fewer than 2 points or a total
/// length below min_length_m, e.g. every point at the same position). pretty_build() accepts such
/// a path, and Trajectory::restore() on it segfaults (WalkwayModuleManager::launchNewModules,
/// 2026-11-11 14:29, ego long out of route + localization jump). onTrigger skips the cycle.
inline bool is_degenerate_path(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const double min_length_m = 0.01)
{
  return points.size() < 2 || path_arc_length(points) < min_length_m;
}

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__EXPERIMENTAL__PATH_GUARD_HPP_
