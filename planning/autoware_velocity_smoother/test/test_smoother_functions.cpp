// Copyright 2022 Tier IV, Inc.
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

#include "autoware/velocity_smoother/trajectory_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware::velocity_smoother::trajectory_utils::TrajectoryPoints;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoints genStraightTrajectory(const size_t size)
{
  double x = 0.0;
  double dx = 1.0;
  geometry_msgs::msg::Quaternion o;
  o.x = 0.0;
  o.y = 0.0;
  o.z = 0.0;
  o.w = 1.0;

  TrajectoryPoints tps;
  TrajectoryPoint p;
  for (size_t i = 0; i < size; ++i) {
    p.pose.position.x = x;
    p.pose.orientation = o;
    x += dx;
    tps.push_back(p);
  }
  return tps;
}

TEST(TestTrajectoryUtils, CalcTrajectoryCurvatureFrom3Points)
{
  // the output curvature vector should have the same size of the input trajectory.
  const auto checkOutputSize = [](const size_t trajectory_size, const size_t idx_dist) {
    const auto trajectory_points = genStraightTrajectory(trajectory_size);
    const auto curvatures =
      autoware::velocity_smoother::trajectory_utils::calcTrajectoryCurvatureFrom3Points(
        trajectory_points, idx_dist);
    EXPECT_EQ(curvatures.size(), trajectory_size) << ", idx_dist = " << idx_dist;
  };

  const auto trajectory_size_arr = {0, 1, 2, 3, 4, 5, 10, 100};
  const auto idx_dist_arr = {0, 1, 2, 3, 4, 5, 10, 100};
  for (const auto trajectory_size : trajectory_size_arr) {
    for (const auto idx_dist : idx_dist_arr) {
      checkOutputSize(trajectory_size, idx_dist);
    }
  }
}

namespace
{
// out along y=0, a 1.25 m-radius half circle, back along y=2.5: the two legs are 2.5 m apart
TrajectoryPoints genHairpin(const double ds)
{
  TrajectoryPoints tps;
  TrajectoryPoint p;
  const double r = 1.25;
  const double straight = 10.0;
  const double length = 2.0 * straight + M_PI * r;
  for (double s = 0.0; s <= length + 1e-9; s += ds) {
    if (s < straight) {
      p.pose.position.x = s;
      p.pose.position.y = 0.0;
    } else if (s < straight + M_PI * r) {
      const double th = (s - straight) / r;
      p.pose.position.x = straight + r * std::sin(th);
      p.pose.position.y = r - r * std::cos(th);
    } else {
      p.pose.position.x = straight - (s - straight - M_PI * r);
      p.pose.position.y = 2.0 * r;
    }
    p.longitudinal_velocity_mps = 1.0;
    tps.push_back(p);
  }
  return tps;
}

// every n-th point, velocity = arc length (so the transferred velocity reads back the arc length)
TrajectoryPoints sparseSamples(const TrajectoryPoints & dense, const size_t n, const double ds)
{
  TrajectoryPoints out;
  for (size_t i = 0; i < dense.size(); i += n) {
    auto p = dense.at(i);
    p.longitudinal_velocity_mps = static_cast<float>(i * ds);
    out.push_back(p);
  }
  return out;
}
}  // namespace

TEST(TestTrajectoryUtils, TransferVelocityToGeometryKeepsHairpin)
{
  using autoware::velocity_smoother::trajectory_utils::transferVelocityToGeometry;
  const double ds = 0.1;
  const auto dense = genHairpin(ds);
  const auto sparse = sparseSamples(dense, 40, ds);  // 4 m apart, like the optimizer's resample

  const auto out = transferVelocityToGeometry(dense, sparse, 0.5);
  ASSERT_TRUE(out.has_value());

  // the dense geometry is kept up to the last sparse point
  const size_t last = (dense.size() - 1) / 40 * 40;
  ASSERT_EQ(out->size(), last + 1);
  for (size_t i = 0; i < out->size(); ++i) {
    EXPECT_DOUBLE_EQ(out->at(i).pose.position.x, dense.at(i).pose.position.x);
    EXPECT_DOUBLE_EQ(out->at(i).pose.position.y, dense.at(i).pose.position.y);
    // velocity is the arc length of each point along the path, i.e. matched to the right leg
    EXPECT_NEAR(out->at(i).longitudinal_velocity_mps, i * ds, 1e-3) << "i=" << i;
  }
}

TEST(TestTrajectoryUtils, TransferVelocityToGeometryKeepsInputStop)
{
  using autoware::velocity_smoother::trajectory_utils::transferVelocityToGeometry;
  const double ds = 0.1;
  auto dense = genHairpin(ds);
  dense.at(55).longitudinal_velocity_mps = 0.0;
  const auto sparse = sparseSamples(dense, 40, ds);

  const auto out = transferVelocityToGeometry(dense, sparse, 0.5);
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->at(55).longitudinal_velocity_mps, 0.0);
}

TEST(TestTrajectoryUtils, TransferVelocityToGeometryRejectsOffPath)
{
  using autoware::velocity_smoother::trajectory_utils::transferVelocityToGeometry;
  const double ds = 0.1;
  const auto dense = genHairpin(ds);
  auto sparse = sparseSamples(dense, 40, ds);
  sparse.at(2).pose.position.y += 1.0;

  EXPECT_FALSE(transferVelocityToGeometry(dense, sparse, 0.5).has_value());
}
