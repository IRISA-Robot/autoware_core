// Copyright 2021-2024 TIER IV, Inc.
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

#include "autoware/route_handler/route_handler.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/route_checker.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/path.hpp>

#include <boost/geometry/algorithms/detail/comparable_distance/interface.hpp>
#include <boost/geometry/algorithms/detail/envelope/interface.hpp>
#include <boost/geometry/index/distance_predicates.hpp>
#include <boost/geometry/index/predicates.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::route_handler
{
namespace
{
using autoware::experimental::lanelet2_utils::is_bicycle_lane;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::Path;
using autoware_utils_geometry::create_point;
using autoware_utils_geometry::create_quaternion_from_yaw;
using geometry_msgs::msg::Pose;
using lanelet::utils::to2D;

bool exists(const std::vector<LaneletPrimitive> & primitives, const int64_t & id)
{
  for (const auto & p : primitives) {
    if (p.id == id) {
      return true;
    }
  }
  return false;
}

bool exists(const lanelet::ConstLanelets & vectors, const lanelet::ConstLanelet & item)
{
  for (const auto & i : vectors) {
    if (i.id() == item.id()) {
      return true;
    }
  }
  return false;
}

std::optional<geometry_msgs::msg::Point> getGeometryPointFrom2DArcLength(
  const lanelet::ConstLanelets & lanelet_sequence, const double s)
{
  double accumulated_distance2d = 0;
  for (const auto & llt : lanelet_sequence) {
    const auto & centerline = llt.centerline();
    lanelet::ConstPoint3d prev_pt;
    if (!centerline.empty()) {
      prev_pt = centerline.front();
    }
    for (const auto & pt : centerline) {
      const double distance2d = lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
      if (accumulated_distance2d + distance2d > s) {
        const double ratio = (s - accumulated_distance2d) / distance2d;
        const auto interpolated_pt = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;

        geometry_msgs::msg::Point p;
        p.x = interpolated_pt.x();
        p.y = interpolated_pt.y();
        p.z = interpolated_pt.z();
        return p;
      }
      accumulated_distance2d += distance2d;
      prev_pt = pt;
    }
  }

  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  if (lanelet_sequence.back().centerline().empty()) {
    return std::nullopt;
  }

  const auto p_lanelet = lanelet_sequence.back().centerline().back().basicPoint();
  return autoware_utils_geometry::create_point(p_lanelet.x(), p_lanelet.y(), p_lanelet.z());
}

PathWithLaneId removeOverlappingPoints(const PathWithLaneId & input_path)
{
  PathWithLaneId filtered_path;
  filtered_path.points.reserve(input_path.points.size());

  for (const auto & pt : input_path.points) {
    if (filtered_path.points.empty()) {
      filtered_path.points.push_back(pt);
      continue;
    }

    constexpr double th_overlapping_dist = 0.001;
    const double dist_between_points =
      autoware_utils_geometry::calc_distance3d(filtered_path.points.back().point, pt.point);

    if (dist_between_points < th_overlapping_dist) {
      filtered_path.points.back().lane_ids.push_back(pt.lane_ids.front());
      filtered_path.points.back().point.longitudinal_velocity_mps =
        pt.point.longitudinal_velocity_mps;
      continue;
    }

    filtered_path.points.push_back(pt);
  }

  filtered_path.left_bound = input_path.left_bound;
  filtered_path.right_bound = input_path.right_bound;
  return filtered_path;
}

std::string toString(const geometry_msgs::msg::Pose & pose)
{
  std::stringstream ss;
  ss << "(" << pose.position.x << ", " << pose.position.y << "," << pose.position.z << ")";
  return ss.str();
}

bool isClose(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, const double epsilon)
{
  return std::abs(p1.x - p2.x) < epsilon && std::abs(p1.y - p2.y) < epsilon;
}

PiecewiseReferencePoints convertWaypointsToReferencePoints(
  const std::vector<geometry_msgs::msg::Point> & piecewise_waypoints)
{
  PiecewiseReferencePoints piecewise_ref_points;
  for (const auto & piecewise_waypoint : piecewise_waypoints) {
    piecewise_ref_points.push_back(ReferencePoint{true, piecewise_waypoint});
  }
  return piecewise_ref_points;
}

template <typename T>
bool isIndexWithinVector(const std::vector<T> & vec, const int index)
{
  return 0 <= index && index < static_cast<int>(vec.size());
}

template <typename T>
void removeIndicesFromVector(std::vector<T> & vec, std::vector<size_t> indices)
{
  // sort indices in a descending order
  std::sort(indices.begin(), indices.end(), std::greater<int>());

  // remove indices from vector
  for (const size_t index : indices) {
    vec.erase(vec.begin() + index);
  }
}

lanelet::ArcCoordinates calcArcCoordinates(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & point)
{
  return lanelet::geometry::toArcCoordinates(
    to2D(lanelet.centerline()),
    to2D(lanelet::utils::conversion::toLaneletPoint(point)).basicPoint());
}

std::string convertLaneletsIdToString(const lanelet::ConstLanelets & lanelets)
{
  std::stringstream ss;
  ss << "{";
  for (const auto & lanelet : lanelets) {
    ss << lanelet.id() << ",";
  }
  ss << "}";

  return ss.str();
}
}  // namespace

RouteHandler::RouteHandler(const LaneletMapBin & map_msg)
{
  setMap(map_msg);
  route_ptr_ = nullptr;
}

void RouteHandler::setMap(const LaneletMapBin & map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  const auto map_major_version_opt =
    lanelet::io_handlers::parseMajorVersion(map_msg.version_map_format);
  if (!map_major_version_opt) {
    RCLCPP_WARN(
      logger_, "setMap() for invalid version map: %s", map_msg.version_map_format.c_str());
  } else if (map_major_version_opt.value() > static_cast<uint64_t>(lanelet::autoware::version)) {
    RCLCPP_WARN(
      logger_, "setMap() for a map(version %s) newer than lanelet2_extension support version(%d)",
      map_msg.version_map_format.c_str(), static_cast<int>(lanelet::autoware::version));
  }

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  const lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  const lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  const lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);

  is_map_msg_ready_ = true;
  is_handler_ready_ = false;

  setLaneletsFromRouteMsg();
}

bool RouteHandler::isRouteLooped(const RouteSections & route_sections)
{
  std::set<lanelet::Id> lane_primitives;
  for (const auto & route_section : route_sections) {
    for (const auto & primitive : route_section.primitives) {
      if (lane_primitives.find(primitive.id) != lane_primitives.end()) {
        return true;  // find duplicated id
      }
      lane_primitives.emplace(primitive.id);
    }
  }
  return false;
}

void RouteHandler::setRoute(const LaneletRoute & route_msg)
{
  if (!isRouteLooped(route_msg.segments)) {
    // if get not modified route but new route, reset original start pose
    if (!route_ptr_ || route_ptr_->uuid != route_msg.uuid) {
      original_start_pose_ = route_msg.start_pose;
      original_goal_pose_ = route_msg.goal_pose;
    }
    route_ptr_ = std::make_shared<LaneletRoute>(route_msg);
    is_handler_ready_ = false;
    setLaneletsFromRouteMsg();
  } else {
    RCLCPP_ERROR(
      logger_,
      "Loop detected within route! Currently, no loop is allowed for route! Using previous route");
  }
}

bool RouteHandler::isHandlerReady() const
{
  return is_handler_ready_;
}

void RouteHandler::setRouteLanelets(const lanelet::ConstLanelets & path_lanelets)
{
  // Bidirectional-driving support: path_lanelets is the one place in this function where the
  // traversed ConstLanelet objects still carry their correct .inverted() bit (they come straight
  // out of planPathLaneletsBetweenCheckpoints()'s routing-graph search). Every other lanelet
  // touched below is re-fetched by id from lanelet_map_ptr_ (always the forward view), so record
  // the per-id direction here before that happens.
  reversed_in_route_.clear();
  for (const auto & lane : path_lanelets) {
    reversed_in_route_[lane.id()] = lane.inverted();
  }

  if (!path_lanelets.empty()) {
    const auto & first_lanelet = path_lanelets.front();
    start_lanelets_ = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, first_lanelet);
    const auto & last_lanelet = path_lanelets.back();
    goal_lanelets_ = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, last_lanelet);
  }

  // set route lanelets
  std::unordered_set<lanelet::Id> route_lanelets_id;
  std::unordered_set<lanelet::Id> candidate_lanes_id;
  for (const auto & lane : path_lanelets) {
    route_lanelets_id.insert(lane.id());
    const auto right_relations = routing_graph_ptr_->rightRelations(lane);
    for (const auto & right_relation : right_relations) {
      if (right_relation.relationType == lanelet::routing::RelationType::Right) {
        route_lanelets_id.insert(right_relation.lanelet.id());
      } else if (right_relation.relationType == lanelet::routing::RelationType::AdjacentRight) {
        candidate_lanes_id.insert(right_relation.lanelet.id());
      }
    }
    const auto left_relations = routing_graph_ptr_->leftRelations(lane);
    for (const auto & left_relation : left_relations) {
      if (left_relation.relationType == lanelet::routing::RelationType::Left) {
        route_lanelets_id.insert(left_relation.lanelet.id());
      } else if (left_relation.relationType == lanelet::routing::RelationType::AdjacentLeft) {
        candidate_lanes_id.insert(left_relation.lanelet.id());
      }
    }
  }

  //  check if candidates are really part of route
  for (const auto & candidate_id : candidate_lanes_id) {
    lanelet::ConstLanelet lanelet = lanelet_map_ptr_->laneletLayer.get(candidate_id);
    auto previous_lanelets = routing_graph_ptr_->previous(lanelet);
    bool is_connected_to_main_lanes_prev = false;
    bool is_connected_to_candidate_prev = true;
    if (exists(start_lanelets_, lanelet)) {
      is_connected_to_candidate_prev = false;
    }
    while (!previous_lanelets.empty() && is_connected_to_candidate_prev &&
           !is_connected_to_main_lanes_prev) {
      is_connected_to_candidate_prev = false;

      for (const auto & prev_lanelet : previous_lanelets) {
        if (route_lanelets_id.find(prev_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_prev = true;
          break;
        }
        if (exists(start_lanelets_, prev_lanelet)) {
          break;
        }

        if (candidate_lanes_id.find(prev_lanelet.id()) != candidate_lanes_id.end()) {
          is_connected_to_candidate_prev = true;
          previous_lanelets = routing_graph_ptr_->previous(prev_lanelet);
          break;
        }
      }
    }

    auto following_lanelets = routing_graph_ptr_->following(lanelet);
    bool is_connected_to_main_lanes_next = false;
    bool is_connected_to_candidate_next = true;
    if (exists(goal_lanelets_, lanelet)) {
      is_connected_to_candidate_next = false;
    }
    while (!following_lanelets.empty() && is_connected_to_candidate_next &&
           !is_connected_to_main_lanes_next) {
      is_connected_to_candidate_next = false;
      for (const auto & next_lanelet : following_lanelets) {
        if (route_lanelets_id.find(next_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_next = true;
          break;
        }
        if (exists(goal_lanelets_, next_lanelet)) {
          break;
        }
        if (candidate_lanes_id.find(next_lanelet.id()) != candidate_lanes_id.end()) {
          is_connected_to_candidate_next = true;
          following_lanelets = routing_graph_ptr_->following(next_lanelet);
          break;
        }
      }
    }

    if (is_connected_to_main_lanes_next && is_connected_to_main_lanes_prev) {
      route_lanelets_id.insert(candidate_id);
    }
  }

  route_lanelets_.clear();
  route_lanelets_.reserve(route_lanelets_id.size());
  std::vector<RouteRtreeNode> rtree_nodes;
  rtree_nodes.reserve(route_lanelets_id.size());
  size_t i = 0;
  for (const auto & id : route_lanelets_id) {
    // Re-apply the tracked direction (if any) when re-fetching by id -- laneletLayer.get()
    // always yields the forward view, which would otherwise silently discard reversed_in_route_
    // recorded above for the primary path_lanelets ids. Ids added purely as lane-change
    // candidates (right/left relations, not in path_lanelets) default to forward, which is
    // correct since lane_change is not exercised on reversed segments.
    const lanelet::ConstLanelet forward_llt = lanelet_map_ptr_->laneletLayer.get(id);
    const auto reversed_it = reversed_in_route_.find(id);
    const lanelet::ConstLanelet llt =
      (reversed_it != reversed_in_route_.end() && reversed_it->second) ? forward_llt.invert()
                                                                        : forward_llt;
    route_lanelets_.push_back(llt);
    rtree_nodes.emplace_back(
      boost::geometry::return_envelope<autoware_utils_geometry::Box2d>(
        route_lanelets_.back().polygon2d().basicPolygon()),
      i++);
  }
  route_lanelets_rtree_ = RouteRtree(rtree_nodes);
  is_handler_ready_ = true;
}

void RouteHandler::clearRoute()
{
  route_lanelets_.clear();
  route_lanelets_rtree_.clear();
  preferred_lanelets_.clear();
  start_lanelets_.clear();
  goal_lanelets_.clear();
  reversed_in_route_.clear();
  route_ptr_ = nullptr;
  is_handler_ready_ = false;
}

void RouteHandler::setAllowReverseRoute(bool allow)
{
  allow_reverse_route_ = allow;
}

bool RouteHandler::isLaneletInvertedInRoute(const lanelet::ConstLanelet & lanelet) const
{
  const auto it = reversed_in_route_.find(lanelet.id());
  if (it != reversed_in_route_.end()) {
    return it->second;
  }
  // Not tracked (e.g. queried before any route is set, or for a lanelet outside the route) --
  // fall back to trusting the caller-provided ConstLanelet's own orientation.
  return lanelet.inverted();
}

bool RouteHandler::isBidirectionalDrivingLanelet(const lanelet::ConstLanelet & llt)
{
  // Idiom follows the existing custom-tag convention in this file, see isNoDrivableLane()
  // (attributeOr("no_drivable_lane", "no")).
  //
  // IMPORTANT: attributeOr<T>() deduces T from the literal type of the default value. A bare
  // `"no"` is a `const char*`, so `attributeOr("bidirectional_driving", "no") == "yes"` compared
  // two `const char*` POINTERS (address comparison), not their string content -- always false in
  // practice, regardless of the actual tag value. isNoDrivableLane() avoided this bug only by
  // accident, via an intermediate `const std::string` variable that implicitly converts the
  // `const char*` result before the `==` comparison. Do that explicitly here too (found via live
  // testing: the attribute was confirmed present with value "yes" at runtime, yet this function
  // still returned false before this fix).
  const std::string bidirectional_driving_attribute =
    llt.attributeOr("bidirectional_driving", "no");
  return bidirectional_driving_attribute == "yes";
}

bool RouteHandler::hasDeferredRegulatoryElementForReverse(const lanelet::ConstLanelet & llt)
{
  // Regulatory element types explicitly deferred by the bidirectional-driving migration -- see
  // bidirectional_plan/06-deferred-scope.md. Checked generically via each regulatory element's
  // "subtype" attribute so this does not require linking against every concrete regulatory
  // element class (many of blind_spot/no_stopping_area/virtual_traffic_light/occlusion_spot are
  // autoware_lanelet2_extension custom regulatory elements, not lanelet2-core ones).
  static const std::unordered_set<std::string> deferred_types = {
    "traffic_light",       "roundabout",         "crosswalk",     "blind_spot",
    "no_stopping_area",    "virtual_traffic_light", "occlusion_spot", "speed_bump"};

  for (const auto & re : llt.regulatoryElements()) {
    if (!re) continue;
    if (!re->hasAttribute(lanelet::AttributeName::Subtype)) continue;
    const std::string subtype = re->attribute(lanelet::AttributeName::Subtype).value();
    if (deferred_types.count(subtype) != 0) {
      return true;
    }
  }

  // "intersection" is not a regulatory element in this codebase's maps -- it is expressed as a
  // RightOfWay regulatory element (already caught above if tagged with subtype "right_of_way" is
  // NOT in the deferred list on purpose, since plain right_of_way without an intersection
  // attribute is not itself deferred) plus lanelet-level attributes. Reject on the lanelet-level
  // "turn_direction" attribute (set on lanelets inside a signalized/unsignalized intersection in
  // this map authoring convention) as a conservative proxy for "this is an intersection lanelet".
  if (llt.hasAttribute("turn_direction")) {
    return true;
  }

  return false;
}

void RouteHandler::setLaneletsFromRouteMsg()
{
  if (!route_ptr_ || !is_map_msg_ready_) {
    return;
  }
  route_lanelets_.clear();
  route_lanelets_rtree_.clear();
  preferred_lanelets_.clear();
  reversed_in_route_.clear();
  const bool is_route_valid = lanelet::utils::route::isRouteValid(*route_ptr_, lanelet_map_ptr_);
  if (!is_route_valid) {
    return;
  }

  size_t primitive_size{0};
  for (const auto & route_section : route_ptr_->segments) {
    primitive_size += route_section.primitives.size();
  }
  route_lanelets_.reserve(primitive_size);
  std::vector<RouteRtreeNode> rtree_nodes;
  rtree_nodes.reserve(primitive_size);
  size_t i = 0;

  // Bidirectional-driving support: this is the one place a LaneletRoute message is turned back
  // into live ConstLanelet objects, so it is where LaneletSegment::is_reversed must be read back
  // out and re-applied via .invert() -- otherwise a route loaded from a message (as opposed to
  // one just computed in-process by planPathLaneletsBetweenCheckpoints()) would silently lose
  // direction. Also repopulates the reversed_in_route_ side-table so it round-trips correctly.
  for (const auto & route_section : route_ptr_->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const lanelet::ConstLanelet forward_llt = lanelet_map_ptr_->laneletLayer.get(id);
      const lanelet::ConstLanelet llt =
        route_section.is_reversed ? forward_llt.invert() : forward_llt;
      reversed_in_route_[id] = route_section.is_reversed;
      route_lanelets_.push_back(llt);
      rtree_nodes.emplace_back(
        boost::geometry::return_envelope<autoware_utils_geometry::Box2d>(
          llt.polygon2d().basicPolygon()),
        i++);
      if (id == route_section.preferred_primitive.id) {
        preferred_lanelets_.push_back(llt);
      }
    }
  }
  route_lanelets_rtree_ = RouteRtree(rtree_nodes);
  goal_lanelets_.clear();
  start_lanelets_.clear();
  if (!route_ptr_->segments.empty()) {
    const auto & goal_section = route_ptr_->segments.back();
    goal_lanelets_.reserve(goal_section.primitives.size());
    for (const auto & primitive : goal_section.primitives) {
      const auto id = primitive.id;
      const lanelet::ConstLanelet forward_llt = lanelet_map_ptr_->laneletLayer.get(id);
      goal_lanelets_.push_back(goal_section.is_reversed ? forward_llt.invert() : forward_llt);
    }
    const auto & start_section = route_ptr_->segments.front();
    start_lanelets_.reserve(start_section.primitives.size());
    for (const auto & primitive : start_section.primitives) {
      const auto id = primitive.id;
      const lanelet::ConstLanelet forward_llt = lanelet_map_ptr_->laneletLayer.get(id);
      start_lanelets_.push_back(start_section.is_reversed ? forward_llt.invert() : forward_llt);
    }
  }
  is_handler_ready_ = true;
}

Header RouteHandler::getRouteHeader() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getRouteHeader: Route has not been set yet");
    return Header();
  }
  return route_ptr_->header;
}

UUID RouteHandler::getRouteUuid() const
{
  if (!route_ptr_) {
    RCLCPP_WARN_SKIPFIRST(logger_, "[Route Handler] getRouteUuid: Route has not been set yet");
    return UUID();
  }
  return route_ptr_->uuid;
}

bool RouteHandler::isAllowedGoalModification() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getRouteUuid: Route has not been set yet");
    return false;
  }
  return route_ptr_->allow_modification;
}

std::vector<lanelet::ConstLanelet> RouteHandler::getLanesAfterGoal(
  const double vehicle_length) const
{
  lanelet::ConstLanelet goal_lanelet;
  if (!getGoalLanelet(&goal_lanelet)) {
    return std::vector<lanelet::ConstLanelet>{};
  }

  const double min_succeeding_length = vehicle_length * 2;
  const auto succeeding_lanes_vec = lanelet::utils::query::getSucceedingLaneletSequences(
    routing_graph_ptr_, goal_lanelet, min_succeeding_length);
  if (succeeding_lanes_vec.empty()) {
    return std::vector<lanelet::ConstLanelet>{};
  }

  return succeeding_lanes_vec.front();
}

lanelet::ConstLanelets RouteHandler::getRouteLanelets() const
{
  return route_lanelets_;
}

lanelet::ConstLanelets RouteHandler::getPreferredLanelets() const
{
  return preferred_lanelets_;
}

Pose RouteHandler::getStartPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getStartPose: Route has not been set yet");
    return Pose();
  }
  return route_ptr_->start_pose;
}

Pose RouteHandler::getOriginalStartPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getOriginalStartPose: Route has not been set yet");
    return Pose();
  }
  return original_start_pose_;
}

Pose RouteHandler::getGoalPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getGoalPose: Route has not been set yet");
    return Pose();
  }
  return route_ptr_->goal_pose;
}

Pose RouteHandler::getOriginalGoalPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getOriginalGoalPose: Route has not been set yet");
    return Pose();
  }
  return original_goal_pose_;
}

lanelet::Id RouteHandler::getGoalLaneId() const
{
  if (!route_ptr_ || route_ptr_->segments.empty()) {
    return lanelet::InvalId;
  }

  return route_ptr_->segments.back().preferred_primitive.id;
}

bool RouteHandler::getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const
{
  const lanelet::Id goal_lane_id = getGoalLaneId();
  for (const auto & llt : route_lanelets_) {
    if (llt.id() == goal_lane_id) {
      *goal_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::isInGoalRouteSection(const lanelet::ConstLanelet & lanelet) const
{
  if (!route_ptr_ || route_ptr_->segments.empty()) {
    return false;
  }
  return exists(route_ptr_->segments.back().primitives, lanelet.id());
}

lanelet::ConstLanelets RouteHandler::getLaneletsFromIds(const lanelet::Ids & ids) const
{
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(ids.size());
  for (const auto & id : ids) {
    lanelets.push_back(getLaneletsFromId(id));
  }
  return lanelets;
}

lanelet::ConstLanelet RouteHandler::getLaneletsFromId(const lanelet::Id id) const
{
  // [BIDIR-BUG-FIX] laneletLayer.get(id) always yields the map-authored forward view, silently
  // discarding whatever direction this lanelet was actually traversed in the active route (the
  // same "re-fetch-by-id discards .inverted()" bug class already fixed 5x in this file, e.g.
  // constructRouteLaneletsFromCheckpoints() re-applying reversed_in_route_ when rebuilding
  // route_lanelets_ by id). Callers of this accessor (e.g. getLaneletsFromPath() /
  // get_lanelet_sequence_from_path() in behavior_path_planner_common's utils.cpp, feeding
  // generateDrivableLanes()) rely on the returned ConstLanelet's own leftBound3d()/
  // rightBound3d() to build the drivable-area boundary polygon in travel order; if this
  // lanelet was actually traversed inverted in the route, that boundary comes out
  // forward-oriented instead, producing a discontinuous / self-intersecting drivable area on
  // reversed-lanelet segments -- plausible root cause of "trajectory ngawur" on bidirectional
  // routes. Re-apply the tracked direction the same way setRouteLanelets() does when
  // re-fetching route_lanelets_ by id.
  const lanelet::ConstLanelet forward_llt = lanelet_map_ptr_->laneletLayer.get(id);
  const auto reversed_it = reversed_in_route_.find(id);
  if (reversed_it != reversed_in_route_.end() && reversed_it->second) {
    // Throttled: this accessor is called every planning cycle for every lanelet id in the
    // path (e.g. via get_lanelet_sequence_from_path()), so an unthrottled WARN here would
    // flood the log on any route containing a reversed segment.
    static rclcpp::Clock clock{RCL_ROS_TIME};
    RCLCPP_WARN_THROTTLE(
      logger_, clock, 1000,
      "[BIDIR-DEBUG] getLaneletsFromId: id=%ld re-applying inverted orientation "
      "(reversed_in_route_) that laneletLayer.get() would otherwise have discarded",
      id);
    return forward_llt.invert();
  }
  return forward_llt;
}

bool RouteHandler::isDeadEndLanelet(const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelet next_lanelet;
  return !getNextLaneletWithinRoute(lanelet, &next_lanelet);
}

lanelet::ConstLanelets RouteHandler::getLaneChangeableNeighbors(
  const lanelet::ConstLanelet & lanelet) const
{
  return lanelet::utils::query::getLaneChangeableNeighbors(routing_graph_ptr_, lanelet);
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_forward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet)) {
      break;
    }
    // loop check
    if (lanelet.id() == next_lanelet.id()) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(next_lanelet.centerline().basicLineString()));
  }

  return lanelet_sequence_forward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  double length = 0;
  lanelet::ConstLanelets previous_lanelets;

  auto checkForLoop =
    [&lanelet](const lanelet::ConstLanelets & lanelets_to_check, const bool is_route_lanelets) {
      if (is_route_lanelets) {
        return std::none_of(
          lanelets_to_check.begin(), lanelets_to_check.end(),
          [lanelet](auto & prev_llt) { return lanelet.id() != prev_llt.id(); });
      }
      return std::any_of(
        lanelets_to_check.begin(), lanelets_to_check.end(),
        [lanelet](auto & prev_llt) { return lanelet.id() == prev_llt.id(); });
    };

  auto isNewLanelet = [&lanelet,
                       &lanelet_sequence_backward](const lanelet::ConstLanelet & lanelet_to_check) {
    if (lanelet.id() == lanelet_to_check.id()) return false;
    return std::none_of(
      lanelet_sequence_backward.begin(), lanelet_sequence_backward.end(),
      [&lanelet_to_check](auto & backward) { return (backward.id() == lanelet_to_check.id()); });
  };

  while (rclcpp::ok() && length < min_length) {
    previous_lanelets.clear();
    if (!getPreviousLaneletsWithinRoute(current_lanelet, &previous_lanelets)) {
      break;
    }

    if (checkForLoop(previous_lanelets, true)) break;

    for (const auto & prev_lanelet : previous_lanelets) {
      if (!isNewLanelet(prev_lanelet) || exists(goal_lanelets_, prev_lanelet)) continue;
      lanelet_sequence_backward.push_back(prev_lanelet);
      length +=
        static_cast<double>(boost::geometry::length(prev_lanelet.centerline().basicLineString()));
      current_lanelet = prev_lanelet;
      break;
    }
  }

  std::reverse(lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const double backward_distance,
  const double forward_distance) const
{
  Pose current_pose{};
  current_pose.orientation.w = 1;
  if (!lanelet.centerline().empty()) {
    current_pose.position = lanelet::utils::conversion::toGeomMsgPt(lanelet.centerline().front());
  }

  lanelet::ConstLanelets lanelet_sequence;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  const lanelet::ConstLanelets lanelet_sequence_forward =
    getLaneletSequenceAfter(lanelet, forward_distance);
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      return getLaneletSequenceUpTo(lanelet, backward_distance);
    }
    return lanelet::ConstLanelets{};
  });

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  lanelet_sequence.push_back(lanelet);
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & current_pose, const double backward_distance,
  const double forward_distance) const
{
  if (!exists(route_lanelets_, lanelet)) {
    return {};
  }

  lanelet::ConstLanelets lanelet_sequence_forward =
    getLaneletSequenceAfter(lanelet, forward_distance);
  lanelet::ConstLanelets lanelet_sequence = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      return getLaneletSequenceUpTo(lanelet, backward_distance);
    }
    return lanelet::ConstLanelets{};
  });

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence.empty()) {
    if (lanelet_sequence.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.push_back(lanelet);
  std::move(
    lanelet_sequence_forward.begin(), lanelet_sequence_forward.end(),
    std::back_inserter(lanelet_sequence));
  return lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getRoadLaneletsAtPose(const Pose & pose) const
{
  lanelet::ConstLanelets road_lanelets_at_pose;
  const lanelet::BasicPoint2d p{pose.position.x, pose.position.y};
  const auto lanelets_at_pose = lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p));
  for (const auto & lanelet_at_pose : lanelets_at_pose) {
    // confirm that the pose is inside the lanelet since "search" does an approximation with boxes
    const auto is_pose_inside_lanelet = lanelet::geometry::inside(lanelet_at_pose, p);
    if (is_pose_inside_lanelet && isRoadLanelet(lanelet_at_pose))
      road_lanelets_at_pose.push_back(lanelet_at_pose);
  }
  return road_lanelets_at_pose;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getFollowingShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  bool found = false;
  const auto & search_point = lanelet.centerline().back().basicPoint2d();
  const auto next_lanelet = lanelet_map_ptr_->laneletLayer.nearestUntil(
    search_point, [&](const auto & bbox, const auto & ll) {
      if (isShoulderLanelet(ll) && lanelet::geometry::follows(lanelet, ll)) found = true;
      // stop search once next shoulder lanelet is found, or the bbox does not touch the search
      // point
      return found || lanelet::geometry::distance2d(bbox, search_point) > 1e-3;
    });
  if (found && next_lanelet.has_value()) return *next_lanelet;
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getLeftShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound())) {
    if (other_lanelet.rightBound() == lanelet.leftBound() && isShoulderLanelet(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getRightShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound())) {
    if (other_lanelet.leftBound() == lanelet.rightBound() && isShoulderLanelet(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getLeftBicycleLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound())) {
    if (other_lanelet.rightBound() == lanelet.leftBound() && is_bicycle_lane(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getRightBicycleLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound())) {
    if (other_lanelet.leftBound() == lanelet.rightBound() && is_bicycle_lane(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletsAtPose(const Pose & pose) const
{
  lanelet::ConstLanelets lanelets_at_pose;
  const lanelet::BasicPoint2d p{pose.position.x, pose.position.y};
  const auto candidates_at_pose = lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p));
  for (const auto & candidate : candidates_at_pose) {
    // confirm that the pose is inside the lanelet since "search" does an approximation with boxes
    const auto is_pose_inside_lanelet = lanelet::geometry::inside(candidate, p);
    if (is_pose_inside_lanelet && isShoulderLanelet(candidate))
      lanelets_at_pose.push_back(candidate);
  }
  return lanelets_at_pose;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!isShoulderLanelet(lanelet)) return lanelet_sequence_forward;

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::set<lanelet::Id> searched_ids{};
  while (rclcpp::ok() && length < min_length) {
    const auto next_lanelet = getFollowingShoulderLanelet(current_lanelet);
    if (!next_lanelet) break;
    lanelet_sequence_forward.push_back(*next_lanelet);
    if (searched_ids.find(next_lanelet->id()) != searched_ids.end()) {
      // loop shoulder detected
      break;
    }
    searched_ids.insert(next_lanelet->id());
    current_lanelet = *next_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(next_lanelet->centerline().basicLineString()));
  }

  return lanelet_sequence_forward;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getPreviousShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  bool found = false;
  const auto & search_point = lanelet.centerline().front().basicPoint2d();
  const auto previous_lanelet = lanelet_map_ptr_->laneletLayer.nearestUntil(
    search_point, [&](const auto & bbox, const auto & ll) {
      if (isShoulderLanelet(ll) && lanelet::geometry::follows(ll, lanelet)) found = true;
      // stop search once prev shoulder lanelet is found, or the bbox does not touch the search
      // point
      return found || lanelet::geometry::distance2d(bbox, search_point) > 1e-3;
    });
  if (found && previous_lanelet.has_value()) return *previous_lanelet;
  return std::nullopt;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!isShoulderLanelet(lanelet)) return lanelet_sequence_backward;

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::set<lanelet::Id> searched_ids{};
  while (rclcpp::ok() && length < min_length) {
    const auto prev_lanelet = getPreviousShoulderLanelet(current_lanelet);
    if (!prev_lanelet) break;

    lanelet_sequence_backward.insert(lanelet_sequence_backward.begin(), *prev_lanelet);
    if (searched_ids.find(prev_lanelet->id()) != searched_ids.end()) {
      // loop shoulder detected
      break;
    }
    searched_ids.insert(prev_lanelet->id());
    current_lanelet = *prev_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(prev_lanelet->centerline().basicLineString()));
  }

  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & pose, const double backward_distance,
  const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  if (!isShoulderLanelet(lanelet)) {
    return lanelet_sequence;
  }

  lanelet::ConstLanelets lanelet_sequence_forward =
    getShoulderLaneletSequenceAfter(lanelet, forward_distance);
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, pose);
    if (arc_coordinate.length < backward_distance) {
      return getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
    }
    return lanelet::ConstLanelets{};
  });

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());

  lanelet_sequence.push_back(lanelet);
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::get_shoulder_lanelet_sequence(
  const lanelet::ConstLanelet & lanelet, const double backward_distance,
  const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  if (!isShoulderLanelet(lanelet)) {
    return lanelet_sequence;
  }

  Pose current_pose{};
  current_pose.orientation.w = 1;
  if (!lanelet.centerline().empty()) {
    current_pose.position = lanelet::utils::conversion::toGeomMsgPt(lanelet.centerline().front());
  }

  lanelet::ConstLanelets lanelet_sequence_forward =
    getShoulderLaneletSequenceAfter(lanelet, forward_distance);
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      return getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
    }
    return lanelet::ConstLanelets{};
  });

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());

  lanelet_sequence.push_back(lanelet);
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

bool RouteHandler::getClosestLaneletWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
{
  if (route_lanelets_.empty() || closest_lanelet == nullptr) {
    return false;
  }
  const auto search_point = lanelet::BasicPoint2d(search_pose.position.x, search_pose.position.y);
  const auto query_nearest = boost::geometry::index::nearest(search_point, route_lanelets_.size());
  auto min_dist_to_route_lanelet = std::numeric_limits<double>::max();
  size_t nearest_id = 0;
  // search starting from the nearest bounding box
  for (auto query_it = route_lanelets_rtree_.qbegin(query_nearest);
       query_it != route_lanelets_rtree_.qend(); ++query_it) {
    const auto dist_to_bbox = boost::geometry::comparable_distance(search_point, query_it->first);
    // stop when the distance to the bounding box is larger than the min distance found so far
    if (dist_to_bbox > min_dist_to_route_lanelet) {
      break;
    }
    const auto dist = boost::geometry::comparable_distance(
      search_point, route_lanelets_[query_it->second].polygon2d().basicPolygon());
    if (dist < min_dist_to_route_lanelet) {
      min_dist_to_route_lanelet = dist;
      nearest_id = query_it->second;
    }
  }
  *closest_lanelet = route_lanelets_[nearest_id];
  return true;
}

bool RouteHandler::getClosestPreferredLaneletWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
{
  return lanelet::utils::query::getClosestLanelet(
    preferred_lanelets_, search_pose, closest_lanelet);
}

bool RouteHandler::getClosestLaneletWithConstrainsWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet, const double dist_threshold,
  const double yaw_threshold) const
{
  if (route_lanelets_.empty() || closest_lanelet == nullptr) {
    return false;
  }
  const auto pose_yaw = tf2::getYaw(search_pose.orientation);
  const auto search_point = lanelet::BasicPoint2d(search_pose.position.x, search_pose.position.y);
  const auto query_nearest = boost::geometry::index::nearest(search_point, route_lanelets_.size());
  auto min_dist_to_route_lanelet = std::numeric_limits<double>::max();
  auto min_angle_diff_to_route_lanelet = std::numeric_limits<double>::max();
  size_t nearest_id = 0;
  // search starting from the nearest bounding box
  for (auto query_it = route_lanelets_rtree_.qbegin(query_nearest);
       query_it != route_lanelets_rtree_.qend(); ++query_it) {
    const auto dist_to_bbox = boost::geometry::comparable_distance(search_point, query_it->first);
    // stop when the distance to the bounding box is larger than the min distance found so far
    if (dist_to_bbox > min_dist_to_route_lanelet || dist_to_bbox > dist_threshold) {
      break;
    }
    const auto & lanelet = route_lanelets_[query_it->second];
    const auto dist =
      boost::geometry::comparable_distance(search_point, lanelet.polygon2d().basicPolygon());
    const double lanelet_angle = autoware::experimental::lanelet2_utils::get_lanelet_angle(
      lanelet, autoware::experimental::lanelet2_utils::from_ros(search_pose.position).basicPoint());
    const double angle_diff =
      std::abs(autoware_utils_geometry::normalize_radian(lanelet_angle - pose_yaw));
    if (dist > dist_threshold || angle_diff > std::abs(yaw_threshold)) {
      continue;
    }
    if (
      dist < min_dist_to_route_lanelet ||
      (dist == min_dist_to_route_lanelet && angle_diff < min_angle_diff_to_route_lanelet)) {
      min_dist_to_route_lanelet = dist;
      min_angle_diff_to_route_lanelet = angle_diff;
      nearest_id = query_it->second;
    }
  }
  if (min_dist_to_route_lanelet > dist_threshold) {
    return false;
  }
  *closest_lanelet = route_lanelets_[nearest_id];
  return true;
}

bool RouteHandler::getClosestRouteLaneletFromLanelet(
  const Pose & search_pose, const lanelet::ConstLanelet & reference_lanelet,
  lanelet::ConstLanelet * closest_lanelet, const double dist_threshold,
  const double yaw_threshold) const
{
  lanelet::ConstLanelets previous_lanelets, next_lanelets, lanelet_sequence;
  if (getPreviousLaneletsWithinRoute(reference_lanelet, &previous_lanelets)) {
    lanelet_sequence = previous_lanelets;
  }

  lanelet_sequence.push_back(reference_lanelet);

  if (getNextLaneletsWithinRoute(reference_lanelet, &next_lanelets)) {
    lanelet_sequence.insert(lanelet_sequence.end(), next_lanelets.begin(), next_lanelets.end());
  }
  auto opt = autoware::experimental::lanelet2_utils::get_closest_lanelet_within_constraint(
    lanelet_sequence, search_pose, dist_threshold, yaw_threshold);
  if (opt.has_value()) {
    *closest_lanelet = *opt;
    return true;
  }

  return false;
}

bool RouteHandler::getNextLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * next_lanelets) const
{
  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }

  const auto start_lane_id = route_ptr_->segments.front().preferred_primitive.id;

  const auto following_lanelets = routing_graph_ptr_->following(lanelet);
  next_lanelets->clear();
  for (const auto & llt : following_lanelets) {
    if (start_lane_id != llt.id() && exists(route_lanelets_, llt)) {
      next_lanelets->push_back(llt);
    }
  }
  return !(next_lanelets->empty());
}

bool RouteHandler::getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const
{
  lanelet::ConstLanelets next_lanelets{};
  if (getNextLaneletsWithinRoute(lanelet, &next_lanelets)) {
    *next_lanelet = next_lanelets.front();
    return true;
  }
  return false;
}

lanelet::ConstLanelets RouteHandler::getNextLanelets(const lanelet::ConstLanelet & lanelet) const
{
  return routing_graph_ptr_->following(lanelet);
}

bool RouteHandler::getPreviousLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * prev_lanelets) const
{
  if (exists(start_lanelets_, lanelet)) {
    return false;
  }
  const auto candidate_lanelets = routing_graph_ptr_->previous(lanelet);
  prev_lanelets->clear();
  for (const auto & llt : candidate_lanelets) {
    if (exists(route_lanelets_, llt)) {
      prev_lanelets->push_back(llt);
    }
  }
  return !(prev_lanelets->empty());
}

lanelet::ConstLanelets RouteHandler::getPreviousLanelets(
  const lanelet::ConstLanelet & lanelet) const
{
  return routing_graph_ptr_->previous(lanelet);
}

std::optional<lanelet::ConstLanelet> RouteHandler::getRightLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // right road lanelet of shoulder lanelet
  if (isShoulderLanelet(lanelet)) {
    const auto right_lanelets = lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound());
    for (const auto & right_lanelet : right_lanelets)
      if (isRoadLanelet(right_lanelet) && lanelet.rightBound() == right_lanelet.leftBound())
        return right_lanelet;
    return std::nullopt;
  }

  // right shoulder lanelet
  if (get_shoulder_lane) {
    const auto right_shoulder_lanelet = getRightShoulderLanelet(lanelet);
    if (right_shoulder_lanelet) {
      return *right_shoulder_lanelet;
    }
  }

  // routable lane
  const auto & right_lane = routing_graph_ptr_->right(lanelet);
  if (right_lane) {
    return *right_lane;
  }

  // non-routable lane (e.g. lane change infeasible)
  const auto & adjacent_right_lane = routing_graph_ptr_->adjacentRight(lanelet);
  if (adjacent_right_lane) {
    return *adjacent_right_lane;
  }

  // same root right lanelet
  if (!enable_same_root) {
    return std::nullopt;
  }

  lanelet::ConstLanelets prev_lanelet;
  if (!getPreviousLaneletsWithinRoute(lanelet, &prev_lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelet next_lanelet;
  if (!getNextLaneletWithinRoute(lanelet, &next_lanelet)) {
    for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
      if (
        lanelet.rightBound().back().id() == lane.leftBound().back().id() &&
        lane.id() != lanelet.id()) {
        return lane;
      }
    }
    return std::nullopt;
  }

  const auto next_right_lane = getRightLanelet(next_lanelet, false);
  if (!next_right_lane) {
    return std::nullopt;
  }

  for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
    for (const auto & target_lane : getNextLanelets(lane)) {
      if (next_right_lane.value().id() == target_lane.id() && lane.id() != lanelet.id()) {
        return lane;
      }
    }
  }

  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getLeftLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // left road lanelet of shoulder lanelet
  if (isShoulderLanelet(lanelet)) {
    const auto left_lanelets = lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound());
    for (const auto & left_lanelet : left_lanelets)
      if (isRoadLanelet(left_lanelet) && lanelet.leftBound() == left_lanelet.rightBound())
        return left_lanelet;
    return std::nullopt;
  }

  // left shoulder lanelet
  if (get_shoulder_lane) {
    const auto left_shoulder_lanelet = getLeftShoulderLanelet(lanelet);
    if (left_shoulder_lanelet) {
      return *left_shoulder_lanelet;
    }
  }

  // routable lane
  const auto & left_lane = routing_graph_ptr_->left(lanelet);
  if (left_lane) {
    return *left_lane;
  }

  // non-routable lane (e.g. lane change infeasible)
  const auto & adjacent_left_lane = routing_graph_ptr_->adjacentLeft(lanelet);
  if (adjacent_left_lane) {
    return *adjacent_left_lane;
  }

  // same root right lanelet
  if (!enable_same_root) {
    return std::nullopt;
  }

  lanelet::ConstLanelets prev_lanelet;
  if (!getPreviousLaneletsWithinRoute(lanelet, &prev_lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelet next_lanelet;
  if (!getNextLaneletWithinRoute(lanelet, &next_lanelet)) {
    for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
      if (
        lanelet.leftBound().back().id() == lane.rightBound().back().id() &&
        lane.id() != lanelet.id()) {
        return lane;
      }
    }
    return std::nullopt;
  }

  const auto next_left_lane = getLeftLanelet(next_lanelet, false);
  if (!next_left_lane) {
    return std::nullopt;
  }

  for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
    for (const auto & target_lane : getNextLanelets(lane)) {
      if (next_left_lane.value().id() == target_lane.id() && lane.id() != lanelet.id()) {
        return lane;
      }
    }
  }

  return std::nullopt;
}

lanelet::Lanelets RouteHandler::getRightOppositeLanelets(
  const lanelet::ConstLanelet & lanelet) const
{
  const auto opposite_candidate_lanelets =
    lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound().invert());

  lanelet::Lanelets opposite_lanelets;
  for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
    if (candidate_lanelet.leftBound().id() == lanelet.rightBound().id()) {
      continue;
    }

    opposite_lanelets.push_back(candidate_lanelet);
  }

  return opposite_lanelets;
}

lanelet::ConstLanelets RouteHandler::getAllLeftSharedLinestringLanelets(
  const lanelet::ConstLanelet & lane, const bool & include_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets linestring_shared;
  try {
    auto lanelet_at_left = getLeftLanelet(lane);
    auto lanelet_at_left_opposite = getLeftOppositeLanelets(lane);
    while (lanelet_at_left) {
      linestring_shared.push_back(lanelet_at_left.value());
      lanelet_at_left = getLeftLanelet(lanelet_at_left.value());
      if (!lanelet_at_left) {
        break;
      }
      lanelet_at_left_opposite = getLeftOppositeLanelets(lanelet_at_left.value());
    }

    if (!lanelet_at_left_opposite.empty() && include_opposite) {
      if (invert_opposite) {
        linestring_shared.push_back(lanelet_at_left_opposite.front().invert());
      } else {
        linestring_shared.push_back(lanelet_at_left_opposite.front());
      }
      auto lanelet_at_right = getRightLanelet(lanelet_at_left_opposite.front());
      while (lanelet_at_right) {
        if (invert_opposite) {
          linestring_shared.push_back(lanelet_at_right.value().invert());
        } else {
          linestring_shared.push_back(lanelet_at_right.value());
        }
        lanelet_at_right = getRightLanelet(lanelet_at_right.value());
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "Exception in getAllLeftSharedLinestringLanelets: " << e.what() << std::endl;
    return {};
  } catch (...) {
    std::cerr << "Unknown exception in getAllLeftSharedLinestringLanelets" << std::endl;
    return {};
  }
  return linestring_shared;
}

lanelet::ConstLanelets RouteHandler::getAllRightSharedLinestringLanelets(
  const lanelet::ConstLanelet & lane, const bool & include_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets linestring_shared;
  try {
    auto lanelet_at_right = getRightLanelet(lane);
    auto lanelet_at_right_opposite = getRightOppositeLanelets(lane);
    while (lanelet_at_right) {
      linestring_shared.push_back(lanelet_at_right.value());
      lanelet_at_right = getRightLanelet(lanelet_at_right.value());
      if (!lanelet_at_right) {
        break;
      }
      lanelet_at_right_opposite = getRightOppositeLanelets(lanelet_at_right.value());
    }

    if (!lanelet_at_right_opposite.empty() && include_opposite) {
      if (invert_opposite) {
        linestring_shared.push_back(lanelet_at_right_opposite.front().invert());
      } else {
        linestring_shared.push_back(lanelet_at_right_opposite.front());
      }
      auto lanelet_at_left = getLeftLanelet(lanelet_at_right_opposite.front());
      while (lanelet_at_left) {
        if (invert_opposite) {
          linestring_shared.push_back(lanelet_at_left.value().invert());
        } else {
          linestring_shared.push_back(lanelet_at_left.value());
        }
        lanelet_at_left = getLeftLanelet(lanelet_at_left.value());
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "Exception in getAllRightSharedLinestringLanelets: " << e.what() << std::endl;
    return {};
  } catch (...) {
    std::cerr << "Unknown exception in getAllRightSharedLinestringLanelets" << std::endl;
    return {};
  }
  return linestring_shared;
}

lanelet::Lanelets RouteHandler::getLeftOppositeLanelets(const lanelet::ConstLanelet & lanelet) const
{
  const auto opposite_candidate_lanelets =
    lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound().invert());

  lanelet::Lanelets opposite_lanelets;
  for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
    if (candidate_lanelet.rightBound().id() == lanelet.leftBound().id()) {
      continue;
    }

    opposite_lanelets.push_back(candidate_lanelet);
  }

  return opposite_lanelets;
}

lanelet::ConstLanelet RouteHandler::getMostRightLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // recursively compute the width of the lanes
  const auto & same = getRightLanelet(lanelet, enable_same_root, get_shoulder_lane);

  if (same) {
    return getMostRightLanelet(same.value(), enable_same_root, get_shoulder_lane);
  }

  return lanelet;
}

lanelet::ConstLanelet RouteHandler::getMostLeftLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // recursively compute the width of the lanes
  const auto & same = getLeftLanelet(lanelet, enable_same_root, get_shoulder_lane);

  if (same) {
    return getMostLeftLanelet(same.value(), enable_same_root, get_shoulder_lane);
  }

  return lanelet;
}

std::vector<lanelet::ConstLanelets> RouteHandler::getPrecedingLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const double length,
  const lanelet::ConstLanelets & exclude_lanelets) const
{
  return lanelet::utils::query::getPrecedingLaneletSequences(
    routing_graph_ptr_, lanelet, length, exclude_lanelets);
}

std::optional<lanelet::ConstLanelet> RouteHandler::getLaneChangeTarget(
  const lanelet::ConstLanelets & lanelets, const Direction direction) const
{
  for (const auto & lanelet : lanelets) {
    const int num = getNumLaneToPreferredLane(lanelet, direction);
    if (num == 0) {
      continue;
    }

    if (direction == Direction::NONE || direction == Direction::RIGHT) {
      if (num < 0) {
        const auto right_lanes = routing_graph_ptr_->right(lanelet);
        if (!!right_lanes) {
          return *right_lanes;
        }
      }
    }

    if (direction == Direction::NONE || direction == Direction::LEFT) {
      if (num > 0) {
        const auto left_lanes = routing_graph_ptr_->left(lanelet);
        if (!!left_lanes) {
          return *left_lanes;
        }
      }
    }
  }

  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getLaneChangeTargetExceptPreferredLane(
  const lanelet::ConstLanelets & lanelets, const Direction direction) const
{
  for (const auto & lanelet : lanelets) {
    if (direction == Direction::RIGHT) {
      // Get right lanelet if preferred lane is on the left
      if (getNumLaneToPreferredLane(lanelet, direction) < 0) {
        continue;
      }

      const auto right_lanes = routing_graph_ptr_->right(lanelet);
      if (!!right_lanes) {
        return *right_lanes;
      }
    }

    if (direction == Direction::LEFT) {
      // Get left lanelet if preferred lane is on the right
      if (getNumLaneToPreferredLane(lanelet, direction) > 0) {
        continue;
      }
      const auto left_lanes = routing_graph_ptr_->left(lanelet);
      if (!!left_lanes) {
        return *left_lanes;
      }
    }
  }

  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getPullOverTarget(const Pose & goal_pose) const
{
  const lanelet::BasicPoint2d p(goal_pose.position.x, goal_pose.position.y);
  constexpr auto search_distance = 0.1;
  const lanelet::BasicPoint2d offset(search_distance, search_distance);
  const auto lanelets_in_range =
    lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p - offset, p + offset));
  for (const auto & lanelet : lanelets_in_range) {
    const auto is_in_lanelet = lanelet::utils::isInLanelet(goal_pose, lanelet, search_distance);
    if (is_in_lanelet && isShoulderLanelet(lanelet)) return lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getPullOutStartLane(
  const Pose & pose, const double vehicle_width) const
{
  const lanelet::BasicPoint2d p(pose.position.x, pose.position.y);
  const auto search_distance = vehicle_width / 2.0;
  const lanelet::BasicPoint2d offset(search_distance, search_distance);
  const auto lanelets_in_range =
    lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p - offset, p + offset));
  for (const auto & lanelet : lanelets_in_range) {
    const auto is_in_lanelet = lanelet::utils::isInLanelet(pose, lanelet, search_distance);
    if (is_in_lanelet && isShoulderLanelet(lanelet)) return lanelet;
  }
  return std::nullopt;
}

int RouteHandler::getNumLaneToPreferredLane(
  const lanelet::ConstLanelet & lanelet, const Direction direction) const
{
  if (exists(preferred_lanelets_, lanelet)) {
    return 0;
  }

  if ((direction == Direction::NONE) || (direction == Direction::RIGHT)) {
    int num{0};
    const auto & right_lanes =
      lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet);
    for (const auto & right : right_lanes) {
      num--;
      if (exists(preferred_lanelets_, right)) {
        return num;
      }
    }
  }

  if ((direction == Direction::NONE) || (direction == Direction::LEFT)) {
    const auto & left_lanes =
      lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet);
    int num = 0;
    for (const auto & left : left_lanes) {
      num++;
      if (exists(preferred_lanelets_, left)) {
        return num;
      }
    }
  }

  return 0;  // TODO(Horibe) check if return 0 is appropriate.
}

std::vector<double> RouteHandler::getLateralIntervalsToPreferredLane(
  const lanelet::ConstLanelet & lanelet, const Direction direction) const
{
  if (exists(preferred_lanelets_, lanelet)) {
    return {};
  }

  if ((direction == Direction::NONE) || (direction == Direction::RIGHT)) {
    std::vector<double> intervals;
    lanelet::ConstLanelet current_lanelet = lanelet;
    const auto & right_lanes =
      lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet);
    for (const auto & right : right_lanes) {
      const auto & current_centerline = current_lanelet.centerline();
      const auto & next_centerline = right.centerline();
      if (current_centerline.empty() || next_centerline.empty()) {
        return intervals;
      }
      const auto & curr_pt = current_centerline.front();
      const auto & next_pt = next_centerline.front();
      intervals.push_back(-lanelet::geometry::distance2d(to2D(curr_pt), to2D(next_pt)));

      if (exists(preferred_lanelets_, right)) {
        return intervals;
      }
      current_lanelet = right;
    }
  }

  if ((direction == Direction::NONE) || (direction == Direction::LEFT)) {
    std::vector<double> intervals;
    lanelet::ConstLanelet current_lanelet = lanelet;
    const auto & left_lanes =
      lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet);
    for (const auto & left : left_lanes) {
      const auto & current_centerline = current_lanelet.centerline();
      const auto & next_centerline = left.centerline();
      if (current_centerline.empty() || next_centerline.empty()) {
        return intervals;
      }
      const auto & curr_pt = current_centerline.front();
      const auto & next_pt = next_centerline.front();
      intervals.push_back(lanelet::geometry::distance2d(to2D(curr_pt), to2D(next_pt)));

      if (exists(preferred_lanelets_, left)) {
        return intervals;
      }
      current_lanelet = left;
    }
  }

  return {};
}

PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
  bool use_exact) const
{
  using lanelet::utils::to2D;
  using lanelet::utils::conversion::toLaneletPoint;

  // 1. calculate reference points by lanelets' centerline
  // NOTE: This vector aligns the vector lanelet_sequence.
  std::vector<PiecewiseReferencePoints> piecewise_ref_points_vec;
  for (const auto & llt : lanelet_sequence) {
    const lanelet::ConstLineString3d centerline = llt.centerline();

    piecewise_ref_points_vec.push_back(std::vector<ReferencePoint>{});
    for (const auto & center_point : centerline) {
      piecewise_ref_points_vec.back().push_back(
        ReferencePoint{false, lanelet::utils::conversion::toGeomMsgPt(center_point)});
    }
  }

  // 2. calculate waypoints
  const auto waypoints_vec = calcWaypointsVector(lanelet_sequence);

  // 3. remove points in the margin of the waypoint
  for (const auto & waypoints : waypoints_vec) {
    for (auto piecewise_waypoints_itr = waypoints.begin();
         piecewise_waypoints_itr != waypoints.end(); ++piecewise_waypoints_itr) {
      const auto & piecewise_waypoints = piecewise_waypoints_itr->piecewise_waypoints;
      const auto lanelet_id = piecewise_waypoints_itr->lanelet_id;

      // calculate index of lanelet_sequence which corresponds to piecewise_waypoints.
      const auto lanelet_sequence_itr = std::find_if(
        lanelet_sequence.begin(), lanelet_sequence.end(),
        [&](const auto & lanelet) { return lanelet.id() == lanelet_id; });
      if (lanelet_sequence_itr == lanelet_sequence.end()) {
        continue;
      }
      const size_t piecewise_waypoints_lanelet_sequence_index =
        std::distance(lanelet_sequence.begin(), lanelet_sequence_itr);

      // calculate reference points by waypoints
      const auto ref_points_by_waypoints = convertWaypointsToReferencePoints(piecewise_waypoints);

      // update reference points by waypoints
      const bool is_first_waypoint_contained = piecewise_waypoints_itr == waypoints.begin();
      const bool is_last_waypoint_contained = piecewise_waypoints_itr == waypoints.end() - 1;
      if (is_first_waypoint_contained || is_last_waypoint_contained) {
        // If piecewise_waypoints_itr is the end (first or last) of piecewise_waypoints

        const auto original_piecewise_ref_points =
          piecewise_ref_points_vec.at(piecewise_waypoints_lanelet_sequence_index);

        // define current_piecewise_ref_points, and initialize it with waypoints
        auto & current_piecewise_ref_points =
          piecewise_ref_points_vec.at(piecewise_waypoints_lanelet_sequence_index);
        current_piecewise_ref_points = ref_points_by_waypoints;
        if (is_first_waypoint_contained) {
          // add original reference points to current reference points, and remove reference points
          // overlapped with waypoints
          current_piecewise_ref_points.insert(
            current_piecewise_ref_points.begin(), original_piecewise_ref_points.begin(),
            original_piecewise_ref_points.end());
          const bool is_removing_direction_forward = false;
          removeOverlappedCenterlineWithWaypoints(
            piecewise_ref_points_vec, piecewise_waypoints, lanelet_sequence,
            piecewise_waypoints_lanelet_sequence_index, is_removing_direction_forward);
        }
        if (is_last_waypoint_contained) {
          // add original reference points to current reference points, and remove reference points
          // overlapped with waypoints
          current_piecewise_ref_points.insert(
            current_piecewise_ref_points.end(), original_piecewise_ref_points.begin(),
            original_piecewise_ref_points.end());
          const bool is_removing_direction_forward = true;
          removeOverlappedCenterlineWithWaypoints(
            piecewise_ref_points_vec, piecewise_waypoints, lanelet_sequence,
            piecewise_waypoints_lanelet_sequence_index, is_removing_direction_forward);
        }
      } else {
        // If piecewise_waypoints_itr is not the end (first or last) of piecewise_waypoints,
        // remove all the reference points and add waypoints.
        piecewise_ref_points_vec.at(piecewise_waypoints_lanelet_sequence_index) =
          ref_points_by_waypoints;
      }
    }
  }

  PathWithLaneId reference_path{};
  const auto add_path_point =
    [&](const auto & point, const auto & lanelet, const auto & speed_limit) {
      PathPointWithLaneId p{};
      p.point.pose.position = point;
      p.lane_ids.push_back(lanelet.id());
      p.point.longitudinal_velocity_mps = speed_limit;
      reference_path.points.push_back(p);
    };

  // 4. convert to PathPointsWithLaneIds with cropping
  double s = 0.0;
  for (size_t lanelet_idx = 0; lanelet_idx < lanelet_sequence.size(); ++lanelet_idx) {
    const auto & lanelet = lanelet_sequence.at(lanelet_idx);
    const float speed_limit =
      static_cast<float>(traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value());

    const auto & piecewise_ref_points = piecewise_ref_points_vec.at(lanelet_idx);
    for (size_t ref_point_idx = 0; ref_point_idx < piecewise_ref_points.size(); ++ref_point_idx) {
      const auto & ref_point = piecewise_ref_points.at(ref_point_idx);
      const auto & next_ref_point = (ref_point_idx + 1 < piecewise_ref_points.size())
                                      ? piecewise_ref_points.at(ref_point_idx + 1)
                                      : piecewise_ref_points.at(ref_point_idx);

      const double distance =
        autoware_utils_geometry::calc_distance2d(ref_point.point, next_ref_point.point);

      if (s < s_start && s + distance > s_start) {
        const auto p_opt = getGeometryPointFrom2DArcLength(lanelet_sequence, s_start);
        if (p_opt.has_value()) {
          const auto p = use_exact ? p_opt.value() : ref_point.point;
          add_path_point(p, lanelet, speed_limit);
        } else {
          add_path_point(ref_point.point, lanelet, speed_limit);
        }
      }
      if (s >= s_start && s <= s_end) {
        add_path_point(ref_point.point, lanelet, speed_limit);
      }
      if (s < s_end && s + distance > s_end) {
        const auto p_opt = getGeometryPointFrom2DArcLength(lanelet_sequence, s_end);
        if (p_opt.has_value()) {
          const auto p = use_exact ? p_opt.value() : ref_point.point;
          add_path_point(p, lanelet, speed_limit);
        } else {
          add_path_point(ref_point.point, lanelet, speed_limit);
        }
      }
      s += distance;
    }
  }
  reference_path = removeOverlappingPoints(reference_path);

  // append a point only when having one point so that yaw calculation would work
  if (reference_path.points.size() == 1) {
    const lanelet::Id lane_id = reference_path.points.front().lane_ids.front();
    const lanelet::ConstLanelet forward_lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
    // Bidirectional-driving support: laneletLayer.get() always returns the forward view, so
    // re-apply the tracked direction before computing yaw, otherwise the synthesized point below
    // would point the wrong way on a reversed segment.
    const lanelet::ConstLanelet lanelet =
      isLaneletInvertedInRoute(forward_lanelet) ? forward_lanelet.invert() : forward_lanelet;
    const auto point = reference_path.points.front().point.pose.position;
    const auto lane_yaw = autoware::experimental::lanelet2_utils::get_lanelet_angle(
      lanelet, autoware::experimental::lanelet2_utils::from_ros(point).basicPoint());
    PathPointWithLaneId path_point{};
    path_point.lane_ids.push_back(lane_id);
    constexpr double ds{0.1};
    path_point.point.pose.position.x = point.x + ds * std::cos(lane_yaw);
    path_point.point.pose.position.y = point.y + ds * std::sin(lane_yaw);
    path_point.point.pose.position.z = point.z;
    reference_path.points.push_back(path_point);
  }

  // set angle
  for (size_t i = 0; i < reference_path.points.size(); i++) {
    double angle{0.0};
    const auto & pts = reference_path.points;
    if (i + 1 < reference_path.points.size()) {
      angle = autoware_utils_geometry::calc_azimuth_angle(
        pts.at(i).point.pose.position, pts.at(i + 1).point.pose.position);
    } else if (i != 0) {
      angle = autoware_utils_geometry::calc_azimuth_angle(
        pts.at(i - 1).point.pose.position, pts.at(i).point.pose.position);
    }
    reference_path.points.at(i).point.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(angle);
  }

  return reference_path;
}

std::vector<Waypoints> RouteHandler::calcWaypointsVector(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  std::vector<Waypoints> waypoints_vec;
  for (size_t lanelet_idx = 0; lanelet_idx < lanelet_sequence.size(); ++lanelet_idx) {
    const auto & lanelet = lanelet_sequence.at(lanelet_idx);
    if (!lanelet.hasAttribute("waypoints")) {
      continue;
    }

    // generate piecewise waypoints
    //
    // BIDIR-BUG-FIX: lineStringLayer.get(waypoints_id) always yields the waypoints linestring in
    // the order it was authored in the map (i.e. matching the lanelet's *forward* direction),
    // completely independent of whether `lanelet` here is an inverted ConstLanelet view. This is
    // the same re-fetch-by-id direction-loss bug class already fixed in route_handler.cpp for
    // route_lanelets_/getNextLanelets()/getPreviousLanelets(), just never audited in this
    // function. Left unfixed, a reversed-in-route lanelet carrying an explicit "waypoints"
    // override (typically used on curves, exactly where auto-centerline flipping isn't enough)
    // would have its waypoint reference points ordered backwards relative to the actual direction
    // of travel, producing an erratic/self-crossing reference path on that lanelet while adjacent
    // plain-centerline lanelets look fine -- matching the reported "trajectory doesn't follow the
    // reversed lanelet, sometimes erratic" symptom.
    PiecewiseWaypoints piecewise_waypoints{lanelet.id(), {}};
    const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
    for (const auto & waypoint : lanelet_map_ptr_->lineStringLayer.get(waypoints_id)) {
      piecewise_waypoints.piecewise_waypoints.push_back(
        lanelet::utils::conversion::toGeomMsgPt(waypoint));
    }
    if (lanelet.inverted()) {
      std::reverse(
        piecewise_waypoints.piecewise_waypoints.begin(),
        piecewise_waypoints.piecewise_waypoints.end());
    }
    if (piecewise_waypoints.piecewise_waypoints.empty()) {
      continue;
    }

    // check if the piecewise waypoints are connected to the previous piecewise waypoints
    if (
      !waypoints_vec.empty() && isClose(
                                  waypoints_vec.back().back().piecewise_waypoints.back(),
                                  piecewise_waypoints.piecewise_waypoints.front(), 1.0)) {
      waypoints_vec.back().push_back(piecewise_waypoints);
    } else {
      // add new waypoints
      Waypoints new_waypoints;
      new_waypoints.push_back(piecewise_waypoints);
      waypoints_vec.push_back(new_waypoints);
    }
  }

  return waypoints_vec;
}

void RouteHandler::removeOverlappedCenterlineWithWaypoints(
  std::vector<PiecewiseReferencePoints> & piecewise_ref_points_vec,
  const std::vector<geometry_msgs::msg::Point> & piecewise_waypoints,
  const lanelet::ConstLanelets & lanelet_sequence,
  const size_t piecewise_waypoints_lanelet_sequence_index,
  const bool is_removing_direction_forward) const
{
  const double waypoints_interpolation_arc_margin_ratio = 10.0;

  // calculate arc length threshold
  const double front_arc_length_threshold = [&]() {
    const auto front_waypoint_arc_coordinates = calcArcCoordinates(
      lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index), piecewise_waypoints.front());
    const double lanelet_arc_length = boost::geometry::length(
      lanelet::utils::to2D(lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index)
                             .centerline()
                             .basicLineString()));
    return -lanelet_arc_length + front_waypoint_arc_coordinates.length -
           std::abs(front_waypoint_arc_coordinates.distance) *
             waypoints_interpolation_arc_margin_ratio;
  }();
  const double back_arc_length_threshold = [&]() {
    const auto back_waypoint_arc_coordinates = calcArcCoordinates(
      lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index), piecewise_waypoints.back());
    return back_waypoint_arc_coordinates.length + std::abs(back_waypoint_arc_coordinates.distance) *
                                                    waypoints_interpolation_arc_margin_ratio;
  }();

  double offset_arc_length = 0.0;
  int target_lanelet_sequence_index = static_cast<int>(piecewise_waypoints_lanelet_sequence_index);
  while (isIndexWithinVector(lanelet_sequence, target_lanelet_sequence_index)) {
    auto & target_piecewise_ref_points = piecewise_ref_points_vec.at(target_lanelet_sequence_index);
    const double target_lanelet_arc_length = boost::geometry::length(lanelet::utils::to2D(
      lanelet_sequence.at(target_lanelet_sequence_index).centerline().basicLineString()));

    // search overlapped ref points in the target lanelet
    std::vector<size_t> overlapped_ref_points_indices{};
    const bool is_search_finished = [&]() {
      for (size_t ref_point_unsigned_index = 0;
           ref_point_unsigned_index < target_piecewise_ref_points.size();
           ++ref_point_unsigned_index) {
        const size_t ref_point_index =
          is_removing_direction_forward
            ? ref_point_unsigned_index
            : target_piecewise_ref_points.size() - 1 - ref_point_unsigned_index;
        const auto & ref_point = target_piecewise_ref_points.at(ref_point_index);

        // skip waypoints
        if (ref_point.is_waypoint) {
          if (
            target_lanelet_sequence_index ==
            static_cast<int>(piecewise_waypoints_lanelet_sequence_index)) {
            overlapped_ref_points_indices.clear();
          }
          continue;
        }

        const double ref_point_arc_length =
          (is_removing_direction_forward ? 0 : -target_lanelet_arc_length) +
          calcArcCoordinates(lanelet_sequence.at(target_lanelet_sequence_index), ref_point.point)
            .length;
        if (is_removing_direction_forward) {
          if (back_arc_length_threshold < offset_arc_length + ref_point_arc_length) {
            return true;
          }
        } else {
          if (offset_arc_length + ref_point_arc_length < front_arc_length_threshold) {
            return true;
          }
        }

        overlapped_ref_points_indices.push_back(ref_point_index);
      }
      return false;
    }();

    // remove overlapped indices from ref_points
    removeIndicesFromVector(target_piecewise_ref_points, overlapped_ref_points_indices);

    // break if searching overlapped centerline is finished.
    if (is_search_finished) {
      break;
    }

    target_lanelet_sequence_index += is_removing_direction_forward ? 1 : -1;
    offset_arc_length = (is_removing_direction_forward ? 1 : -1) * target_lanelet_arc_length;
  }
}

bool RouteHandler::isMapMsgReady() const
{
  return is_map_msg_ready_;
}

lanelet::routing::RoutingGraphPtr RouteHandler::getRoutingGraphPtr() const
{
  return routing_graph_ptr_;
}

lanelet::traffic_rules::TrafficRulesPtr RouteHandler::getTrafficRulesPtr() const
{
  return traffic_rules_ptr_;
}

std::shared_ptr<const lanelet::routing::RoutingGraphContainer> RouteHandler::getOverallGraphPtr()
  const
{
  return overall_graphs_ptr_;
}

lanelet::LaneletMapPtr RouteHandler::getLaneletMapPtr() const
{
  return lanelet_map_ptr_;
}

bool RouteHandler::isShoulderLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
         lanelet.attribute(lanelet::AttributeName::Subtype) == "road_shoulder";
}

bool RouteHandler::isRouteLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet::utils::contains(route_lanelets_, lanelet);
}

bool RouteHandler::isRoadLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
         lanelet.attribute(lanelet::AttributeName::Subtype) == lanelet::AttributeValueString::Road;
}

lanelet::ConstLanelets RouteHandler::getPreviousLaneletSequence(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  lanelet::ConstLanelets previous_lanelet_sequence;
  if (lanelet_sequence.empty()) {
    return previous_lanelet_sequence;
  }

  const auto & first_lane = lanelet_sequence.front();
  if (exists(start_lanelets_, first_lane)) {
    return previous_lanelet_sequence;
  }

  auto right_relations =
    lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, first_lane);
  for (const auto & right : right_relations) {
    previous_lanelet_sequence = getLaneletSequenceUpTo(right);
    if (!previous_lanelet_sequence.empty()) {
      return previous_lanelet_sequence;
    }
  }

  auto left_relations = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, first_lane);
  for (const auto & left : left_relations) {
    previous_lanelet_sequence = getLaneletSequenceUpTo(left);
    if (!previous_lanelet_sequence.empty()) {
      return previous_lanelet_sequence;
    }
  }
  return previous_lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getNeighborsWithinRoute(
  const lanelet::ConstLanelet & lanelet) const
{
  const lanelet::ConstLanelets neighbor_lanelets =
    lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, lanelet);
  lanelet::ConstLanelets neighbors_within_route;
  for (const auto & llt : neighbor_lanelets) {
    if (exists(route_lanelets_, llt)) {
      neighbors_within_route.push_back(llt);
    }
  }
  return neighbors_within_route;
}

bool RouteHandler::planPathLaneletsBetweenCheckpoints(
  const Pose & start_checkpoint, const Pose & goal_checkpoint,
  lanelet::ConstLanelets * path_lanelets, const bool consider_no_drivable_lanes) const
{
  // Find lanelets for start point. First, find all lanelets containing the start point to calculate
  // all possible route later. It fails when the point is not located on any road lanelet (e.g. the
  // start point is located out of any lanelets or road_shoulder lanelet which is not contained in
  // road lanelet). In that case, find the closest lanelet instead (within some maximum range).
  constexpr auto max_search_range = 20.0;
  auto start_lanelets = getRoadLaneletsAtPose(start_checkpoint);
  lanelet::ConstLanelet start_lanelet;
  if (start_lanelets.empty()) {
    const lanelet::BasicPoint2d p(start_checkpoint.position.x, start_checkpoint.position.y);
    const lanelet::BoundingBox2d bbox(
      lanelet::BasicPoint2d(p.x() - max_search_range, p.y() - max_search_range),
      lanelet::BasicPoint2d(p.x() + max_search_range, p.y() + max_search_range));
    // std::as_const(*ptr) to use the const version of the search function
    auto candidates = std::as_const(*lanelet_map_ptr_).laneletLayer.search(bbox);
    candidates.erase(
      std::remove_if(
        candidates.begin(), candidates.end(), [&](const auto & l) { return !isRoadLanelet(l); }),
      candidates.end());
    if (lanelet::utils::query::getClosestLanelet(candidates, start_checkpoint, &start_lanelet))
      start_lanelets = {start_lanelet};
  }
  if (start_lanelets.empty()) {
    RCLCPP_WARN_STREAM(
      logger_, "Failed to find current lanelet."
                 << std::endl
                 << " - start checkpoint: " << toString(start_checkpoint) << std::endl
                 << " - goal checkpoint: " << toString(goal_checkpoint) << std::endl);
    return false;
  }

  // Find lanelets for goal point.
  lanelet::ConstLanelet goal_lanelet;
  const lanelet::BasicPoint2d p(goal_checkpoint.position.x, goal_checkpoint.position.y);
  const lanelet::BoundingBox2d bbox(
    lanelet::BasicPoint2d(p.x() - max_search_range, p.y() - max_search_range),
    lanelet::BasicPoint2d(p.x() + max_search_range, p.y() + max_search_range));
  auto candidates = std::as_const(*lanelet_map_ptr_).laneletLayer.search(bbox);
  candidates.erase(
    std::remove_if(
      candidates.begin(), candidates.end(), [&](const auto & l) { return !isRoadLanelet(l); }),
    candidates.end());
  // if there is a lanelet in candidates that is included in previous preferred lanelets,
  // set it as goal_lanelet.
  // this is to select the same lane as much as possible when rerouting with waypoints.
  const auto findGoalClosestPreferredLanelet = [&]() -> std::optional<lanelet::ConstLanelet> {
    lanelet::ConstLanelet closest_lanelet;
    if (getClosestPreferredLaneletWithinRoute(goal_checkpoint, &closest_lanelet)) {
      if (std::find(candidates.begin(), candidates.end(), closest_lanelet) != candidates.end()) {
        if (lanelet::utils::isInLanelet(goal_checkpoint, closest_lanelet)) {
          return closest_lanelet;
        }
      }
    }
    if (getClosestLaneletWithinRoute(goal_checkpoint, &closest_lanelet)) {
      if (std::find(candidates.begin(), candidates.end(), closest_lanelet) != candidates.end()) {
        if (lanelet::utils::isInLanelet(goal_checkpoint, closest_lanelet)) {
          std::stringstream preferred_lanelets_str;
          for (const auto & preferred_lanelet : preferred_lanelets_) {
            preferred_lanelets_str << preferred_lanelet.id() << ", ";
          }
          RCLCPP_WARN(
            logger_,
            "Failed to find reroute on previous preferred lanelets %s, but on previous route "
            "segment %ld still",
            preferred_lanelets_str.str().c_str(), closest_lanelet.id());
          return closest_lanelet;
        }
      }
    }
    return std::nullopt;
  };
  if (auto closest_lanelet = findGoalClosestPreferredLanelet()) {
    goal_lanelet = closest_lanelet.value();
  } else {
    if (!lanelet::utils::query::getClosestLanelet(candidates, goal_checkpoint, &goal_lanelet)) {
      RCLCPP_WARN_STREAM(
        logger_, "Failed to find closest lanelet."
                   << std::endl
                   << " - start checkpoint: " << toString(start_checkpoint) << std::endl
                   << " - goal checkpoint: " << toString(goal_checkpoint) << std::endl);
      return false;
    }
  }

  lanelet::Optional<lanelet::routing::Route> optional_route;
  lanelet::routing::LaneletPath shortest_path;
  bool is_route_found = false;

  double min_route_cost = std::numeric_limits<double>::max();
  constexpr double yaw_threshold = M_PI / 2.0;
  constexpr double angle_diff_weight = 1000.0;
  // [BIDIR-BUG-FIX] Live-test evidence (2026-08-17): forward candidate length2d=64.60 vs
  // reverse-start+reverse-goal candidate length2d=62.50 for the same start/goal -- reverse won
  // the raw cost comparison by a razor-thin 3.4% margin, even though reversing has real practical
  // costs a plain 2D path-length metric can't see (lower speed limit while reversing, must
  // decelerate to near-zero before/after the maneuver, harder to control precisely, awkward for
  // a human observer). Without a penalty, the router will flip-flop to reverse for ANY marginal
  // length advantage, however tiny -- reverse should only ever be chosen when it is SUBSTANTIALLY
  // shorter than the forward alternative, not just nominally shorter. Multiplicatively penalize
  // reverse-start candidate lengths before they compete with the forward cost below; a forward
  // route must be more than this factor longer than a reverse one before reverse is allowed to
  // win. Hardcoded here (matching this function's existing inline-constexpr style for
  // yaw_threshold/angle_diff_weight) rather than wired through a new ROS param -- revisit via
  // tuning-engineer + a proper param if the exact factor needs field-tuning.
  constexpr double reverse_route_length_penalty_factor = 1.5;

  for (const auto & st_llt : start_lanelets) {
    // check if the angle difference between start_checkpoint and start lanelet center line
    // orientation is in yaw_threshold range
    double lanelet_angle = autoware::experimental::lanelet2_utils::get_lanelet_angle(
      st_llt,
      autoware::experimental::lanelet2_utils::from_ros(start_checkpoint.position).basicPoint());
    double pose_yaw = tf2::getYaw(start_checkpoint.orientation);
    double angle_diff = std::abs(autoware_utils_math::normalize_radian(lanelet_angle - pose_yaw));

    bool is_proper_angle = angle_diff <= std::abs(yaw_threshold);

    optional_route = routing_graph_ptr_->getRoute(st_llt, goal_lanelet, 0);
    RCLCPP_WARN(
      logger_,
      "[BIDIR-DEBUG] forward candidate: st_llt=%ld found=%s is_proper_angle=%s angle_diff_deg=%.2f "
      "length2d=%.2f",
      st_llt.id(), (optional_route ? "true" : "false"), (is_proper_angle ? "true" : "false"),
      angle_diff * 180.0 / M_PI, (optional_route ? optional_route->length2d() : -1.0));
    if (!optional_route || !is_proper_angle) {
      RCLCPP_DEBUG_STREAM(
        logger_, "Failed to find a proper route!"
                   << std::endl
                   << " - start checkpoint: " << toString(start_checkpoint) << std::endl
                   << " - goal checkpoint: " << toString(goal_checkpoint) << std::endl
                   << " - start lane id: " << st_llt.id() << std::endl
                   << " - goal lane id: " << goal_lanelet.id() << std::endl);
      continue;
    }
    is_route_found = true;
    lanelet::ConstLanelet preferred_lane{};
    if (getClosestPreferredLaneletWithinRoute(start_checkpoint, &preferred_lane)) {
      if (st_llt.id() == preferred_lane.id()) {
        shortest_path = optional_route->shortestPath();
        start_lanelet = st_llt;
        break;
      }
    }
    const double optional_route_length = optional_route->length2d();
    const double optional_route_cost = optional_route_length + angle_diff_weight * angle_diff;
    RCLCPP_DEBUG(
      logger_, "Lanelet ID %ld: Route length = %.1f, Angle Diff = %.4f rad, Route cost = %.2f",
      st_llt.id(), optional_route_length, angle_diff, optional_route_cost);
    if (optional_route_cost < min_route_cost) {
      min_route_cost = optional_route_cost;
      shortest_path = optional_route->shortestPath();
      start_lanelet = st_llt;
    }

    // Also consider the *inverted* (reverse-departing) orientation of this same start lanelet.
    // `getRoadLaneletsAtPose()` only ever returns forward-oriented ConstLanelets, and
    // `routing_graph_ptr_->getRoute(from, to, ...)` only searches the routing graph reachable
    // from exactly the orientation of `from` that is passed in -- it does not implicitly also try
    // the inverted view. So without this block, a route that begins with ego backing straight out
    // of its current lanelet is never even generated as a search candidate, regardless of what the
    // routing graph itself supports (see bidirectional_plan/04-routing-foundation.md §4b and the
    // post-hoc reject-gate below, which can only ever fire on inverted lanelets discovered further
    // along a path -- never on the very first lanelet -- until this block exists).
    //
    // Gated identically to the post-hoc reject-gate below (allow_reverse_route_ /
    // isBidirectionalDrivingLanelet / !hasDeferredRegulatoryElementForReverse) so we don't spend a
    // getRoute() call on a start orientation that would just be rejected afterward anyway, and so
    // this seeding step stays policy-consistent with the gate that already governs mid-path
    // inversions.
    {
      const bool dbg_allow_reverse_route = allow_reverse_route_;
      const bool dbg_is_bidir_start = isBidirectionalDrivingLanelet(st_llt);
      const bool dbg_not_deferred_start = !hasDeferredRegulatoryElementForReverse(st_llt);
      RCLCPP_WARN(
        logger_,
        "[BIDIR-DEBUG] reverse-start gate: st_llt=%ld allow_reverse_route_=%s "
        "isBidirectionalDrivingLanelet=%s !hasDeferredRegulatoryElementForReverse=%s",
        st_llt.id(), (dbg_allow_reverse_route ? "true" : "false"),
        (dbg_is_bidir_start ? "true" : "false"), (dbg_not_deferred_start ? "true" : "false"));
      // [BIDIR-DEBUG] dump the FULL attribute map seen at runtime for st_llt, to settle
      // definitively whether "bidirectional_driving" is present/absent/mismatched.
      std::stringstream dbg_attrs_ss;
      for (const auto & kv : st_llt.attributes()) {
        dbg_attrs_ss << kv.first << "='" << kv.second.value() << "' ";
      }
      RCLCPP_WARN(
        logger_, "[BIDIR-DEBUG] full attribute dump for st_llt=%ld: %s", st_llt.id(),
        dbg_attrs_ss.str().c_str());
    }
    if (
      allow_reverse_route_ && isBidirectionalDrivingLanelet(st_llt) &&
      !hasDeferredRegulatoryElementForReverse(st_llt)) {
      const lanelet::ConstLanelet inv_llt = st_llt.invert();

      // Eligibility for a reverse-start candidate is "is ego's actual heading consistent with
      // being positioned/aligned in this physical lanelet at all" -- which is exactly what
      // `is_proper_angle` (computed above against `st_llt`'s FORWARD tangent) already answers,
      // and this block is unreachable unless `is_proper_angle` is true (see the `continue` a few
      // lines above that skips this entire loop body otherwise). Reversing out of a lanelet does
      // not rotate the vehicle 180 degrees -- ego's nose still points the same physical direction
      // it always did, it simply drives backward along the lanelet in reverse gear. So the correct
      // check here is the SAME forward-tangent alignment already established, not a fresh
      // alignment check against the inverted lanelet's tangent.
      //
      // A prior version of this code computed a separate `inv_angle_diff` by comparing pose_yaw
      // against `get_lanelet_angle(inv_llt, ...)` -- the INVERTED centerline's tangent, which is
      // ~180 degrees opposite the forward tangent by construction (verified by reading
      // lanelet2_core's ConstLanelet::centerline3d(): for an inverted view, `.centerline()`
      // already returns the point-order-reversed linestring). That is mathematically guaranteed to
      // fail the same yaw_threshold (90 deg) test whenever the forward `is_proper_angle` passed
      // (proof: if |angle_diff| <= 90 deg, then |normalize(angle_diff +/- 180 deg)| >= 90 deg,
      // with equality only at the boundary) -- so on real hardware, where ego is normally driving
      // forward with its heading aligned to the lane, the reverse-start candidate was *always*
      // silently discarded here, regardless of whether the routing graph itself supported the
      // reverse route. Confirmed empirically against the real production map
      // (/home/ubuntu/sim_ws/maps/map1/lanelet2_map.osm, lanelet 1970 -> 1945): forward tangent
      // -179.1 deg, inverted tangent -14.3 deg, diff 164.9 deg -- while
      // getRoute(1970.invert(), 1945.invert()) itself succeeds with a 1-hop, length2d=45.4 path
      // (versus the 88.4-length 5-hop forward loop), proving the routing graph was never the
      // problem.
      const auto inv_optional_route = routing_graph_ptr_->getRoute(inv_llt, goal_lanelet, 0);
      RCLCPP_WARN(
        logger_,
        "[BIDIR-DEBUG] reverse-start candidate (inv_llt -> goal_lanelet): st_llt=%ld found=%s "
        // NOTE: lanelet::routing::Route has no public cost() accessor -- length2d() is the only
        // scalar route-cost proxy exposed by the lanelet2_routing API, so cost is approximated
        // downstream via `length2d + angle_diff_weight * angle_diff` (see optional_route_cost /
        // inv_route_cost just below) rather than logged directly here.
        "length2d=%.2f",
        st_llt.id(), (inv_optional_route ? "true" : "false"),
        (inv_optional_route ? inv_optional_route->length2d() : -1.0));
      if (inv_optional_route && is_proper_angle) {
        is_route_found = true;

        // Deliberately NOT wired into the getClosestPreferredLaneletWithinRoute() early-break
        // fast path above (the `if (st_llt.id() == preferred_lane.id())` block). That fast path
        // exists purely to keep re-routes on the same lanelet as a previous route for continuity,
        // and it matches on lanelet id only -- which is direction-agnostic, so `inv_llt` would
        // trivially satisfy it any time the forward `st_llt` already did. But the forward
        // candidate is evaluated first in this same loop iteration and would already have taken
        // that early break in that case, so this branch is only ever reached when the forward
        // candidate did NOT take the fast path (either it didn't match the preferred lanelet, or
        // no preferred lanelet / no route was found for it). Falling through to plain cost
        // comparison here is the correct choice: it lets a genuinely-better reverse-start win on
        // merit without silently reusing a continuity shortcut that was designed for forward-only
        // rerouting semantics, and without ever letting a reverse-start pre-empt a valid
        // forward continuity match.
        const double inv_route_length = inv_optional_route->length2d();
        const double inv_route_cost =
          reverse_route_length_penalty_factor * inv_route_length + angle_diff_weight * angle_diff;
        RCLCPP_DEBUG(
          logger_,
          "Lanelet ID %ld (inverted / reverse-start): Route length = %.1f, Angle Diff = %.4f "
          "rad, Route cost = %.2f",
          st_llt.id(), inv_route_length, angle_diff, inv_route_cost);
        if (inv_route_cost < min_route_cost) {
          min_route_cost = inv_route_cost;
          shortest_path = inv_optional_route->shortestPath();
          start_lanelet = inv_llt;
        }
      }

      // IMPORTANT, discovered by empirical trace (not in the original task spec -- see report):
      // `routing_graph_ptr_->getRoute(from, to, ...)` requires an *exact* vertex match on `to` as
      // well as `from` -- lanelet2 treats a forward lanelet and its inverted counterpart as two
      // distinct graph vertices (confirmed by reading ConstLanelet::operator==, which compares
      // both constData() AND inverted()). `goal_lanelet` above is always the forward-oriented
      // ConstLanelet returned by `getClosestLanelet()` (map storage only ever holds the forward
      // primitive). For the direct "reverse straight out of the current lanelet into the
      // immediately-preceding lanelet" case -- i.e. exactly the failure this task exists to fix --
      // ego arrives at the goal traveling in the *inverted* orientation of the goal lanelet, not
      // the forward one. A minimal manual trace (ring of 4 lanelets A-B-C-D-A, all one_way=no +
      // bidirectional_driving=yes, ego mid-B, goal in A) confirms this concretely:
      //   getRoute(B.invert(), A)          -> NOT FOUND (0 results)
      //   getRoute(B.invert(), A.invert()) -> FOUND: [B.invert(), A.invert()]  (the direct 1-hop
      //                                        reverse route the user actually wants)
      // So `getRoute(st_llt.invert(), goal_lanelet, 0)` alone (the literal candidate described in
      // this task's instructions) is NOT sufficient to fix the reported bug in the simple/direct
      // case -- it would only ever succeed if the search happens to loop back around to a forward-
      // oriented vertex, which is generally impossible for a direct one-lanelet reversal and is not
      // what the user's failure requires. Adding this second candidate -- same inverted start,
      // *also* inverted goal -- is necessary for this fix to actually do anything for the reported
      // scenario. It is safe to add: it goes through the exact same cost-comparison logic as every
      // other candidate in this loop, and the post-hoc reject-gate below already independently
      // validates any inverted lanelet appearing anywhere in the winning path -- including the
      // final (goal) lanelet -- via isBidirectionalDrivingLanelet()/hasDeferredRegulatoryElement-
      // ForReverse(), so no separate eligibility pre-check on goal_lanelet is required for
      // correctness (only skipped here as a minor optimization, mirroring the start-side gate,
      // to avoid a doomed getRoute() call).
      {
        const bool dbg_is_bidir_goal = isBidirectionalDrivingLanelet(goal_lanelet);
        const bool dbg_not_deferred_goal = !hasDeferredRegulatoryElementForReverse(goal_lanelet);
        RCLCPP_WARN(
          logger_,
          "[BIDIR-DEBUG] reverse-goal gate: st_llt=%ld goal_lanelet=%ld "
          "isBidirectionalDrivingLanelet(goal)=%s !hasDeferredRegulatoryElementForReverse(goal)=%s",
          st_llt.id(), goal_lanelet.id(), (dbg_is_bidir_goal ? "true" : "false"),
          (dbg_not_deferred_goal ? "true" : "false"));
      }
      if (
        isBidirectionalDrivingLanelet(goal_lanelet) &&
        !hasDeferredRegulatoryElementForReverse(goal_lanelet)) {
        const lanelet::ConstLanelet inv_goal_llt = goal_lanelet.invert();
        const auto inv_goal_optional_route = routing_graph_ptr_->getRoute(inv_llt, inv_goal_llt, 0);
        RCLCPP_WARN(
          logger_,
          "[BIDIR-DEBUG] reverse-start+reverse-goal candidate (inv_llt -> inv_goal_llt): "
          "st_llt=%ld found=%s length2d=%.2f",
          st_llt.id(), (inv_goal_optional_route ? "true" : "false"),
          (inv_goal_optional_route ? inv_goal_optional_route->length2d() : -1.0));
        if (inv_goal_optional_route && is_proper_angle) {
          is_route_found = true;
          const double inv_goal_route_length = inv_goal_optional_route->length2d();
          const double inv_goal_route_cost =
            reverse_route_length_penalty_factor * inv_goal_route_length +
            angle_diff_weight * angle_diff;
          RCLCPP_DEBUG(
            logger_,
            "Lanelet ID %ld (inverted start + inverted goal, direct reverse): Route length = "
            "%.1f, Angle Diff = %.4f rad, Route cost = %.2f",
            st_llt.id(), inv_goal_route_length, angle_diff, inv_goal_route_cost);
          if (inv_goal_route_cost < min_route_cost) {
            min_route_cost = inv_goal_route_cost;
            shortest_path = inv_goal_optional_route->shortestPath();
            start_lanelet = inv_llt;
          }
        }
      }
    }
  }

  if (is_route_found) {
    lanelet::routing::LaneletPath path;
    path = [&]() -> lanelet::routing::LaneletPath {
      if (consider_no_drivable_lanes && hasNoDrivableLaneInPath(shortest_path)) {
        const auto drivable_lane_path = findDrivableLanePath(start_lanelet, goal_lanelet);
        if (drivable_lane_path) return *drivable_lane_path;
      }
      return shortest_path;
    }();

    path_lanelets->reserve(path.size());
    for (const auto & llt : path) {
      path_lanelets->push_back(llt);
    }

    // Bidirectional-driving policy gate (see bidirectional_plan/04-routing-foundation.md §4b).
    // lanelet2's own RoutingGraph::build() already adds edges for both orientations of any
    // one_way=no lanelet (verified in autoware_lanelet2_extension's Phase-0 spike), so the
    // Dijkstra search above may have already returned a path traversing an inverted lanelet
    // purely because lanelet2 allows it -- independent of Autoware's own policy. Reject (rather
    // than silently accept) any such path unless:
    //   1) allow_reverse_route_ is explicitly enabled, AND
    //   2) every inverted lanelet in the path additionally carries bidirectional_driving=yes
    //      (the narrower Autoware-only policy tag), AND
    //   3) no inverted lanelet in the path carries a regulatory element type this migration
    //      explicitly defers (traffic_light, intersection, roundabout, crosswalk, blind_spot,
    //      no_stopping_area, virtual_traffic_light, occlusion_spot, speed_bump).
    // This is a post-hoc check on the winning path rather than a routing-cost-graph rebuild per
    // call: simpler, and acceptable given allow_reverse_route_ defaults to false and this is a
    // new opt-in capability -- the tradeoff is that a valid, entirely-forward alternate route
    // could theoretically be rejected in favor of "no route found" if the (rejected) shortest
    // path happens to dip into a disallowed inverted lanelet. Flagged for follow-up if this
    // proves too conservative in practice.
    for (const auto & llt : *path_lanelets) {
      if (!llt.inverted()) {
        continue;
      }
      if (!allow_reverse_route_) {
        RCLCPP_WARN(
          logger_,
          "planPathLaneletsBetweenCheckpoints: rejecting path -- traverses inverted lanelet %ld "
          "but allow_reverse_route is false",
          llt.id());
        path_lanelets->clear();
        return false;
      }
      if (!isBidirectionalDrivingLanelet(llt)) {
        RCLCPP_WARN(
          logger_,
          "planPathLaneletsBetweenCheckpoints: rejecting path -- inverted lanelet %ld is not "
          "tagged bidirectional_driving=yes",
          llt.id());
        path_lanelets->clear();
        return false;
      }
      if (hasDeferredRegulatoryElementForReverse(llt)) {
        RCLCPP_WARN(
          logger_,
          "planPathLaneletsBetweenCheckpoints: rejecting path -- inverted lanelet %ld carries a "
          "regulatory element type deferred for reverse routing",
          llt.id());
        path_lanelets->clear();
        return false;
      }
    }
  } else {
    RCLCPP_ERROR_STREAM(
      logger_, "Failed to find a proper route!"
                 << std::endl
                 << " - start checkpoint: " << toString(start_checkpoint) << std::endl
                 << " - goal checkpoint: " << toString(goal_checkpoint) << std::endl
                 << " - start lane ids: " << convertLaneletsIdToString(start_lanelets) << std::endl
                 << " - goal lane id: " << goal_lanelet.id() << std::endl);
  }

  {
    std::stringstream dbg_path_ss;
    for (const auto & llt : *path_lanelets) {
      dbg_path_ss << llt.id() << (llt.inverted() ? "(inv)" : "(fwd)") << " ";
    }
    RCLCPP_WARN(
      logger_, "[BIDIR-DEBUG] final path_lanelets (is_route_found=%s): [ %s]",
      (is_route_found ? "true" : "false"), dbg_path_ss.str().c_str());
  }

  return is_route_found;
}

std::vector<LaneletSegment> RouteHandler::createMapSegments(
  const lanelet::ConstLanelets & path_lanelets) const
{
  const auto main_path = getMainLanelets(path_lanelets);

  std::vector<LaneletSegment> route_sections;

  if (main_path.empty()) {
    return route_sections;
  }

  route_sections.reserve(main_path.size());
  for (const auto & main_llt : main_path) {
    LaneletSegment route_section_msg;
    const lanelet::ConstLanelets route_section_lanelets = getNeighborsWithinRoute(main_llt);
    route_section_msg.preferred_primitive.id = main_llt.id();
    // Bidirectional-driving support: this is the narrowest, most load-bearing spot in the
    // conversion from RouteHandler's lanelet::ConstLanelets sequence to LaneletSegment[] -- the
    // .inverted() bit on main_llt (sourced from route_lanelets_, already direction-corrected by
    // setRouteLanelets()) must be read off here before it is otherwise lost once this message is
    // serialized. is_reversed is a segment-level field (not per LaneletPrimitive): a reversed
    // segment is a single-lane reverse corridor and does not offer lane-change alternatives.
    route_section_msg.is_reversed = main_llt.inverted();
    route_section_msg.primitives.reserve(route_section_lanelets.size());
    for (const auto & section_llt : route_section_lanelets) {
      LaneletPrimitive p;
      p.id = section_llt.id();
      p.primitive_type = "lane";
      route_section_msg.primitives.push_back(p);
    }
    route_sections.push_back(route_section_msg);
  }
  return route_sections;
}

lanelet::ConstLanelets RouteHandler::getMainLanelets(
  const lanelet::ConstLanelets & path_lanelets) const
{
  auto lanelet_sequence = getLaneletSequence(path_lanelets.back());

  RCLCPP_INFO_STREAM(logger_, "getMainLanelets: lanelet_sequence = " << lanelet_sequence);

  lanelet::ConstLanelets main_lanelets;
  while (!lanelet_sequence.empty()) {
    main_lanelets.insert(main_lanelets.begin(), lanelet_sequence.begin(), lanelet_sequence.end());
    lanelet_sequence = getPreviousLaneletSequence(lanelet_sequence);
  }
  return main_lanelets;
}

bool RouteHandler::isNoDrivableLane(const lanelet::ConstLanelet & llt)
{
  const std::string no_drivable_lane_attribute = llt.attributeOr("no_drivable_lane", "no");
  return no_drivable_lane_attribute == "yes";
}

bool RouteHandler::hasNoDrivableLaneInPath(const lanelet::routing::LaneletPath & path) const
{
  for (const auto & llt : path)
    if (isNoDrivableLane(llt)) return true;
  return false;
}

std::optional<lanelet::routing::LaneletPath> RouteHandler::findDrivableLanePath(
  const lanelet::ConstLanelet & start_lanelet, const lanelet::ConstLanelet & goal_lanelet) const
{
  // we create a new routing graph with infinite cost on no drivable lanes
  const auto drivable_routing_graph_ptr = lanelet::routing::RoutingGraph::build(
    *lanelet_map_ptr_, *traffic_rules_ptr_,
    lanelet::routing::RoutingCostPtrs{std::make_shared<RoutingCostDrivable>()});
  const auto route = drivable_routing_graph_ptr->getRoute(start_lanelet, goal_lanelet, 0);
  if (route) return route->shortestPath();
  return {};
}

Pose RouteHandler::get_pose_from_2d_arc_length(
  const lanelet::ConstLanelets & lanelet_sequence, const double s) const
{
  double accumulated_distance2d = 0;
  for (const auto & llt : lanelet_sequence) {
    const auto & centerline = llt.centerline();
    for (auto it = centerline.begin(); std::next(it) != centerline.end(); ++it) {
      const auto pt = *it;
      const auto next_pt = *std::next(it);
      const double distance2d = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));
      if (accumulated_distance2d + distance2d > s) {
        const double ratio = (s - accumulated_distance2d) / distance2d;
        const auto interpolated_pt = pt.basicPoint() * (1 - ratio) + next_pt.basicPoint() * ratio;
        const auto yaw = std::atan2(next_pt.y() - pt.y(), next_pt.x() - pt.x());
        Pose pose;
        pose.position = create_point(interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z());
        pose.orientation = create_quaternion_from_yaw(yaw);
        return pose;
      }
      accumulated_distance2d += distance2d;
    }
  }
  return Pose{};
}
}  // namespace autoware::route_handler
