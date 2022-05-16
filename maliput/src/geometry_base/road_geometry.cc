#include "maliput/geometry_base/road_geometry.h"

#include <utility>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/segment.h"

namespace maliput {
namespace geometry_base {


api::RoadPositionResult RoadGeometry::DoToRoadPosition(const api::InertialPosition& inertial_position,
                                                       const std::optional<api::RoadPosition>& hint) const {
  MALIPUT_THROW_UNLESS(kdtree_ != nullptr);
  if(hint.has_value()) {
    return DoBruteForceToRoadPosition(inertial_position, hint);
  }
  // Use the kdtree implementation for getting the position.
  const auto nearest_point = kdtree_->nearest({inertial_position.x(), inertial_position.y(), inertial_position.z()});
  const auto lane_id = points_lane_id_.at(nearest_point);
  const auto lane = this->ById().GetLane(lane_id);
  const auto lane_position_result = lane->ToLanePosition(inertial_position);
  return {
      {lane, lane_position_result.lane_position}, lane_position_result.nearest_position, lane_position_result.distance};
}

api::RoadPositionResult RoadGeometry::DoBruteForceToRoadPosition(const api::InertialPosition& inertial_position,
                                                                 const std::optional<api::RoadPosition>& hint) const {
  MALIPUT_THROW_MESSAGE("Should be implemented by the backend.");
}

void RoadGeometry::SpacialReorganization() {
  // Sample the surface Lane::ToInertialPosition.
  // Load the kdtree with the sampled RoadNetwork.
  const auto lanes = this->ById().GetLanes();

  std::vector<maliput::math::point<double, 3>> points;
  for (const auto& lane : lanes) {
    const auto lane_length = lane.second->length();
    for (double s = 0; s < lane_length; s += 0.1) {
      const auto lane_bounds = lane.second->lane_bounds(s);
      for (double r = lane_bounds.min(); r <= lane_bounds.max(); r += 0.1) {
        const auto inertial_pos = lane.second->ToInertialPosition({s, r, 0. /* h */}).xyz();
        const maliput::math::point<double, 3> point{inertial_pos.x(), inertial_pos.y(), inertial_pos.z()};
        points.push_back(point);
        points_lane_id_.emplace(point, lane.second->id());
      }
    }
  }
  kdtree_ = std::make_unique<maliput::math::kdtree<double, 3>>(points.begin(), points.end());
}

void RoadGeometry::AddJunctionPrivate(std::unique_ptr<Junction> junction) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(junction.get() != nullptr);
  junctions_.emplace_back(std::move(junction));
  Junction* const raw_junction = junctions_.back().get();
  // clang-format off
  raw_junction->AttachToRoadGeometry({}, this, [this](auto segment) { id_index_.AddSegment(segment); },
                                     [this](auto lane) { id_index_.AddLane(lane); });
  // clang-format on
  id_index_.AddJunction(raw_junction);
}

void RoadGeometry::AddBranchPointPrivate(std::unique_ptr<BranchPoint> branch_point) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(branch_point.get() != nullptr);
  branch_points_.emplace_back(std::move(branch_point));
  BranchPoint* const raw_branch_point = branch_points_.back().get();
  raw_branch_point->AttachToRoadGeometry({}, this);
  id_index_.AddBranchPoint(raw_branch_point);
}

const api::Junction* RoadGeometry::do_junction(int index) const { return junctions_.at(index).get(); }

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const { return branch_points_.at(index).get(); }

}  // namespace geometry_base
}  // namespace maliput
