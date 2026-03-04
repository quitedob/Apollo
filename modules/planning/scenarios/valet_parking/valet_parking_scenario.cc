/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"

#include <cmath>
#include <limits>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"
#include "modules/planning/scenarios/valet_parking/stage_parking.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

namespace {

Vec2d GetParkingSpotCenter(const ParkingSpaceInfoConstPtr& parking_spot) {
  Vec2d center_point(0.0, 0.0);
  const auto& points = parking_spot->polygon().points();
  for (const auto& point : points) {
    center_point += point;
  }
  if (!points.empty()) {
    center_point /= static_cast<double>(points.size());
  }
  return center_point;
}

}  // namespace

bool ValetParkingScenario::Init(std::shared_ptr<DependencyInjector> injector,
                                const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioValetParkingConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  init_ = true;
  return true;
}

bool ValetParkingScenario::IsTransferable(const Scenario* const other_scenario,
                                          const Frame& frame) {
  if (!frame.local_view().planning_command->has_parking_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }

  const auto& nearby_path =
      frame.reference_line_info().front().reference_line().map_path();
  const auto& vehicle_state = frame.vehicle_state();

  std::string target_parking_spot_id;
  bool has_specific_spot = false;
  if (frame.local_view().planning_command->has_parking_command() &&
      frame.local_view()
          .planning_command->parking_command()
          .has_parking_spot_id()) {
    target_parking_spot_id = frame.local_view()
                                 .planning_command->parking_command()
                                 .parking_spot_id();
    has_specific_spot = !target_parking_spot_id.empty();
  }

  if (!has_specific_spot) {
    const double search_radius_m =
        context_.scenario_config.has_parking_search_radius()
            ? context_.scenario_config.parking_search_radius()
            : FLAGS_parking_search_radius_m;
    std::string chosen_spot;
    if (!SelectNearestParkingSpotNearEntrance(
            nearby_path, vehicle_state, FLAGS_parking_entrance_id,
            search_radius_m, &chosen_spot)) {
      AERROR << "[COMPETITION_PARKING] Failed to find available parking spot.";
      return false;
    }
    target_parking_spot_id = chosen_spot;
    AINFO << "[COMPETITION_PARKING] Auto-selected parking spot: "
          << target_parking_spot_id;
  }

  if (target_parking_spot_id.empty()) {
    ADEBUG << "No parking space id available";
    return false;
  }

  PathOverlap parking_space_overlap;
  if (!SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot_id,
                                     &parking_space_overlap)) {
    ADEBUG << "No such parking spot found after searching all path forward "
              "possible"
           << target_parking_spot_id;
    return false;
  }
  double parking_spot_range_to_start =
      context_.scenario_config.parking_spot_range_to_start();
  if (!CheckDistanceToParkingSpot(frame, vehicle_state, nearby_path,
                                  parking_spot_range_to_start,
                                  parking_space_overlap)) {
    ADEBUG << "target parking spot found, but too far, distance larger than "
              "pre-defined distance"
           << target_parking_spot_id;
    return false;
  }

  context_.parking_start_time = Clock::NowInSeconds();
  context_.target_parking_spot_id = target_parking_spot_id;
  return true;
}

bool ValetParkingScenario::SearchTargetParkingSpotOnPath(
    const Path& nearby_path, const std::string& target_parking_id,
    PathOverlap* parking_space_overlap) {
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
  for (const auto& parking_overlap : parking_space_overlaps) {
    if (parking_overlap.object_id == target_parking_id) {
      *parking_space_overlap = parking_overlap;
      return true;
    }
  }
  return false;
}

bool ValetParkingScenario::CheckDistanceToParkingSpot(
    const Frame& frame, const VehicleState& vehicle_state,
    const Path& nearby_path, const double parking_start_range,
    const PathOverlap& parking_space_overlap) {
  // Parking overlap s may be inconsistent on some maps and is not used here.
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  double center_point_s, center_point_l;
  id.set_id(parking_space_overlap.object_id);
  ParkingSpaceInfoConstPtr target_parking_spot_ptr =
      hdmap->GetParkingSpaceById(id);
  if (target_parking_spot_ptr == nullptr) {
    return false;
  }
  Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
  Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
  Vec2d right_top_point = target_parking_spot_ptr->polygon().points().at(2);
  Vec2d left_top_point = target_parking_spot_ptr->polygon().points().at(3);
  Vec2d center_point = (left_bottom_point + right_bottom_point +
                        right_top_point + left_top_point) /
                       4.0;
  nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l);
  double vehicle_point_s = 0.0;
  double vehicle_point_l = 0.0;
  Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
  if (std::abs(center_point_s - vehicle_point_s) < parking_start_range) {
    return true;
  }
  return false;
}

bool ValetParkingScenario::SelectNearestParkingSpotNearEntrance(
    const Path& nearby_path, const VehicleState& vehicle_state,
    const std::string& entrance_id, double search_radius_m,
    std::string* out_parking_spot_id) const {
  CHECK_NOTNULL(out_parking_spot_id);
  out_parking_spot_id->clear();

  if (hdmap_ == nullptr) {
    return false;
  }

  if (!entrance_id.empty()) {
    ADEBUG << "[COMPETITION_PARKING] parking_entrance_id=[" << entrance_id
           << "] is set, but entrance geometry is unavailable in this map API;"
              " fallback to nearest spot by current vehicle position.";
  }

  const Vec2d vehicle_position(vehicle_state.x(), vehicle_state.y());
  double best_dist = std::numeric_limits<double>::infinity();
  for (const auto& parking_overlap : nearby_path.parking_space_overlaps()) {
    hdmap::Id id;
    id.set_id(parking_overlap.object_id);
    const auto parking_spot = hdmap_->GetParkingSpaceById(id);
    if (parking_spot == nullptr) {
      continue;
    }

    const Vec2d center_point = GetParkingSpotCenter(parking_spot);
    const double dist = center_point.DistanceTo(vehicle_position);
    if (search_radius_m > 0.0 && dist > search_radius_m) {
      continue;
    }
    if (dist < best_dist) {
      best_dist = dist;
      *out_parking_spot_id = parking_overlap.object_id;
    }
  }

  if (out_parking_spot_id->empty()) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
