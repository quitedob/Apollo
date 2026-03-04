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

#include "modules/planning/scenarios/valet_parking/stage_parking.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;

namespace {

double DistanceToLineSegment(double px, double py, double x1, double y1,
                             double x2, double y2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  if (dx == 0.0 && dy == 0.0) {
    return std::hypot(px - x1, py - y1);
  }

  double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
  t = std::max(0.0, std::min(1.0, t));

  const double closest_x = x1 + t * dx;
  const double closest_y = y1 + t * dy;
  return std::hypot(px - closest_x, py - closest_y);
}

}  // namespace

StageResult StageParking::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  (void)planning_init_point;
  auto scenario_context = GetContextAs<ValetParkingContext>();
  CHECK_NOTNULL(scenario_context);

  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      scenario_context->target_parking_spot_id;

  double timeout_sec = FLAGS_parking_search_timeout_sec;
  if (scenario_context->scenario_config.has_parking_search_timeout_sec()) {
    timeout_sec = scenario_context->scenario_config.parking_search_timeout_sec();
  }
  if (timeout_sec <= 0.0) {
    timeout_sec = FLAGS_scenario_time_limit_sec;
  }
  if (timeout_sec <= 0.0) {
    timeout_sec = 90.0;
  }

  if (scenario_context->parking_start_time > 0.0) {
    const double elapsed_time =
        Clock::NowInSeconds() - scenario_context->parking_start_time;
    if (elapsed_time > timeout_sec) {
      AERROR << "[COMPETITION_PARKING_TIMEOUT] Parking scenario exceeded time "
                "limit: "
             << elapsed_time << "s > " << timeout_sec << "s.";
      return StageResult(StageStatusType::ERROR);
    }
  }

  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "StageParking planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  if (result.GetStageStatus() == StageStatusType::FINISHED) {
    if (CheckParkingLineViolation(frame, scenario_context->target_parking_spot_id)) {
      AERROR << "[COMPETITION_PARKING_LINE_VIOLATION] Parking line violation "
                "detected.";
      scenario_context->parking_line_violation = true;
    }
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StageParking::FinishStage() { return FinishScenario(); }

bool StageParking::CheckParkingLineViolation(Frame* frame,
                                             const std::string& parking_spot_id) {
  if (parking_spot_id.empty()) {
    return false;
  }

  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    return false;
  }

  hdmap::Id id;
  id.set_id(parking_spot_id);
  const auto parking_spot = hdmap->GetParkingSpaceById(id);
  if (parking_spot == nullptr) {
    return false;
  }

  const auto scenario_context = GetContextAs<ValetParkingContext>();
  double tolerance = FLAGS_parking_line_tolerance_m;
  if (scenario_context != nullptr &&
      scenario_context->scenario_config.has_parking_line_tolerance()) {
    tolerance = scenario_context->scenario_config.parking_line_tolerance();
  }

  const auto& vehicle_state = frame->vehicle_state();
  const auto& vehicle_param =
      VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  Vec2d center_point(vehicle_state.x(), vehicle_state.y());
  const double shift_distance =
      vehicle_param.front_edge_to_center() - vehicle_param.length() / 2.0;
  center_point +=
      Vec2d::CreateUnitVec2d(vehicle_state.heading()) * shift_distance;

  Box2d vehicle_box(center_point, vehicle_state.heading(), vehicle_param.length(),
                    vehicle_param.width());
  std::vector<Vec2d> corners;
  vehicle_box.GetAllCorners(&corners);

  const auto& parking_polygon = parking_spot->polygon();
  const auto& points = parking_polygon.points();
  if (points.size() < 2) {
    return false;
  }

  for (const auto& corner : corners) {
    if (parking_polygon.IsPointIn(corner)) {
      continue;
    }

    double min_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < points.size(); ++i) {
      const auto& p1 = points[i];
      const auto& p2 = points[(i + 1) % points.size()];
      min_distance =
          std::min(min_distance, DistanceToLineSegment(corner.x(), corner.y(),
                                                       p1.x(), p1.y(), p2.x(),
                                                       p2.y()));
    }
    if (min_distance > tolerance) {
      return true;
    }
  }

  return false;
}

}  // namespace planning
}  // namespace apollo
