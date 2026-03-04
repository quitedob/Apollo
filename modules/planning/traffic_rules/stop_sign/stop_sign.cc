/*****************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/traffic_rules/stop_sign/stop_sign.h"

#include <algorithm>
#include <memory>

#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool StopSign::Init(const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  return TrafficRule::LoadConfig<StopSignConfig>(&config_);
}

Status StopSign::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void StopSign::MakeDecisions(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.enabled()) {
    return;
  }

  const auto& stop_sign_status =
      injector_->planning_context()->planning_status().stop_sign();
  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();

  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info->reference_line().map_path().stop_sign_overlaps();
  for (const auto& stop_sign_overlap : stop_sign_overlaps) {
    if (stop_sign_overlap.end_s <= adc_back_edge_s) {
      continue;
    }

    if (stop_sign_overlap.object_id ==
        stop_sign_status.done_stop_sign_overlap_id()) {
      continue;
    }

    ADEBUG << "BuildStopDecision: stop_sign[" << stop_sign_overlap.object_id
           << "] start_s[" << stop_sign_overlap.start_s << "]";
    const std::string virtual_obstacle_id =
        STOP_SIGN_VO_ID_PREFIX + stop_sign_overlap.object_id;
    const std::vector<std::string> wait_for_obstacle_ids(
        stop_sign_status.wait_for_obstacle_id().begin(),
        stop_sign_status.wait_for_obstacle_id().end());
    util::BuildStopDecision(
        virtual_obstacle_id, stop_sign_overlap.start_s, config_.stop_distance(),
        StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids, Getname(),
        frame, reference_line_info);

    const double stop_line_s = stop_sign_overlap.start_s;
    const double planned_stop_distance = config_.stop_distance();
    const double planned_stop_s = stop_line_s - planned_stop_distance;

    double min_req_distance = FLAGS_stop_tolerance_min;
    double max_req_distance = FLAGS_stop_tolerance_max;
    if (min_req_distance > max_req_distance) {
      AERROR << "Invalid stop tolerance config, min [" << min_req_distance
             << "] > max [" << max_req_distance << "], swap applied.";
      std::swap(min_req_distance, max_req_distance);
    }

    if (planned_stop_distance < min_req_distance) {
      const double required_adjustment =
          min_req_distance - planned_stop_distance;
      const double adjusted_stop_s = planned_stop_s - required_adjustment;
      if (adjusted_stop_s >= adc_back_edge_s) {
        util::BuildStopDecision(
            virtual_obstacle_id, stop_sign_overlap.start_s, min_req_distance,
            StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids,
            Getname(), frame, reference_line_info);
        ADEBUG << "Adjusted stop distance for stop_sign["
               << stop_sign_overlap.object_id << "] from "
               << planned_stop_distance << "m to " << min_req_distance << "m.";
      } else {
        AERROR << "[COMPETITION_VIOLATION] STOP_DISTANCE_VIOLATION: "
               << "Cannot adjust stop point for stop_sign["
               << stop_sign_overlap.object_id
               << "] to meet min distance requirement. Planned distance: "
               << planned_stop_distance << "m, required min: "
               << min_req_distance << "m, required range: ["
               << min_req_distance << ", " << max_req_distance << "]m.";
      }
    } else if (planned_stop_distance > max_req_distance) {
      const double required_adjustment =
          planned_stop_distance - max_req_distance;
      const double adjusted_stop_s = planned_stop_s + required_adjustment;
      if (adjusted_stop_s < stop_line_s) {
        util::BuildStopDecision(
            virtual_obstacle_id, stop_sign_overlap.start_s, max_req_distance,
            StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids,
            Getname(), frame, reference_line_info);
        ADEBUG << "Adjusted stop distance for stop_sign["
               << stop_sign_overlap.object_id << "] from "
               << planned_stop_distance << "m to " << max_req_distance << "m.";
      } else {
        AERROR << "[COMPETITION_VIOLATION] STOP_DISTANCE_VIOLATION: "
               << "Cannot adjust stop point for stop_sign["
               << stop_sign_overlap.object_id
               << "] to meet max distance requirement. Planned distance: "
               << planned_stop_distance << "m, required max: "
               << max_req_distance << "m, required range: ["
               << min_req_distance << ", " << max_req_distance << "]m.";
      }
    }
  }
}

}  // namespace planning
}  // namespace apollo
