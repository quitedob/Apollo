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

#include <memory>

#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool StopSign::Init(const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
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

    // build stop decision
    ADEBUG << "BuildStopDecision: stop_sign[" << stop_sign_overlap.object_id
           << "] start_s[" << stop_sign_overlap.start_s << "]";
    const std::string virtual_obstacle_id =
        STOP_SIGN_VO_ID_PREFIX + stop_sign_overlap.object_id;
    const std::vector<std::string> wait_for_obstacle_ids(
        stop_sign_status.wait_for_obstacle_id().begin(),
        stop_sign_status.wait_for_obstacle_id().end());
    // 构建停车决策（使用配置的停车距离）
    util::BuildStopDecision(
        virtual_obstacle_id, stop_sign_overlap.start_s, config_.stop_distance(),
        StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids, Getname(),
        frame, reference_line_info);

    // 竞赛要求：停车距离必须在[1.5, 2.0]米范围内
    // 计算实际停车距离（停止线位置减去计划停车位置）
    const double stop_line_s = stop_sign_overlap.start_s;
    const double planned_stop_distance = config_.stop_distance();
    const double planned_stop_s = stop_line_s - planned_stop_distance;

    // 竞赛规则：停车距离必须在1.5-2.0m范围内
    const double min_req_distance = 1.5;  // 最小停车距离 (m)
    const double max_req_distance = 2.0;  // 最大停车距离 (m)

    // 如果停车距离太小（停车点离停止线太近）
    if (planned_stop_distance < min_req_distance) {
      const double required_adjustment = min_req_distance - planned_stop_distance;
      const double adjusted_stop_s = planned_stop_s - required_adjustment;

      // 检查调整后的停车点是否在车辆后端边界内
      if (adjusted_stop_s >= adc_back_edge_s) {
        // 重新构建停车决策，使用调整后的距离
        const double adjusted_stop_distance = min_req_distance;
        util::BuildStopDecision(
            virtual_obstacle_id, stop_sign_overlap.start_s, adjusted_stop_distance,
            StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids, Getname(),
            frame, reference_line_info);
        ADEBUG << "Adjusted stop distance for stop_sign[" << stop_sign_overlap.object_id
               << "] from " << planned_stop_distance << "m to " << adjusted_stop_distance
               << "m to meet minimum requirement";
      } else {
        // 无法调整，触发重新规划或记录违规
        AERROR << "[COMPETITION_VIOLATION] STOP_DISTANCE_VIOLATION: "
               << "Cannot adjust stop point for stop_sign[" << stop_sign_overlap.object_id
               << "] to meet min distance requirement. Planned distance: "
               << planned_stop_distance << "m, required min: " << min_req_distance << "m. "
               << "Competition rule: stop distance must be 2.0-2.5m before stop line";
        // 这里可以设置需要重新规划的标志或记录竞赛违规
        // reference_line_info->SetReplanFlag(ReplanReason::STOP_SIGN_DISTANCE_VIOLATION);
      }
    }
    // 如果停车距离太大（停车点离停止线太远）
    else if (planned_stop_distance > max_req_distance) {
      const double required_adjustment = planned_stop_distance - max_req_distance;
      const double adjusted_stop_s = planned_stop_s + required_adjustment;

      // 检查调整后的停车点是否仍在停止线前（不能越过停止线）
      if (adjusted_stop_s < stop_line_s) {
        // 重新构建停车决策，使用调整后的距离
        const double adjusted_stop_distance = max_req_distance;
        util::BuildStopDecision(
            virtual_obstacle_id, stop_sign_overlap.start_s, adjusted_stop_distance,
            StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids, Getname(),
            frame, reference_line_info);
        ADEBUG << "Adjusted stop distance for stop_sign[" << stop_sign_overlap.object_id
               << "] from " << planned_stop_distance << "m to " << adjusted_stop_distance
               << "m to meet maximum requirement";
      } else {
        // 无法调整，触发重新规划或记录违规
        AERROR << "[COMPETITION_VIOLATION] STOP_DISTANCE_VIOLATION: "
               << "Cannot adjust stop point for stop_sign[" << stop_sign_overlap.object_id
               << "] to meet max distance requirement. Planned distance: "
               << planned_stop_distance << "m, required max: " << max_req_distance << "m. "
               << "Competition rule: stop distance must be 2.0-2.5m before stop line";
        // 这里可以设置需要重新规划的标志或记录竞赛违规
        // reference_line_info->SetReplanFlag(ReplanReason::STOP_SIGN_DISTANCE_VIOLATION);
      }
    }
  }
}

}  // namespace planning
}  // namespace apollo
