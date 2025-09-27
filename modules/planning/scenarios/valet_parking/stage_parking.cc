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
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

StageResult StageParking::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  auto scenario_context = GetContextAs<ValetParkingContext>();
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      scenario_context->target_parking_spot_id;

  // 竞赛要求：检查90秒超时
  double current_time = apollo::common::util::Clock::NowInSeconds();
  double elapsed_time = current_time - scenario_context->parking_start_time;
  if (elapsed_time > FLAGS_parking_search_timeout_sec) {
    AERROR << "[COMPETITION_PARKING_TIMEOUT] Parking scenario exceeded time limit: "
           << elapsed_time << "s > " << FLAGS_parking_search_timeout_sec << "s. "
           << "Competition rule: parking must complete within 90 seconds";
    return StageResult(StageStatusType::ERROR);
  }

  // 竞赛要求：检查是否进入禁行区域
  const auto& vehicle_state = frame->vehicle_state();
  apollo::common::PointENU vehicle_pos;
  vehicle_pos.set_x(vehicle_state.x());
  vehicle_pos.set_y(vehicle_state.y());
  vehicle_pos.set_z(vehicle_state.z());

  if (IsPointInNoParkingRegion(vehicle_pos)) {
    AERROR << "[COMPETITION_FORBIDDEN_ZONE] Vehicle entered forbidden zone at ("
           << vehicle_state.x() << ", " << vehicle_state.y() << "). "
           << "Competition rule: entering forbidden zone results in 0 points";
    return StageResult(StageStatusType::ERROR);
  }

  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "StageParking planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  // 竞赛要求：检查停车压线违规（在最终停车位置）
  if (result.GetStageStatus() == StageStatusType::FINISHED) {
    if (CheckParkingLineViolation(frame, scenario_context->target_parking_spot_id)) {
      AERROR << "[COMPETITION_PARKING_LINE_VIOLATION] Parking line violation detected. "
             << "Competition rule: deduct 20 points for parking over the line";
      // 记录违规但允许完成（赛后扣分）
      scenario_context->parking_line_violation = true;
    }
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StageParking::FinishStage() {
  return StageResult(StageStatusType::FINISHED);
}

bool StageParking::CheckParkingLineViolation(Frame* frame, const std::string& parking_spot_id) {
  // 竞赛要求：检查停车是否压线
  // 计算车辆最终位置与停车位边界的距离

  const auto& vehicle_state = frame->vehicle_state();
  apollo::common::PointENU vehicle_pos;
  vehicle_pos.set_x(vehicle_state.x());
  vehicle_pos.set_y(vehicle_state.y());
  vehicle_pos.set_z(vehicle_state.z());

  // 获取停车位边界
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    ADEBUG << "HDMap not available for parking line check";
    return false;
  }

  hdmap::Id id;
  id.set_id(parking_spot_id);
  ParkingSpaceInfoConstPtr parking_spot = hdmap->GetParkingSpaceById(id);
  if (parking_spot == nullptr) {
    ADEBUG << "Parking spot " << parking_spot_id << " not found in HDMap";
    return false;
  }

  // 计算车辆到停车位边界的距离
  const auto& polygon = parking_spot->polygon();
  double min_distance = std::numeric_limits<double>::infinity();

  // 简化检查：计算车辆中心点到停车位各边的距离
  for (size_t i = 0; i < polygon.points().size(); ++i) {
    const auto& p1 = polygon.points()[i];
    const auto& p2 = polygon.points()[(i + 1) % polygon.points().size()];

    // 计算点到线段的距离（简化实现）
    double dist = DistanceToLineSegment(vehicle_pos.x(), vehicle_pos.y(),
                                      p1.x(), p1.y(), p2.x(), p2.y());
    min_distance = std::min(min_distance, dist);
  }

  // 如果距离小于容忍值，认为压线
  bool violation = min_distance < FLAGS_parking_line_tolerance_m;
  if (violation) {
    AERROR << "Parking line violation: distance to boundary " << min_distance
           << "m < tolerance " << FLAGS_parking_line_tolerance_m << "m";
  }

  return violation;
}

bool StageParking::IsPointInNoParkingRegion(const apollo::common::PointENU& point) {
  // 检查点是否在禁行区域内
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  if (hdmap == nullptr) {
    return false;
  }

  // 查询HDMap中的禁行区域
  // 简化实现：检查点是否在任何NoParking区域内
  auto no_parking_areas = hdmap->GetNoParkingAreas();
  for (const auto& area : no_parking_areas) {
    if (area->polygon().IsPointIn({point.x(), point.y()})) {
      return true;
    }
  }

  return false;
}

// 辅助函数：计算点到线段的距离
double DistanceToLineSegment(double px, double py, double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  if (dx == 0 && dy == 0) {
    // 点重合
    return std::hypot(px - x1, py - y1);
  }

  double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
  t = std::max(0.0, std::min(1.0, t));

  double closest_x = x1 + t * dx;
  double closest_y = y1 + t * dy;

  return std::hypot(px - closest_x, py - closest_y);
}

}  // namespace planning
}  // namespace apollo
