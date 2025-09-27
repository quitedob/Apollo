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

#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"
#include "modules/planning/scenarios/valet_parking/stage_parking.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

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
  // 竞赛要求：自主泊车场景判断逻辑
  if (!frame.local_view().planning_command->has_parking_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }

  std::string target_parking_spot_id;
  bool has_specific_spot = false;

  // 检查是否有指定的停车位ID
  if (frame.local_view().planning_command->has_parking_command() &&
      frame.local_view()
          .planning_command->parking_command()
          .has_parking_spot_id()) {
    target_parking_spot_id = frame.local_view()
                                 .planning_command->parking_command()
                                 .parking_spot_id();
    has_specific_spot = !target_parking_spot_id.empty();
  }

  // 如果没有指定停车位，则根据竞赛要求选择距离入口最近的可用停车位
  if (!has_specific_spot) {
    AINFO << "[COMPETITION_PARKING] No specific parking spot provided, "
          << "selecting nearest available spot to entrance";

    std::string chosen_spot;
    if (SelectNearestParkingSpotNearEntrance(FLAGS_parking_entrance_id,
                                           FLAGS_parking_search_radius_m,
                                           &chosen_spot)) {
      target_parking_spot_id = chosen_spot;
      AINFO << "[COMPETITION_PARKING] Selected nearest parking spot: "
            << target_parking_spot_id;
    } else {
      AERROR << "[COMPETITION_PARKING] Failed to find suitable parking spot near entrance";
      return false;
    }
  }

  if (target_parking_spot_id.empty()) {
    ADEBUG << "No parking space id available";
    return false;
  }

  const auto& nearby_path =
      frame.reference_line_info().front().reference_line().map_path();
  PathOverlap parking_space_overlap;
  const auto& vehicle_state = frame.vehicle_state();

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

  // 记录场景开始时间用于超时检查
  context_.parking_start_time = apollo::common::util::Clock::NowInSeconds();

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
  // TODO(Jinyun) parking overlap s are wrong on map, not usable
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  double center_point_s, center_point_l;
  id.set_id(parking_space_overlap.object_id);
  ParkingSpaceInfoConstPtr target_parking_spot_ptr =
      hdmap->GetParkingSpaceById(id);
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
    const std::string& entrance_id, double search_radius_m,
    std::string* out_parking_spot_id) {
  // 竞赛要求：选择距离泊车场入口最近的可用车位

  // 1) 获取入口坐标
  apollo::common::PointENU entrance_pt;
  if (!GetEntrancePointById(entrance_id, &entrance_pt)) {
    AERROR << "SelectNearestParkingSpot: failed to get entrance point for id "
           << entrance_id;
    return false;
  }

  // 2) 从地图获取候选停车位
  std::vector<apollo::hdmap::ParkingSpace> candidates;
  if (!hdmap::HDMapUtil::GetParkingSpacesWithinRadius(entrance_pt, search_radius_m, &candidates)) {
    AERROR << "SelectNearestParkingSpot: HDMap query failed or empty";
    return false;
  }

  // 3) 遍历候选车位，过滤不可用/禁区/太近障碍等
  double best_dist = std::numeric_limits<double>::infinity();
  std::string best_spot_id;

  for (const auto& ps : candidates) {
    // 获取停车位ID和中心点
    const std::string spot_id = ps.id().id();
    apollo::common::PointENU center = ps.center();

    // 过滤：如果车位在禁行区
    if (IsPointInNoParkingRegion(center)) {
      ADEBUG << "spot " << spot_id << " in NoParking region, skip";
      continue;
    }

    // 过滤：如果感知信息表明已占用（这里简化处理，实际应查询感知模块）
    if (IsParkingSpotOccupied(spot_id)) {
      ADEBUG << "spot " << spot_id << " occupied, skip";
      continue;
    }

    // 计算入口到车位中心距离并选最小
    double dx = center.x() - entrance_pt.x();
    double dy = center.y() - entrance_pt.y();
    double d = std::hypot(dx, dy);
    if (d < best_dist) {
      best_dist = d;
      best_spot_id = spot_id;
    }
  }

  if (best_spot_id.empty()) {
    ADEBUG << "SelectNearestParkingSpot: no suitable spot found";
    return false;
  }

  // 4) 返回选中的车位ID
  *out_parking_spot_id = best_spot_id;
  AINFO << "SelectNearestParkingSpot selected spot " << best_spot_id
        << " dist_m=" << best_dist;
  return true;
}

bool ValetParkingScenario::GetEntrancePointById(const std::string& entrance_id,
                                               apollo::common::PointENU* entrance_point) {
  // 简化实现：根据入口ID获取入口坐标
  // 实际实现应从地图数据或配置中查询
  // 这里作为示例，假设入口坐标已知或从routing结果中获取

  if (entrance_id.empty()) {
    AERROR << "Entrance ID is empty";
    return false;
  }

  // 示例：从routing或地图配置中获取入口点
  // 实际实现需要根据具体地图格式调整
  ADEBUG << "Getting entrance point for ID: " << entrance_id;

  // 临时实现：使用默认坐标（实际应从地图查询）
  // TODO: 实现从HDMap查询入口点的逻辑
  entrance_point->set_x(0.0);  // 示例坐标
  entrance_point->set_y(0.0);  // 示例坐标
  entrance_point->set_z(0.0);

  return true;
}

bool ValetParkingScenario::IsParkingSpotOccupied(const std::string& spot_id) {
  // 简化实现：检查停车位是否被占用
  // 实际应查询感知模块的占用信息

  // 示例：这里返回false表示所有车位都可用
  // 实际实现应查询perception模块的结果
  ADEBUG << "Checking occupancy for spot " << spot_id << ": available";
  return false;
}

bool ValetParkingScenario::IsPointInNoParkingRegion(const apollo::common::PointENU& point) {
  // 检查点是否在禁行区域内
  // 实际应查询HDMap的NoParking区域

  if (hdmap_ == nullptr) {
    return false;
  }

  // 示例实现：查询HDMap中的禁行区域
  // 实际需要根据HDMap API实现精确查询
  ADEBUG << "Checking NoParking region for point (" << point.x() << ", " << point.y() << ")";

  // 临时实现：简单边界检查（实际应查询地图禁行区域）
  // TODO: 实现完整的HDMap NoParking区域查询
  return false;
}

}  // namespace planning
}  // namespace apollo
