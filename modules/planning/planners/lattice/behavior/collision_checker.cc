/******************************************************************************
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

#include "modules/planning/planners/lattice/behavior/collision_checker.h"

#include <utility>

#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::PathMatcher;
using apollo::common::math::Vec2d;

CollisionChecker::CollisionChecker(
    const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
    const double ego_vehicle_d,
    const std::vector<PathPoint>& discretized_reference_line,
    const ReferenceLineInfo* ptr_reference_line_info,
    const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph) {
  ptr_reference_line_info_ = ptr_reference_line_info;
  ptr_path_time_graph_ = ptr_path_time_graph;
  BuildPredictedEnvironment(obstacles, ego_vehicle_s, ego_vehicle_d,
                            discretized_reference_line);
}

bool CollisionChecker::InCollision(
    const std::vector<const Obstacle*>& obstacles,
    const DiscretizedTrajectory& ego_trajectory, const double ego_length,
    const double ego_width, const double ego_back_edge_to_center) {
  for (size_t i = 0; i < ego_trajectory.NumOfPoints(); ++i) {
    const auto& ego_point =
        ego_trajectory.TrajectoryPointAt(static_cast<std::uint32_t>(i));
    const auto relative_time = ego_point.relative_time();
    const auto ego_theta = ego_point.path_point().theta();

    Box2d ego_box({ego_point.path_point().x(), ego_point.path_point().y()},
                  ego_theta, ego_length, ego_width);

    // correct the inconsistency of reference point and center point
    // TODO(all): move the logic before constructing the ego_box
    double shift_distance = ego_length / 2.0 - ego_back_edge_to_center;
    Vec2d shift_vec(shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta));
    ego_box.Shift(shift_vec);

    std::vector<Box2d> obstacle_boxes;
    for (const auto obstacle : obstacles) {
      auto obtacle_point = obstacle->GetPointAtTime(relative_time);
      Box2d obstacle_box = obstacle->GetBoundingBox(obtacle_point);
      if (ego_box.HasOverlap(obstacle_box)) {
        return true;
      }
    }
  }
  return false;
}

bool CollisionChecker::InCollision(
    const DiscretizedTrajectory& discretized_trajectory) {
  CHECK_LE(discretized_trajectory.NumOfPoints(),
           predicted_bounding_rectangles_.size());
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();

  for (size_t i = 0; i < discretized_trajectory.NumOfPoints(); ++i) {
    const auto& trajectory_point =
        discretized_trajectory.TrajectoryPointAt(static_cast<std::uint32_t>(i));
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);
    double shift_distance =
        ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);

    for (const auto& obstacle_box : predicted_bounding_rectangles_[i]) {
      if (ego_box.HasOverlap(obstacle_box)) {
        return true;
      }
    }
  }
  return false;
}

void CollisionChecker::BuildPredictedEnvironment(
    const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
    const double ego_vehicle_d,
    const std::vector<PathPoint>& discretized_reference_line) {
  ACHECK(predicted_bounding_rectangles_.empty());
  ACHECK(predicted_is_construction_mask_.empty());

  // If the ego vehicle is in lane,
  // then, ignore all obstacles from the same lane.
  bool ego_vehicle_in_lane = IsEgoVehicleInLane(ego_vehicle_s, ego_vehicle_d);
  std::vector<const Obstacle*> obstacles_considered;
  for (const Obstacle* obstacle : obstacles) {
    // 施工区虚拟障碍物不跳过：带 CONSTRUCTION_ 前缀的虚拟障碍需要纳入碰撞检测
    if (obstacle->IsVirtual()) {
      const std::string& oid = obstacle->Id();
      // 检查是否为施工区虚拟障碍物
      if (oid.find(FLAGS_construction_obstacle_id_prefix) != 0) {
        // 非施工区的虚拟障碍仍然跳过
        continue;
      }
      // 施工区虚拟障碍会被继续处理
    }

    if (ego_vehicle_in_lane &&
        (IsObstacleBehindEgoVehicle(obstacle, ego_vehicle_s,
                                    discretized_reference_line) ||
         !ptr_path_time_graph_->IsObstacleInGraph(obstacle->Id()))) {
      continue;
    }

    obstacles_considered.push_back(obstacle);
  }

  double relative_time = 0.0;
  while (relative_time < FLAGS_trajectory_time_length) {
    std::vector<Box2d> predicted_env;
    std::vector<bool> construction_mask_current_frame;

    for (const Obstacle* obstacle : obstacles_considered) {
      // If an obstacle has no trajectory, it is considered as static.
      // Obstacle::GetPointAtTime has handled this case.
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);
      box.LongitudinalExtend(2.0 * FLAGS_lon_collision_buffer);

      // 竞赛要求：确保侧向绕障横向间距至少为1.0m
      // 计算当前横向缓冲区
      double original_lat_buffer = FLAGS_lat_collision_buffer;
      double calc_lat_extend = 2.0 * original_lat_buffer;

      // 获取车辆参数以确保最小横向间距
      const auto& vehicle_config =
          common::VehicleConfigHelper::Instance()->GetConfig();
      double vehicle_half_width = vehicle_config.vehicle_param().width() / 2.0;

      // 竞赛要求的横向最小间距（米）
      double required_min_lateral_gap = 1.0;

      // 计算最终需要的横向扩展量：取计算值和最小要求中的较大者
      // 确保横向间距至少为 required_min_lateral_gap + vehicle_half_width
      double min_needed_extend = required_min_lateral_gap + vehicle_half_width;
      double final_lat_extend = std::max(calc_lat_extend, min_needed_extend);

      box.LateralExtend(final_lat_extend);
      predicted_env.push_back(std::move(box));

      // 记录该障碍物是否为施工区
      bool is_construction =
          (obstacle->IsVirtual() &&
           obstacle->Id().find(FLAGS_construction_obstacle_id_prefix) == 0);
      construction_mask_current_frame.push_back(is_construction);
    }
    predicted_bounding_rectangles_.push_back(std::move(predicted_env));
    predicted_is_construction_mask_.push_back(std::move(construction_mask_current_frame));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

bool CollisionChecker::IsEgoVehicleInLane(const double ego_vehicle_s,
                                          const double ego_vehicle_d) {
  double left_width = FLAGS_default_reference_line_width * 0.5;
  double right_width = FLAGS_default_reference_line_width * 0.5;
  ptr_reference_line_info_->reference_line().GetLaneWidth(
      ego_vehicle_s, &left_width, &right_width);
  return ego_vehicle_d < left_width && ego_vehicle_d > -right_width;
}

bool CollisionChecker::IsObstacleBehindEgoVehicle(
    const Obstacle* obstacle, const double ego_vehicle_s,
    const std::vector<PathPoint>& discretized_reference_line) {
  double half_lane_width = FLAGS_default_reference_line_width * 0.5;
  TrajectoryPoint point = obstacle->GetPointAtTime(0.0);
  auto obstacle_reference_line_position = PathMatcher::GetPathFrenetCoordinate(
      discretized_reference_line, point.path_point().x(),
      point.path_point().y());

  if (obstacle_reference_line_position.first < ego_vehicle_s &&
      std::fabs(obstacle_reference_line_position.second) < half_lane_width) {
    ADEBUG << "Ignore obstacle [" << obstacle->Id() << "]";
    return true;
  }
  return false;
}

bool CollisionChecker::IsTrajectoryEnteringConstructionZone(
    const DiscretizedTrajectory& discretized_trajectory) {
  // 遍历轨迹每一点，构造 ego_box 并判断是否与施工区 box 有 overlap
  CHECK_LE(discretized_trajectory.NumOfPoints(),
           predicted_bounding_rectangles_.size());

  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();

  for (size_t i = 0; i < discretized_trajectory.NumOfPoints(); ++i) {
    const auto& trajectory_point =
        discretized_trajectory.TrajectoryPointAt(static_cast<uint32_t>(i));
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);
    double shift_distance =
        ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);

    // 检查是否与施工区障碍物有重叠
    for (size_t j = 0; j < predicted_bounding_rectangles_[i].size(); ++j) {
      if (predicted_is_construction_mask_[i][j]) {
        if (ego_box.HasOverlap(predicted_bounding_rectangles_[i][j])) {
          return true;  // 进入施工区
        }
      }
    }
  }
  return false;
}

bool CollisionChecker::IsPointInConstructionZone(double x, double y,
                                                  double relative_time) {
  // 将相对时间转换为帧索引
  size_t frame_index = static_cast<size_t>(
      std::floor(relative_time / FLAGS_trajectory_time_resolution));

  if (frame_index >= predicted_bounding_rectangles_.size()) {
    return false;
  }

  // 检查点是否在施工区障碍物内
  for (size_t j = 0; j < predicted_bounding_rectangles_[frame_index].size(); ++j) {
    if (predicted_is_construction_mask_[frame_index][j]) {
      if (predicted_bounding_rectangles_[frame_index][j].IsPointIn({x, y})) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
