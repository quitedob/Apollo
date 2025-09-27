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
 * @file constraint_checker.cc
 **/

#include "modules/planning/planning_base/math/constraint_checker/constraint_checker.h"

#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {
template <typename T>
bool WithinRange(const T v, const T lower, const T upper) {
  return lower <= v && v <= upper;
}
}  // namespace

ConstraintChecker::Result ConstraintChecker::ValidTrajectory(
    const DiscretizedTrajectory& trajectory) {
  const double kMaxCheckRelativeTime = FLAGS_trajectory_time_length;
  // 竞赛要求：侧向绕障时速度上限为5.0 m/s
  const double kMaxSidePassSpeed = 5.0;  // m/s

  for (const auto& p : trajectory) {
    double t = p.relative_time();
    if (t > kMaxCheckRelativeTime) {
      break;
    }
    double lon_v = p.v();

    // 检查是否处于侧向绕障场景（根据规划上下文或其他标志判断）
    // 这里暂时使用简单的速度阈值检测，如果轨迹中存在低速行驶且有横向偏移的情况
    bool is_side_pass_scenario = IsSidePassScenario(trajectory);

    // 如果处于侧向绕障场景，应用更严格的速度限制
    double max_allowed_speed = is_side_pass_scenario ? kMaxSidePassSpeed : FLAGS_max_driving_speed;

    if (!WithinRange(lon_v, FLAGS_speed_lower_bound, max_allowed_speed)) {
      if (is_side_pass_scenario) {
        // 竞赛违规：侧向绕障速度超限
        AERROR << "[COMPETITION_VIOLATION] SIDE_PASS_SPEED_VIOLATION: "
               << "Side pass velocity at relative time " << t
               << " exceeds bound, value: " << lon_v << " m/s, bound ["
               << FLAGS_speed_lower_bound << ", " << kMaxSidePassSpeed
               << "]. Competition rule: side-pass speed <= 5.0 m/s";
        return Result::SIDE_PASS_SPEED_VIOLATION;
      } else {
        // 竞赛违规：速度超限（60 km/h = 16.67 m/s）
        if (lon_v > FLAGS_planning_upper_speed_limit) {
          AERROR << "[COMPETITION_VIOLATION] SPEED_VIOLATION: "
                 << "Velocity at relative time " << t
                 << " exceeds bound, value: " << lon_v << " m/s, bound ["
                 << FLAGS_speed_lower_bound << ", " << FLAGS_planning_upper_speed_limit
                 << "]. Competition rule: max speed <= 16.67 m/s (60 km/h)";
          return Result::SPEED_VIOLATION;
        } else {
          ADEBUG << "Velocity at relative time " << t
                 << " exceeds bound, value: " << lon_v << ", bound ["
                 << FLAGS_speed_lower_bound << ", " << FLAGS_speed_upper_bound
                 << "].";
        }
        return Result::LON_VELOCITY_OUT_OF_BOUND;
      }
    }

    double lon_a = p.a();
    // 使用竞赛专用加速度限制
    if (!WithinRange(lon_a, -FLAGS_max_long_dec, FLAGS_max_long_acc)) {
      // 竞赛违规：加速度超限
      if (lon_a > FLAGS_max_long_acc) {
        AERROR << "[COMPETITION_VIOLATION] MAX_LONGITUDINAL_ACCELERATION: "
               << "Longitudinal acceleration at relative time " << t
               << " exceeds upper bound, value: " << lon_a << " m/s², bound ["
               << -FLAGS_max_long_dec << ", " << FLAGS_max_long_acc
               << "]. Competition rule: acceleration <= " << FLAGS_max_long_acc << " m/s²";
      } else if (lon_a < -FLAGS_max_long_dec) {
        AERROR << "[COMPETITION_VIOLATION] MIN_LONGITUDINAL_ACCELERATION: "
               << "Longitudinal deceleration at relative time " << t
               << " exceeds lower bound, value: " << lon_a << " m/s², bound ["
               << -FLAGS_max_long_dec << ", " << FLAGS_max_long_acc
               << "]. Competition rule: deceleration >= -" << FLAGS_max_long_dec << " m/s²";
      } else {
        ADEBUG << "Longitudinal acceleration at relative time " << t
               << " exceeds bound, value: " << lon_a << ", bound ["
               << -FLAGS_max_long_dec << ", " << FLAGS_max_long_acc << "].";
      }
      return Result::ACCELERATION_VIOLATION;
    }

    double kappa = p.path_point().kappa();
    if (!WithinRange(kappa, -FLAGS_kappa_bound, FLAGS_kappa_bound)) {
      ADEBUG << "Kappa at relative time " << t
             << " exceeds bound, value: " << kappa << ", bound ["
             << -FLAGS_kappa_bound << ", " << FLAGS_kappa_bound << "].";
      return Result::CURVATURE_OUT_OF_BOUND;
    }
  }

  for (size_t i = 1; i < trajectory.NumOfPoints(); ++i) {
    const auto& p0 = trajectory.TrajectoryPointAt(static_cast<uint32_t>(i - 1));
    const auto& p1 = trajectory.TrajectoryPointAt(static_cast<uint32_t>(i));

    if (p1.relative_time() > kMaxCheckRelativeTime) {
      break;
    }

    double t = p0.relative_time();

    double dt = p1.relative_time() - p0.relative_time();
    double d_lon_a = p1.a() - p0.a();
    double lon_jerk = d_lon_a / dt;
    if (!WithinRange(lon_jerk, FLAGS_longitudinal_jerk_lower_bound,
                     FLAGS_longitudinal_jerk_upper_bound)) {
      ADEBUG << "Longitudinal jerk at relative time " << t
             << " exceeds bound, value: " << lon_jerk << ", bound ["
             << FLAGS_longitudinal_jerk_lower_bound << ", "
             << FLAGS_longitudinal_jerk_upper_bound << "].";
      return Result::LON_JERK_OUT_OF_BOUND;
    }

    double lat_a = p1.v() * p1.v() * p1.path_point().kappa();
    // 使用竞赛专用横向加速度限制
    if (!WithinRange(lat_a, -FLAGS_max_lateral_acc, FLAGS_max_lateral_acc)) {
      // 竞赛违规：向心加速度超限
      AERROR << "[COMPETITION_VIOLATION] MAX_LATERAL_ACCELERATION: "
             << "Lateral acceleration at relative time " << t
             << " exceeds bound, value: " << lat_a << " m/s², bound ["
             << -FLAGS_max_lateral_acc << ", " << FLAGS_max_lateral_acc
             << "]. Competition rule: centripetal acceleration <= " << FLAGS_max_lateral_acc << " m/s²";
      return Result::LAT_ACCELERATION_OUT_OF_BOUND;
    }

    // TODO(zhangyajia): this is temporarily disabled
    // due to low quality reference line.
    /**
    double d_lat_a = p1.v() * p1.v() * p1.path_point().kappa() -
                     p0.v() * p0.v() * p0.path_point().kappa();
    double lat_jerk = d_lat_a / dt;
    if (!WithinRange(lat_jerk, -FLAGS_lateral_jerk_bound,
                     FLAGS_lateral_jerk_bound)) {
      ADEBUG << "Lateral jerk at relative time " << t
             << " exceeds bound, value: " << lat_jerk << ", bound ["
             << -FLAGS_lateral_jerk_bound << ", " << FLAGS_lateral_jerk_bound
             << "].";
      return Result::LAT_JERK_OUT_OF_BOUND;
    }
    **/
  }

  return Result::VALID;
}

bool ConstraintChecker::IsSidePassScenario(const DiscretizedTrajectory& trajectory) {
  // 简单检测侧向绕障场景的逻辑：
  // 1. 检查轨迹中是否存在明显的横向偏移
  // 2. 检查是否在低速行驶（可能是为了绕障而减速）

  if (trajectory.NumOfPoints() < 2) {
    return false;
  }

  // 计算轨迹的平均横向偏移
  double total_lateral_offset = 0.0;
  double max_lateral_offset = 0.0;
  double avg_speed = 0.0;

  for (const auto& point : trajectory) {
    double lateral_offset = std::abs(point.path_point().d());  // 使用d()作为横向偏移
    total_lateral_offset += lateral_offset;
    max_lateral_offset = std::max(max_lateral_offset, lateral_offset);
    avg_speed += point.v();
  }

  double avg_lateral_offset = total_lateral_offset / trajectory.NumOfPoints();
  avg_speed /= trajectory.NumOfPoints();

  // 侧向绕障的判定条件：
  // 1. 平均横向偏移大于阈值（表示有明显的侧向移动）
  // 2. 最大横向偏移大于阈值
  // 3. 平均速度相对较低（绕障时通常会减速）
  const double kMinAvgLateralOffset = 0.5;  // 米
  const double kMinMaxLateralOffset = 1.0;  // 米
  const double kMaxAvgSpeedForSidePass = 8.0;  // m/s

  bool has_lateral_movement = avg_lateral_offset > kMinAvgLateralOffset ||
                             max_lateral_offset > kMinMaxLateralOffset;
  bool is_low_speed = avg_speed < kMaxAvgSpeedForSidePass;

  return has_lateral_movement && is_low_speed;
}

}  // namespace planning
}  // namespace apollo
