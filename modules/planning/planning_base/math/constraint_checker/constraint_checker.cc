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

#include <algorithm>
#include <cmath>

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
  const double kMaxSidePassSpeed = 5.0;  // m/s
  const bool is_side_pass_scenario = IsSidePassScenario(trajectory);
  const double max_nominal_speed =
      std::min(FLAGS_max_driving_speed, FLAGS_planning_upper_speed_limit);
  const double max_allowed_speed =
      is_side_pass_scenario ? kMaxSidePassSpeed : max_nominal_speed;

  for (const auto& p : trajectory) {
    const double t = p.relative_time();
    if (t > kMaxCheckRelativeTime) {
      break;
    }

    const double lon_v = p.v();
    if (!WithinRange(lon_v, FLAGS_speed_lower_bound, max_allowed_speed)) {
      if (is_side_pass_scenario) {
        AERROR << "[COMPETITION_VIOLATION] SIDE_PASS_SPEED_VIOLATION: "
               << "velocity " << lon_v << " m/s exceeds side-pass limit "
               << kMaxSidePassSpeed << " m/s at t=" << t;
        return Result::SIDE_PASS_SPEED_VIOLATION;
      }
      if (lon_v > max_allowed_speed) {
        AERROR << "[COMPETITION_VIOLATION] SPEED_VIOLATION: velocity " << lon_v
               << " m/s exceeds limit " << max_allowed_speed << " m/s at t="
               << t;
        return Result::SPEED_VIOLATION;
      }
      return Result::LON_VELOCITY_OUT_OF_BOUND;
    }

    const double lon_a = p.a();
    if (!WithinRange(lon_a, -FLAGS_max_long_dec, FLAGS_max_long_acc)) {
      if (lon_a > FLAGS_max_long_acc) {
        AERROR << "[COMPETITION_VIOLATION] MAX_LONGITUDINAL_ACCELERATION: "
               << lon_a << " m/s^2 exceeds " << FLAGS_max_long_acc
               << " m/s^2 at t=" << t;
      } else {
        AERROR << "[COMPETITION_VIOLATION] MIN_LONGITUDINAL_ACCELERATION: "
               << lon_a << " m/s^2 exceeds -" << FLAGS_max_long_dec
               << " m/s^2 at t=" << t;
      }
      return Result::ACCELERATION_VIOLATION;
    }

    const double kappa = p.path_point().kappa();
    if (!WithinRange(kappa, -FLAGS_kappa_bound, FLAGS_kappa_bound)) {
      ADEBUG << "Kappa out of bound at t=" << t << ", value=" << kappa;
      return Result::CURVATURE_OUT_OF_BOUND;
    }
  }

  for (size_t i = 1; i < trajectory.NumOfPoints(); ++i) {
    const auto& p0 = trajectory.TrajectoryPointAt(static_cast<uint32_t>(i - 1));
    const auto& p1 = trajectory.TrajectoryPointAt(static_cast<uint32_t>(i));
    if (p1.relative_time() > kMaxCheckRelativeTime) {
      break;
    }

    const double dt = p1.relative_time() - p0.relative_time();
    if (dt <= 1.0e-6) {
      continue;
    }
    const double t = p0.relative_time();

    const double lon_jerk = (p1.a() - p0.a()) / dt;
    if (!WithinRange(lon_jerk, FLAGS_longitudinal_jerk_lower_bound,
                     FLAGS_longitudinal_jerk_upper_bound)) {
      ADEBUG << "Longitudinal jerk out of bound at t=" << t
             << ", value=" << lon_jerk;
      return Result::LON_JERK_OUT_OF_BOUND;
    }

    const double lat_a = p1.v() * p1.v() * p1.path_point().kappa();
    if (!WithinRange(lat_a, -FLAGS_max_lateral_acc, FLAGS_max_lateral_acc)) {
      AERROR << "[COMPETITION_VIOLATION] MAX_LATERAL_ACCELERATION: " << lat_a
             << " m/s^2 exceeds +/-" << FLAGS_max_lateral_acc
             << " m/s^2 at t=" << t;
      return Result::LAT_ACCELERATION_OUT_OF_BOUND;
    }
  }

  return Result::VALID;
}

bool ConstraintChecker::IsSidePassScenario(
    const DiscretizedTrajectory& trajectory) {
  if (trajectory.NumOfPoints() < 2) {
    return false;
  }

  const auto& ref_point = trajectory.TrajectoryPointAt(0).path_point();
  const double ref_x = ref_point.x();
  const double ref_y = ref_point.y();
  const double ref_theta = ref_point.theta();
  const double sin_ref = std::sin(ref_theta);
  const double cos_ref = std::cos(ref_theta);

  double total_lateral_offset = 0.0;
  double max_lateral_offset = 0.0;
  double avg_speed = 0.0;

  for (const auto& point : trajectory) {
    const double dx = point.path_point().x() - ref_x;
    const double dy = point.path_point().y() - ref_y;
    const double lateral_offset = std::abs(-sin_ref * dx + cos_ref * dy);
    total_lateral_offset += lateral_offset;
    max_lateral_offset = std::max(max_lateral_offset, lateral_offset);
    avg_speed += point.v();
  }

  const double point_count = static_cast<double>(trajectory.NumOfPoints());
  const double avg_lateral_offset = total_lateral_offset / point_count;
  avg_speed /= point_count;

  const double kMinAvgLateralOffset = 0.5;   // m
  const double kMinMaxLateralOffset = 1.0;   // m
  const double kMaxAvgSpeedForSidePass = 8.0;  // m/s

  const bool has_lateral_movement =
      avg_lateral_offset > kMinAvgLateralOffset ||
      max_lateral_offset > kMinMaxLateralOffset;
  const bool is_low_speed = avg_speed < kMaxAvgSpeedForSidePass;
  return has_lateral_movement && is_low_speed;
}

}  // namespace planning
}  // namespace apollo
