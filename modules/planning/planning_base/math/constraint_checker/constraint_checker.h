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
 * @file constraint_checker.h
 **/

#pragma once

#include "modules/planning/planning_base/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

class ConstraintChecker {
 public:
  enum class Result {
    VALID,
    LON_VELOCITY_OUT_OF_BOUND,
    LON_ACCELERATION_OUT_OF_BOUND,
    LON_JERK_OUT_OF_BOUND,
    LAT_VELOCITY_OUT_OF_BOUND,
    LAT_ACCELERATION_OUT_OF_BOUND,
    LAT_JERK_OUT_OF_BOUND,
    CURVATURE_OUT_OF_BOUND,
    // 竞赛规则特定的违规类型
    STOP_DISTANCE_VIOLATION,
    SIDE_PASS_LATERAL_VIOLATION,
    SIDE_PASS_SPEED_VIOLATION,
    SPEED_VIOLATION,
    ACCELERATION_VIOLATION,
  };
  ConstraintChecker() = delete;
  static Result ValidTrajectory(const DiscretizedTrajectory& trajectory);
  static bool IsSidePassScenario(const DiscretizedTrajectory& trajectory);
};

}  // namespace planning
}  // namespace apollo
