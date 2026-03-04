# Plan 3（最终版）- Planner 逻辑边界 + 90s 超时时间域

## 目标
修复“把场景累计超时放在单周期规划函数里”的时间域错误，并去掉全局错误停距判定。

## 已修正原则
1. 不在 `PlanOnReferenceLine` 内做场景累计 90s 判定。
2. 不在 `PlanOnReferenceLine` 做全局统一停距验收（避免 StopSign/TrafficLight 规则冲突）。
3. 停距规则由对应 traffic-rule 模块处理：
   - StopSign: `2.0~2.5`
   - TrafficLight: `1.5~2.0`

## 代码落点（真实路径）
1. `modules/planning/planners/lattice/lattice_planner.cc`
2. `modules/planning/scenarios/valet_parking/stage_parking.cc`
3. `tools/competition_evaluator.py`

## 关键实现说明
1. `lattice_planner.cc` 移除：
   - 单周期 `plan_start_time` 超时检查
   - `fabs(stop_s - ego_stop_s)` 的全局停距筛选
2. 场景累计超时在场景层/评测层做：
   - `stage_parking.cc` 使用 `parking_start_time` + timeout
   - `tools/competition_evaluator.py` 支持赛后统一判定
3. 停车线判定必须保留符号：
   - `signed_dist = stop_line_s - ego_stop_s`
   - `signed_dist <= 0` 视为越线

## 检查项（不编译）
1. `rg -n "plan_start_time|SCENARIO_TIMEOUT" modules/planning/planners/lattice/lattice_planner.cc`
2. `rg -n "fabs\(stop_s - ego_stop_s\)" modules/planning/planners/lattice/lattice_planner.cc`
3. `rg -n "signed_dist|STOP_LINE_CROSSED" tools/competition_evaluator.py`
