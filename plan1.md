# Plan 1（最终版）- 基础动力学约束 + StopSign 停距

## 目标
将通用动力学限制与 StopSign 停距规则落地，避免全局规则冲突。

## 生效规则
1. 最大速度：`<= 16.67 m/s`
2. 纵向加速度：`[-6.0, +3.0] m/s^2`
3. 横向加速度：`|a_lat| <= 2.0 m/s^2`
4. StopSign 停距：`2.0m ~ 2.5m`

## 代码落点（真实路径）
1. `modules/planning/planning_base/gflags/planning_gflags.h`
2. `modules/planning/planning_base/gflags/planning_gflags.cc`
3. `modules/planning/planning_base/math/constraint_checker/constraint_checker.cc`
4. `modules/planning/traffic_rules/stop_sign/conf/default_conf.pb.txt`
5. `modules/planning/traffic_rules/stop_sign/stop_sign.cc`

## 关键实现说明
1. `constraint_checker.cc` 使用竞赛参数进行速度/纵向/横向约束检查。
2. StopSign 使用 `stop_distance: 2.25` 作为默认值，并在 `stop_sign.cc` 中按 `FLAGS_stop_tolerance_min/max` 强制区间检查。
3. StopSign 不使用红灯停距参数，避免规则串扰。

## 检查项（不编译）
1. `rg -n "max_driving_speed|max_long_acc|max_long_dec|max_lateral_acc" modules/planning/planning_base/gflags`
2. `rg -n "stop_tolerance_min|stop_tolerance_max" modules/planning/traffic_rules/stop_sign`
3. `rg -n "stop_distance: 2.25" modules/planning/traffic_rules/stop_sign/conf/default_conf.pb.txt`
