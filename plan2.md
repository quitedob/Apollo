# Plan 2（最终版）- TrafficLight 停距 + 红灯右转

## 目标
修正红灯停距参数映射错误，确保红灯规则由 TrafficLight 模块真实生效。

## 生效规则
1. 红灯停车距离：`1.5m ~ 2.0m`
2. 红灯右转：按场景配置开关控制

## 正确参数归属
1. 不使用 `min_stop_distance_obstacle/max_stop_distance_obstacle` 控制红灯停距。
2. 红灯停距由 `traffic_light.cc` 的 `config_.stop_distance()` 生效。
3. 红灯停距区间由 gflags 约束：
   - `traffic_light_stop_distance_min=1.5`
   - `traffic_light_stop_distance_max=2.0`

## 代码落点（真实路径）
1. `modules/planning/planning_base/gflags/planning_gflags.h`
2. `modules/planning/planning_base/gflags/planning_gflags.cc`
3. `modules/planning/traffic_rules/traffic_light/traffic_light.cc`
4. `modules/planning/traffic_rules/traffic_light/conf/default_conf.pb.txt`
5. `modules/planning/scenarios/traffic_light_unprotected_right_turn/conf/scenario_conf.pb.txt`

## 关键实现说明
1. `traffic_light.cc` 在构建 stop decision 前对 `config_.stop_distance()` 做 clamp 到 `[min,max]`。
2. `default_conf.pb.txt` 维持中值 `1.75`，由区间参数兜底。
3. 红灯右转开关路径固定为：
   - `modules/planning/scenarios/traffic_light_unprotected_right_turn/conf/scenario_conf.pb.txt`
   - `enable_right_turn_on_red: true`

## 检查项（不编译）
1. `rg -n "traffic_light_stop_distance_min|traffic_light_stop_distance_max" modules/planning/planning_base/gflags`
2. `rg -n "effective_stop_distance|std::clamp" modules/planning/traffic_rules/traffic_light/traffic_light.cc`
3. `rg -n "enable_right_turn_on_red" modules/planning/scenarios/traffic_light_unprotected_right_turn/conf/scenario_conf.pb.txt`
