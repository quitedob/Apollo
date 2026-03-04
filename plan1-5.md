# Plan 1-5 汇总（最终执行版）

## 统一规则真值表
1. 通用动力学：
   - 速度 `<= 16.67 m/s`
   - 纵向加速度 `[-6.0, +3.0] m/s^2`
   - 横向加速度 `|a_lat| <= 2.0 m/s^2`
2. StopSign 停距：`2.0 ~ 2.5 m`
3. TrafficLight 停距：`1.5 ~ 2.0 m`
4. 施工区：按 zone `mode` 执行
   - `forbidden`: 进入即 0 分
   - `slowdown`: 超速按帧扣分
5. 泊车：90s 超时失败 + 压线检测

## 已修复的问题映射
1. 阈值冲突：通过“StopSign 与 TrafficLight 分离规则”消除。
2. 红灯参数映射错误：改为 TrafficLight 模块 `config_.stop_distance()` + 区间 clamp。
3. 90s 时间域错误：从 `PlanOnReferenceLine` 移出，放场景层与评测层。
4. 施工区逻辑矛盾：改为 `mode` 模型，去除死分支。
5. 错误路径：统一为仓库真实路径。
6. `abs` 停距误判：移除全局错误判定；评测脚本使用 `signed_dist`。
7. 文档占位残留：重写为可执行版本。

## 代码落地文件
1. `modules/planning/planning_base/gflags/planning_gflags.h`
2. `modules/planning/planning_base/gflags/planning_gflags.cc`
3. `modules/planning/planning_base/math/constraint_checker/constraint_checker.cc`
4. `modules/planning/planners/lattice/lattice_planner.cc`
5. `modules/planning/traffic_rules/stop_sign/stop_sign.cc`
6. `modules/planning/traffic_rules/stop_sign/conf/default_conf.pb.txt`
7. `modules/planning/traffic_rules/traffic_light/traffic_light.cc`
8. `modules/planning/traffic_rules/traffic_light/conf/default_conf.pb.txt`
9. `modules/planning/scenarios/valet_parking/*`
10. `scripts/evaluator/construction_zone_scoring.py`
11. `tools/competition_evaluator.py`

## 本地检查（不编译）
1. 路径与配置检查：`rg -n "traffic_light_stop_distance_min|traffic_light_stop_distance_max|stop_tolerance_min|stop_tolerance_max" modules/planning/planning_base/gflags`
2. 逻辑检查：`rg -n "fabs\(stop_s - ego_stop_s\)|plan_start_time|SCENARIO_TIMEOUT" modules/planning/planners/lattice/lattice_planner.cc`
3. 评分脚本检查：`python scripts/evaluator/construction_zone_scoring.py --help`
4. 通用评测脚本检查：`python tools/competition_evaluator.py --help`
