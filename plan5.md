# Plan 5（最终版）- 自主泊车完整落地

## 目标
移除占位/未落地描述，保留可执行实现：自动选最近车位、超时控制、压线检测。

## 已完成实现
1. 无指定车位时自动选最近可用车位（基于当前车辆位置与 map parking overlaps）。
2. 泊车场景累计超时检查（默认 90s，可由场景配置覆盖）。
3. 完成停车后压线检测（车辆 box 与车位 polygon 关系判定）。

## 代码落点（真实路径）
1. `modules/planning/scenarios/valet_parking/valet_parking_scenario.h`
2. `modules/planning/scenarios/valet_parking/valet_parking_scenario.cc`
3. `modules/planning/scenarios/valet_parking/stage_parking.h`
4. `modules/planning/scenarios/valet_parking/stage_parking.cc`
5. `modules/planning/scenarios/valet_parking/proto/valet_parking.proto`
6. `modules/planning/scenarios/valet_parking/conf/scenario_conf.pb.txt`

## 参数
1. `parking_search_radius`（默认 40m）
2. `parking_search_timeout_sec`（默认 90s）
3. `parking_line_tolerance`（默认 0.1m）

## 检查项（不编译）
1. `rg -n "SelectNearestParkingSpotNearEntrance|parking_start_time" modules/planning/scenarios/valet_parking`
2. `rg -n "parking_search_timeout_sec|parking_line_tolerance" modules/planning/scenarios/valet_parking`
3. `rg -n "占位|未实现|placeholder" modules/planning/scenarios/valet_parking/valet_parking_scenario.cc modules/planning/scenarios/valet_parking/stage_parking.cc`
