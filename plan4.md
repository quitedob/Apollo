# Plan 4（最终版）- 施工区规则（无矛盾版本）

## 目标
消除“进入施工区直接0分”和“施工区超速按帧扣分”互相冲突问题，使用统一可配置模式。

## 统一规则模型
每个施工区单独定义 `mode`：
1. `forbidden`：进入即 0 分。
2. `slowdown`：允许通过，但超速按帧扣分。

## 代码落点（真实路径）
1. `modules/planning/planners/lattice/behavior/collision_checker.h`
2. `modules/planning/planners/lattice/behavior/collision_checker.cc`
3. `modules/planning/planners/lattice/lattice_planner.cc`
4. `scripts/evaluator/construction_zone_scoring.py`

## 关键实现说明
1. Planner 侧：
   - 识别 `CONSTRUCTION_` 前缀虚拟障碍并参与碰撞/区域判定。
   - 在候选轨迹筛选时执行施工区进入与限速检查。
2. 评测侧：
   - 新版 `construction_zone_scoring.py` 不再有死分支。
   - 若命中 forbidden 区域：立刻返回 0 分。
   - slowdown 区域按 `floor(excess_speed) * penalty_rate` 累计扣分。

## 检查项（不编译）
1. `rg -n "IsTrajectoryEnteringConstructionZone|IsPointInConstructionZone" modules/planning/planners/lattice`
2. `rg -n "default-zone-mode|forbidden|slowdown" scripts/evaluator/construction_zone_scoring.py`
3. `rg -n "return 0|final_score" scripts/evaluator/construction_zone_scoring.py`
