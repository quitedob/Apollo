下面给出**可执行的修改清单 + 必要的代码片段（含中文注释）+ 测试/验证步骤**，目标是让 Apollo（或基于 Apollo 的竞赛系统）满足你贴出来的竞赛规则（纵向加减速、向心加速度、速度上限、红灯停位 1.5–2.0m、红灯允许右转、借道绕障保持横向 ≥1m 且绕行速度 ≤5 m/s，及对应评分触发条件）。我同时引用了你上传的运行/dump 片段和 Apollo 源（用于定位 flag/模块）。引用：   。另外引用 Apollo planning flags 源文件以及讨论 enable_right_turn_on_red 的 issue（用于说明如何打开“红灯右转”场景配置）。([Apollo开发者社区][1])

---

# 一、总体方案（概览）

1. **用 flag/配置把软约束设为比赛要求的硬限制（优先）**：在 `planning_gflags`（或启动时的命令行）强制设置纵横向/速度上下界与停距区间。这样无需改 planner 算法也能生效。参考：`planning_gflags.h` 中已有大量相关 flag（如 `longitudinal_acceleration_upper_bound`、`lateral_acceleration_bound`、`planning_upper_speed_limit`、`min_stop_distance_obstacle` / `max_stop_distance_obstacle` 等）。([Apollo开发者社区][1])

2. **针对侧向借道（side-pass / nudge）追加运行时检查与速度限制**：在“侧通过程”或 `PathDecider/SidePassPlanner` 中强制给出 `max_side_pass_speed = 5.0`，并在构建障碍物盒子时确保横向 buffer ≥ 1.0m（若 planner 自身的 lat buffer < 1.0，则运行时按 1.0 修正）。这可以通过两处配合实现：

   * 配置层（flags）尽量靠前设置 `static_obstacle_nudge_l_buffer`、`nonstatic_obstacle_nudge_l_buffer` >= 1.0。
   * 代码层在 `CollisionChecker` 或构造障碍包围盒处做最小横向扩展保障（见示例修改片段）。

3. **红灯行为**：保证“红灯停位 1.5–2.0m”可通过 `min_stop_distance_obstacle` / `max_stop_distance_obstacle` 设置来约束 stop point。若需要“红灯右转允许”，打开 traffic-light 相关 scenario 配置选项（issue/讨论显示 Apollo 有 `enable_right_turn_on_red` 配置项）。([GitHub][2])

4. **日志/打分/监控**：在 planner 的 safety/constraint check（ConstraintChecker::ValidTrajectory）里把超限情况映射为明确的日志（例如：`STOP_DISTANCE_VIOLATION`、`SIDE_PASS_LATERAL_VIOLATION`、`SPEED_VIOLATION`、`CENTRIPETAL_ACCEL_VIOLATION`）便于赛后扣分与自动化评估。

---

# 二、具体修改（逐条、可直接应用）

## 2.1 在启动命令或默认配置中强制设置 flag（最小入侵）

**做法 A（推荐临时比赛使用）**：直接在启动 planning 时以命令行参数覆盖 flags，例如：

```bash
# 在启动 planning 时添加如下 gflags 参数
--longitudinal_acceleration_upper_bound=3.0 \
--longitudinal_acceleration_lower_bound=-6.0 \
--lateral_acceleration_bound=2.0 \
--planning_upper_speed_limit=16.67 \
--default_city_road_speed_limit=16.67 \
--min_stop_distance_obstacle=1.5 \
--max_stop_distance_obstacle=2.0 \
--static_obstacle_nudge_l_buffer=1.0 \
--nonstatic_obstacle_nudge_l_buffer=1.0 \
--enable_scenario_side_pass_multiple_parked_obstacles=true
```

**说明**：这些 flag 在 `planning_gflags.h` 中已声明（见源码）。设置后 planner 的轨迹合理性检查就会拒绝违背这些上下界的候选轨迹。([Apollo开发者社区][1])

---

## 2.2 若需要把这些值写入源码默认（持久化），修改文件

* 文件：`modules/planning/planning_base/gflags/planning_gflags.cc`
* 操作：把默认值写入对应 flag 的 DEFINE 处（如果该 `.cc` 存在默认设置），或在系统启动脚本中写清单。
  （**注**：我没有在这次回答里把 .cc 全部列出——因为可能版本差异较大。更稳妥做法是：用命令行覆盖 flags。）

---

## 2.3 **侧向绕障（Side-pass）速度与横距硬约束 — 代码级保障（建议修改）**

### 目标

1. 当 planner 判定当前正在执行“side-pass/绕障”动作时，确保生成的轨迹最大纵向车速 ≤ **5.0 m/s**。
2. 当构造障碍物预测包围盒时，确保横向扩展至少为 **1.0 m**（否则强制扩展）。

### 修改位置（建议）

* `modules/planning/planning_base/math/constraint_checker/collision_checker.cc`（或其等价文件；CollisionChecker 在评估时会构造障碍物包围盒并做轨迹与障碍的碰撞检测）
* `modules/planning/planners/.../side_pass_*` 或 `path_decider`：在选定侧通过程时强制将候选纵向速度剪裁为 `min(candidate_speed, 5.0)`。

### 修改示例（只输出被修改的函数及位置说明，便于你 patch；含中文注释）

**说明**：下面示例把构造障碍横向扩展部分做最小值保证；文件路径和函数名请按你当前代码树确认（示例基于常见 Apollo 代码布局）：

```
文件路径：modules/planning/planning_base/math/constraint_checker/collision_checker.cc
修改：在构造障碍物包围盒（BuildPredictedEnvironment / CreateObstacleBoundary）处修改横向扩展逻辑

// --- 被修改的函数片段（只列出修改函数） ---
// 修改位置: collision_checker.cc 中 BuildPredictedEnvironment 或类似函数
// 大体行号：靠近“构建障碍物边界盒”的代码段

// 中文注释：确保横向扩展至少为比赛要求的 1.0m，避免 planner 因较小的 lat buffer 导致横向靠近
double required_min_lateral_gap = 1.0; // 比赛要求的横向最小间距（米）

// 假设原本代码类似： obs_box.LateralExtend(2.0 * FLAGS_lat_collision_buffer);
double original_lat_buffer = FLAGS_lat_collision_buffer; // 读取 flag
double calc_lat_expand = 2.0 * original_lat_buffer;

// 中文注释：计算最终要用的横向扩展量（取两者中较大者，保证 >= required_min_lateral_gap + 车辆半宽）
double vehicle_half_width = vehicle_param.width() / 2.0; // 或从车辆参数读取
double min_needed_expand = required_min_lateral_gap + vehicle_half_width;
double final_lat_expand = std::max(calc_lat_expand, min_needed_expand);

// 中文注释：将最终横向扩展应用到障碍盒
obs_box.LateralExtend(final_lat_expand);
```

> 说明：`vehicle_param.width()` 仅为示例；在你的代码里请使用已有的车辆参数 API（通常在 vehicle_config/vehicle_param 中可读到 `vehicle_param_.vehicle_width()` 等）。如果无法读取宽度，最坏情况把 `vehicle_half_width` 设为 `0.5m` 做 conservative 保证。

---

### 侧通过程速度限制（示例）

**文件**：`modules/planning/planners/<your_planner>/side_pass_planner.cc` 或 `path_decider.cc`
**修改点**：在把 candidate trajectory 输出到 evaluator 前，做一遍速度裁剪：

```cpp
// 中文注释：将侧通过程的轨迹纵向速度上限为比赛要求 5.0 m/s
const double kMaxSidePassSpeed = 5.0; // m/s

for (auto &traj : candidate_trajectories) {
    // 中文注释：遍历轨迹点，若速度超过上限则裁剪并重新计算相关衍生量
    for (auto &p : traj.points()) {
        if (p.v() > kMaxSidePassSpeed) {
            p.set_v(kMaxSidePassSpeed);
            // 如果需要：同时修改加速度/加jerk 为平滑值（略）
        }
    }
    // 重新评估轨迹 cost / 约束（调用原有流程）
}
```

---

## 2.4 **红灯停位 1.5–2.0m**（场景配置 + stop point）

1. 打开/检查交通灯场景配置文件：`modules/planning/conf/scenario/traffic_light_*.pb.txt`，确保 stop 点的 `stop_wall` / `min_stop_distance` 与 `max_stop_distance` 对应到 `min_stop_distance_obstacle` / `max_stop_distance_obstacle`，或在 scenario 的 stage_stop 阶段把 stop distance 强制为 [1.5, 2.0]。
2. 直接设置 flag（简单、可靠）：

```bash
--min_stop_distance_obstacle=1.5 --max_stop_distance_obstacle=2.0
```

3. 若你发现车辆仍然不右转（参见 issue），注意要打开`enable_right_turn_on_red`或`enable_right_turn_on_red`等开关（取决于版本）。有 issue 提示 `enable_right_turn_on_red` 在 `modules/planning/conf/scenario/traffic_light_unprotected_right_turn_config.pb.txt` 使用。([GitHub][2])

---

## 2.5 **向心加速度（centripetal）检查**

* 在 lattice 的轨迹评估器中通常有 `weight_centripetal_acceleration` 和 `CentripetalAccelerationCost`，以及 `ConstraintChecker` 会校验 curvature/kappa 是否超限（`kappa_bound`）和 lateral acceleration bound (`lateral_acceleration_bound`)。把 `lateral_acceleration_bound` 设为 `2.0`（m/s^2）：

```bash
--lateral_acceleration_bound=2.0
```

> 备注：`lateral_acceleration_bound` 在 `planning_gflags.h` 中声明（见源码）。([Apollo开发者社区][1])

---

## 2.6 **监测与 scoring 输出（建议）**

在 `ConstraintChecker::ValidTrajectory()`（或调用该函数的上层）增加**明确的日志/metric**，当违反：

* stop 距离不在 [1.5,2.0] → 输出 `STOP_DISTANCE_VIOLATION`（并写入 monitor topic）
* 侧向横距 < 1.0 → `SIDE_PASS_LATERAL_VIOLATION`
* 侧通过程最大速度 > 5.0 → `SIDE_PASS_SPEED_VIOLATION`
* 速度>16.67 → `SPEED_VIOLATION`
* 加速度超出 → `ACCELERATION_VIOLATION`
  这些都便于对手写的评分脚本或自动评分器统计扣分（frame-by-frame 或每场景最终汇总）。

---

# 三、逐条对应你给出评分规则的实现/验证清单（Checklist）

1. **加速度限制：加 <= 3 m/s²，减 >= -6 m/s²**

   * 设置：`--longitudinal_acceleration_upper_bound=3.0`、`--longitudinal_acceleration_lower_bound=-6.0`。
   * 验证：在 simulator（LGSVL 或 Apollo Dreamview）中进行直线加速/制动测试，检查 `ConstraintChecker::ValidTrajectory()` 不通过时产生 `LON_ACCELERATION_OUT_OF_BOUND` 日志。([Apollo开发者社区][1])

2. **向心加速度 <= 2.0 m/s²**

   * 设置：`--lateral_acceleration_bound=2.0`。
   * 验证：在拐弯场景放置固定速度，查看是否产生 `CENTRIPETAL_ACCEL_VIOLATION` 或 `CURVATURE_OUT_OF_BOUND`。

3. **速度上限 16.67 m/s（60 km/h）**

   * 设置：`--planning_upper_speed_limit=16.67`、`--default_city_road_speed_limit=16.67`。
   * 验证：跑 highway 场景确保 planner 不生成超过该值的轨迹。

4. **红灯停位 1.5–2.0 m**

   * 设置：`--min_stop_distance_obstacle=1.5`、`--max_stop_distance_obstacle=2.0` 并确保 traffic-light scenario 的 stop stage 使用上述 flags。
   * 验证：在 traffic-light 场景中 ego 停车位置至停止线距离在 1.5–2.0m；若不满足，planner 会拒绝轨迹并记录 `STOP_DISTANCE_VIOLATION`。

5. **红灯允许右转**

   * 操作：在对应 scenario 配置文件里打开 `enable_right_turn_on_red`（或等价开关）。参考社区 issue。([GitHub][2])
   * 验证：设置目标到右转车道并在红灯时观察 planner 是否进入 `TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN` 场景并允许右转决策。

6. **借道绕行：横距 ≥1m、绕行速度 ≤5 m/s**

   * 设置 flags: `--static_obstacle_nudge_l_buffer=1.0`、`--nonstatic_obstacle_nudge_l_buffer=1.0` 并在碰撞构造处强制横距最小值（代码补丁示例已给出）。
   * 对绕行速度：代码级限制 `kMaxSidePassSpeed = 5.0` 在 side-pass 输出前裁剪。
   * 验证：构造场景：前方车道有停驶车辆，测试绕行，记录横距与速度曲线，检查是否满足约束。

7. **评分对应的扣分触发（自动化）**

   * 在 planner monitor topic（或 `cyber_monitor`）里输出事件（STOP_DISTANCE_VIOLATION 等），评分器订阅这些 topic，每次触发按规则扣分。你上传的 dumps 显示 system/monitor 可以记录 metrics（见你上传的 dump 片段），便于直接接入评分系统。

---

# 四、测试计划（建议的一组自动化测试用例）

1. **直线加减速场**：给定目标速度 10 m/s，强制全油门再急刹，检查最大正/负加速度是否超出阈值。
2. **曲线速度场**：不同曲率下运行，检查横向加速度不超过 2.0。
3. **红灯场景**：红灯直行、红灯右转（在打开 `enable_right_turn_on_red` 的情况下），检查停位距离是否落在 [1.5,2.0]，并记录是否触发右转。
4. **借道绕障场景**：放置静态车辆阻挡，测试侧通过程，检查横向最小距离≥1.0m 且侧通过程纵向速度 ≤5 m/s。
5. **评分模拟**：对每帧/每场景触发日志，写一个评估脚本把 violations 映射为扣分并打印结果，验证规则一致性。

---

# 五、Patch 建议与优先级（什么时候做什么）

1. **优先级 A（立即可生效）**：通过命令行 flags 覆盖（见 2.1）。优点：不改源码、风险低、比赛快速生效。
2. **优先级 B（中期）**：在 `CollisionChecker` 中加入横向最小扩展保护（见 2.3 的函数片段）并在 side-pass 输出前裁剪速度。优点：更稳健、可避免 flag 被意外改写导致违规。
3. **优先级 C（长期）**：在 scenario config pbtxt 中把 stop 区间与“红灯右转”逻辑做明确字段，加入 metric/monitor 逻辑用于自动扣分。优点：结构清晰、便于后续维护与评测。

---

# 六、示例命令行（比赛当天建议直接用）

```bash
# 示例：启动 planning 时的完整参数（合并）
bazel run //modules/planning:planning -- \
--longitudinal_acceleration_upper_bound=3.0 \
--longitudinal_acceleration_lower_bound=-6.0 \
--lateral_acceleration_bound=2.0 \
--planning_upper_speed_limit=16.67 \
--default_city_road_speed_limit=16.67 \
--min_stop_distance_obstacle=1.5 \
--max_stop_distance_obstacle=2.0 \
--static_obstacle_nudge_l_buffer=1.0 \
--nonstatic_obstacle_nudge_l_buffer=1.0 \
--enable_scenario_side_pass_multiple_parked_obstacles=true
```

---

# 七、额外注意事项与陷阱

1. **版本差异**：不同 Apollo 版本中 flag 名称、场景 conf 的字段名可能略有差异。请优先用 `--help` 或查阅 `planning_gflags.h` 确认（我检索到你版本的 `planning_gflags.h` 中确实存在这些 flag）。([Apollo开发者社区][1])
2. **红灯右转行为的地图/标注依赖**：红灯右转有时依赖 map 中 signal/stop-line 的相对位置与仿真器的 traffic_signal topic。如果右转不被触发，先检查 map 的 signal domain 与车辆目标点位置（issue 讨论指出这点）。([GitHub][2])
3. **竞赛评分与 planner safety**：强制把某些限制设置得过严可能导致 planner 无可行轨迹进而触发 safety (fallback/estop)。建议在比赛现场多跑几遍并观察 planner safety log，适当微调 `lattice_stop_buffer` / `fallback_total_time` 等容错参数。([Apollo开发者社区][1])

---

# 八、我能帮你继续做的事（你可以直接回复下列任意一项）

1. 我可以 **基于你当前代码树**（你上传的 repo/dumps）把上面 `collision_checker` 的具体 patch 以 diff/完整函数形式写出来（并给出准确行号）。为此我会直接打开你上传的相应源码文件并输出修改版本（如果你想，我会把完整文件放在 `modules/.../collision_checker.cc` 中）。
2. 我可以把**比赛当天的一键启动脚本**写好（包含所有 flags 与监控 hook），以及一个评分脚本（订阅 monitor topic，转成扣分日志）。
3. 我可以为你生成 LGSVL / Apollo 的**自动化回归测试场景**（json / scenario），用于验证每一条规则。

如果要我直接给出代码补丁，请回复“给我 patch”，我会基于你当前 repo（我可以访问的文件）直接输出修改后的函数或文件，并严格按照你之前的 code 输出要求（文件路径在开头、每段代码含简洁中文注释、若只改一个函数只给出该函数并标明修改的文件与行区间）。

---

最后再提醒一遍关键引用（便于你核对）：

* `planning_gflags.h`（flags 列表与名词对应）— 源码浏览页面。([Apollo开发者社区][1])
* 社区 issue（关于 `enable_right_turn_on_red` 的讨论）— 说明 traffic-light 右转是有开关的。([GitHub][2])
* 你上传的系统 dump / Apollo-main 片段（可用于定位监控 / metrics）。  

要我现在直接输出 **可打补丁的代码 diff**（把 `collision_checker` 的改动 + `side-pass` 速度裁剪函数）吗？（我会直接把文件路径写在最上方并按你要求添加中文注释与行号提示）

[1]: https://apollo.baidu.com/docs/apollo/9.0/planning__gflags_8h_source.html "Apollo: modules/planning/planning_base/gflags/planning_gflags.h 源文件"
[2]: https://github.com/ApolloAuto/apollo/issues/14842?utm_source=chatgpt.com "Apollo didn't turn right, maybe a bug · Issue #14842"
