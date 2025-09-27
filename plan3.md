好的——我把需要修改的点分成 **配置（flags）**、**约束检查（ConstraintChecker / ConstraintChecker::ValidTrajectory）**、**规划器（LatticePlanner）**、**评测/场景管理（scenario timeout & 打分检查）**、以及 **测试/CI** 五部分，给出具体的文件/函数修改建议、代码片段（均含简短中文注释）和最终验证步骤。为了便于你把改动直接落到 Apollo 项目中，我在每处都标明了修改的**文件路径**和**修改位置说明**，并给出编译/测试命令。下面直接开始。

> 先声明：你仓库里已有 `lattice_planner`、`collision_checker` 等实现（见库片段）。我将基于这些文件给出最小侵入、可复现的修改方案。引用项目文件用于对照：  

---

# 一、总体思路（一句话）

在规划阶段强制把速度/加速度/向心加速度限制到竞赛要求（`v ≤ 16.67 m/s`，`long accel ≤ 3 m/s²`，`long dec ≤ 6 m/s²`，`lat accel ≤ 2 m/s²`），并在选定最终轨迹前对“停车到停止线的距离”做验收（必须在 2.0–2.5m），场景超时（>90s）或与障碍碰撞则判定本场景得 0 分或视作 planning 失败。实现路径主要在 gflags（参数可调）、ConstraintChecker（约束判断）、LatticePlanner（流程 & 最终检查）以及评测 harness（time limit & scoring）上。

---

# 二、配置（新增 gflags）

**文件**：`modules/planning/planning_base/gflags/planning_gflags.h` （或项目中已有此头文件）
**修改位置说明**：新增一些竞赛专用 gflags（可在已有 gflags 文件中追加）

```cpp
// 文件路径: modules/planning/planning_base/gflags/planning_gflags.h
// 说明：为竞赛约束新增可调 flag（中文注释）
DECLARE_double(max_driving_speed);            // 最大行驶速度（m/s），竞赛限制 16.67
DECLARE_double(max_long_acc);                 // 最大纵向正加速度（m/s^2），竞赛限制 3.0
DECLARE_double(max_long_dec);                 // 最大纵向制动（正数，m/s^2），竞赛限制 6.0
DECLARE_double(max_lateral_acc);              // 最大横向（向心）加速度（m/s^2），竞赛限制 2.0
DECLARE_double(stop_tolerance_min);           // 停车到停止线最小容忍距离（m）, 竞赛 2.0
DECLARE_double(stop_tolerance_max);           // 停车到停止线最大容忍距离（m）, 竞赛 2.5
DECLARE_int32(scenario_time_limit_sec);       // 每场景时间限制（秒），竞赛 90
```

并在对应的 `.cc`（gflags 注册处）中给默认值，例如：

```cpp
// 文件路径: modules/planning/planning_base/gflags/planning_gflags.cc
DEFINE_double(max_driving_speed, 16.67, "比赛最大允许速度(m/s)");
DEFINE_double(max_long_acc, 3.0, "比赛最大纵向加速度(m/s^2)");
DEFINE_double(max_long_dec, 6.0, "比赛最大纵向减速度(取正值, m/s^2)");
DEFINE_double(max_lateral_acc, 2.0, "比赛最大横向加速度(m/s^2)");
DEFINE_double(stop_tolerance_min, 2.0, "停车到停止线最小距离(m)");
DEFINE_double(stop_tolerance_max, 2.5, "停车到停止线最大距离(m)");
DEFINE_int32(scenario_time_limit_sec, 90, "场景限时(秒)");
```

> 目的：所有约束可通过 flag 调整，便于赛前/赛中切换。

---

# 三、约束校验：增强 `ConstraintChecker::ValidTrajectory`

**文件**：`modules/planning/planning_base/math/constraint_checker/constraint_checker.cc`（或相应实现）
**修改目的**：用新 flag 替换硬编码阈值，并把纵向制动（负加速度）按 `max_long_dec` 判断（注意符号），在横向约束处使用曲率和速度计算向心加速度 `a_lat = v^2 * curvature` 并比较 `max_lateral_acc`。

**修改示例（函数体）**：

```cpp
// 文件路径: modules/planning/planning_base/math/constraint_checker/constraint_checker.cc
// 修改函数: ConstraintChecker::ValidTrajectory
// （下面只展示关键检查段落，实际需要替换原来同类检查逻辑）

// --- 开始修改段 ---
for (const auto& point : combined_trajectory) {
  double v = point.v();            // m/s
  double a = point.a();            // m/s^2  (纵向加速度，正为加速，负为减速)
  double curvature = point.path_point().kappa(); // 曲率 (1/m)

  // 1) 纵向加速度限制（加速）
  if (a > FLAGS_max_long_acc + 1e-6) {
    return Result::LON_ACCELERATION_OUT_OF_BOUND;
  }
  // 2) 纵向制动限制（注意 a 为负表示制动，比较绝对值）
  if (a < -FLAGS_max_long_dec - 1e-6) {
    return Result::LON_ACCELERATION_OUT_OF_BOUND;
  }
  // 3) 横向（向心）加速度： a_lat = v^2 * curvature
  double a_lat = v * v * std::fabs(curvature);
  if (a_lat > FLAGS_max_lateral_acc + 1e-6) {
    return Result::LAT_ACCELERATION_OUT_OF_BOUND;
  }
  // 4) 速度上限（全局）
  if (v > FLAGS_max_driving_speed + 1e-6) {
    return Result::LON_VELOCITY_OUT_OF_BOUND;
  }
}
// --- 结束修改段 ---
```

**说明**：

* 用 `fabs(curvature)` 而不是 signed curvature，因为向心加速度和曲率方向无关。
* 保留原有 `Result` 枚举并使用已存在的错误码（如果不存在，可扩展）。
* 这段要替换原来以硬编码阈值判断的逻辑（项目里 `ConstraintChecker::ValidTrajectory` 已有类似结构，修改时请替换相应段落）。引用原 `lattice_planner` 中调用 `ConstraintChecker::ValidTrajectory` 的地方供对照。

---

# 四、规划器（LatticePlanner）——在选取轨迹前后加审查与罚分/失败判定

**文件**：`modules/planning/planners/lattice/lattice_planner.cc`
**修改位置**：函数 `LatticePlanner::PlanOnReferenceLine(...)` —— 在设置 cruise speed 处并在最终选好 `combined_trajectory` 后做停车距离验收和场景超时检查。

**要点修改**：

1. 强制将 cruise speed 限制为 `min(reference_speed, FLAGS_max_driving_speed)`。

2. 在选中 `combined_trajectory`（即准备 `break;` 之前），如果 `planning_target.has_stop_point()`，计算“停车位置到停止线”的距离差并判定是否在 `[stop_tolerance_min, stop_tolerance_max]`。若不在范围内，**视情形**：

   * 若偏差超过可接受（比如停得太近或越过停止线），可以 `continue;` 尝试下一优解，或者直接把此轨迹标为失败并返回 `Status::OK()` 但设置更高 cost（取决你要的竞技逻辑）。竞赛规则要求“未在停止线前停车或距离不在 2.0~2.5 则扣分或判 0 分”，所以在评测端也要判定是否得分为 0；这里建议在 planner 里**如果找不到满足停车容差的轨迹则强制 fallback 为 backup 或标记为不可行（PLANNING_ERROR）**，以便上层评测判定 0 分。

3. 在一开始（PlanOnReferenceLine 开头）记录 `start_time = Clock::NowInSeconds()`，在生成轨迹循环时判断 `Clock::NowInSeconds() - start_time > FLAGS_scenario_time_limit_sec`，若超过则直接返回错误（或把 reference_line_info 标成不可行），以便上层在评测时直接把本场景判为超时失败。

**示例修改段（插入 / 替换位置）**：

```cpp
// 文件: modules/planning/planners/lattice/lattice_planner.cc
// 在 speed_limit 获取后加入 cap
double speed_limit =
    reference_line_info->reference_line().GetSpeedLimitFromS(init_s[0]);
// 限制最大速度为比赛规则
speed_limit = std::min(speed_limit, FLAGS_max_driving_speed);
reference_line_info->SetLatticeCruiseSpeed(speed_limit);

// 在主循环开始处记录时间（如果尚未记录）
double plan_start_time = Clock::NowInSeconds();

// 在每次选择合并轨迹并准备 break 前，加入停车容差检查
if (planning_target.has_stop_point()) {
  double stop_s = planning_target.stop_point().s();  // 停止线位置 s
  // 取 combined_trajectory 最后一个轨迹点 s（假设最后点是停车点）
  const auto& last_point = combined_trajectory.back();
  double ego_stop_s = last_point.path_point().s();

  double stop_dist = std::fabs(stop_s - ego_stop_s);
  if (stop_dist < FLAGS_stop_tolerance_min - 1e-6 ||
      stop_dist > FLAGS_stop_tolerance_max + 1e-6) {
    // 不满足停车容差：尝试下一个轨迹（continue）
    ADEBUG << "Reject trajectory: stop distance " << stop_dist
           << " not in [" << FLAGS_stop_tolerance_min << ","
           << FLAGS_stop_tolerance_max << "]";
    continue;
  }
}

// 场景超时判断
if (Clock::NowInSeconds() - plan_start_time > FLAGS_scenario_time_limit_sec) {
  AERROR << "Planning exceeded scenario time limit (" << FLAGS_scenario_time_limit_sec << "s)";
  return Status(ErrorCode::PLANNING_ERROR, "Scenario time limit exceeded");
}
```

**说明**：

* 这里对停车容差直接用 `fabs(stop_s - ego_stop_s)`，前提是 `s` 的定义一致（若你的 `stop_point.s()` 已是停车线的 s）。如果不是，请根据你项目中 reference_line 的坐标系调整（但大多数 Apollo impl 使用 Frenet s）。
* 如果你希望把“停车距离不在范围”直接视作本场景得 0 分，可以在上层评测 harness（见下一节）捕获 `PLANNING_ERROR` 并计为 0。

参考 `lattice_planner` 的结构（你项目中已有的实现），请在相应 break 前插入上述检查。

---

# 五、评测/场景管理（测试 harness）——强制 90s、计分规则

竞赛规则里的“90 秒”和“扣分判定”更适合放在评测 harness（比赛 runner 或 Scenario Manager）中实现，而不是纯 planner。建议修改或添加以下内容：

**A. 场景超时**：
在 Scenario Runner（你们的评测脚本或测试节点）里，如果从场景开始到场景完成（车辆成功通过或判失败）的 wall-clock 时间超过 `FLAGS_scenario_time_limit_sec`（或直接硬编码 90），把该场景记为失败（score = 0）。如果你没有独立的 runner，请在测试 framework 中新增一层超时监控。

**B. 停车评分**：
在场景结束时（车辆停止或任务结束），评测器计算“停止点到停止线”的距离 `d`：

* 若 `d` 在 `[2.0, 2.5]` -> 不扣分（或按规则扣 0）
* 若 `d` < 2.0 or > 2.5 but vehicle stopped before stop line -> 扣 20 分（或按你规则）
* 若车辆未在停止线前停车（例如越过停止线）或没避让 -> 得 0 分

把这段判定放在评测器更合适（不要放在 planner 中做最终分数）。Planner 的职责是尽力生成合规轨迹并在找不到满足停车容差的轨迹时失败/降级。

---

# 六、碰撞与“施工区”特殊处理

竞赛场景是“部分车道被占用”（施工），这意味着你在 `CollisionChecker::BuildPredictedEnvironment` 或 PathTimeGraph 中需要：

* 把施工区标记为不可通行区域（virtual obstacle），或将施工区的 obstacle box 加大 buffer（使用 `FLAGS_lon_collision_buffer` / `FLAGS_lat_collision_buffer`）以反映占道情况。`collision_checker` 的 `BuildPredictedEnvironment` 已对每个 obstacle 做 `LongitudinalExtend(2.0 * FLAGS_lon_collision_buffer)`，你可以通过调整 flags 增大 buffer 以更保守地避让。具体文件：`modules/planning/planners/lattice/behavior/collision_checker.cc`。

**示例**：在竞赛模式下，将 buffer 增大：

```bash
# 在 gflags 中设置（运行时或配置文件）
--lon_collision_buffer=1.0  # 具体根据车辆与施工区城相对大小调整
--lat_collision_buffer=0.8
```

---

# 七、单元/集成测试 & CI（必须）

每次改动后务必运行：

```bash
# 全量构建
bazel build //...
# 或 buildtool
buildtool build

# 运行 planning 相关单测（如存在）
bazel test //modules/planning/...
```

同时推荐添加下面三类自动化测试（放到 CI）：

1. **约束单测**：手工构造带有特定速度/加速度/曲率的轨迹，调用 `ConstraintChecker::ValidTrajectory`，验证返回值（超阈值 -> 失败，合规 -> 通过）。
2. **停车容差测试**：模拟 planning_target 有 stop_point 的场景，验证 planner 能挑出满足 `[2.0,2.5]` 的轨迹或在找不到时返回错误。
3. **场景超时测试**：用一个人为延迟的模块或模拟慢速环境，触发 `scenario_time_limit` 并验证评测器收到超时并判 0 分。

---

# 八、建议的最小 patch 清单（便于你直接落地）

1. `modules/planning/planning_base/gflags/planning_gflags.h` 与 `.cc`：新增 flags。
2. `modules/planning/planning_base/math/constraint_checker/constraint_checker.cc`：用 flags 替换硬编码阈值；加入向心加速度计算。
3. `modules/planning/planners/lattice/lattice_planner.cc`：在 `PlanOnReferenceLine` 中 cap speed、停车容差检查、场景超时检查。
4. （可选）评测 harness / scenario runner：加入超时与评分逻辑（90s、停止距离扣分/0分逻辑）。
5. 修改 CI：在 PR gate 加入 `bazel build //...` 与规划相关的 unit tests。

---

# 九、示例验证流程（落地后的操作顺序）

1. 修改代码（按上面清单）。
2. 本地编译：`bazel build //modules/planning/...`。
3. 运行约束单测与 planner 单测（或编写简单运行 demo 做黑盒验证）。
4. 在比赛场景（xh_2025_狭窄道路通行）运行 full stack，检查日志：ADEBUG/AERROR 中有关“Reject trajectory: stop distance ...”或超时日志。
5. 若通过，合并到主分支并在 CI 中跑 full build；在场景评测器中验证得分与规则一致。

---

# 十、额外建议（实现细节与注意事项）

* **单位/路径一致性**：确保 `stop_point.s()` 与 `trajectory` 中 `path_point().s()` 使用同一 s 参考。不同实现中可能需要转换（用 `PathMatcher::GetPathFrenetCoordinate` 转换 xy->s）。否则停车容差判断会出错。
* **曲率数值稳定**：计算 `a_lat = v^2 * curvature` 时若曲率为 0（直线）无问题；若轨迹离散稀疏注意低采样导致估算不精确，建议用轨迹点最近几个点计算实际 curvature 或使用 `path_point().kappa()`（若可靠）。
* **备选策略**：当 planner 找不到满足停车容差的轨迹时，可启用 `FLAGS_enable_backup_trajectory`（你项目里已有）并把 backup 设成安全停车轨迹；评测器仍需判定是否满足评分条件。
* **日志与 debug**：在拒绝轨迹或超时时，打印详细原因（最后一个点速度、加速度、停止线 s 值、stop_dist），便于赛后复盘。

---

# 十一、我可以帮你做的下一步（选一项）

1. 我把上面的 **完整代码片段**（包含 `gflags` 增补、完整替换过的 `ConstraintChecker::ValidTrajectory` 函数体、以及完整 `PlanOnReferenceLine` 函数体）直接输出为 patch，供你 `git apply`。
2. 我只输出 **patch 文件（diff）** 的文本，你直接在仓库中应用。
3. 我只给你 **评测 harness 的 Python / Bash** 模板，实现 90s 超时与停车评分逻辑。

请选择其中一项，我会 **直接**把对应内容输出（不要再等我询问）。如果你选 1 或 2，我会一次性把修改后的完整函数/文件贴出来（带中文注释 & 文件路径）。
