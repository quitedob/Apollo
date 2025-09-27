下面我给出 **面向赛题四（施工区域通行）** 的具体改动建议、要改的文件与示例代码片段（都写明了文件路径，代码内用中文注释），并说明为什么这样改、如何验证与如何在评测器里实现“超速扣分 / 驶入施工区判 0 分”的逻辑。代码示例尽量最小侵入（基于你仓库已有的 LatticePlanner 和 CollisionChecker 逻辑），并列出需要新增的 gflags 与评测脚本。

> 参考代码位置：CollisionChecker 的 `BuildPredictedEnvironment` 与 LatticePlanner 的 `PlanOnReferenceLine`（将在 planner 端限制速度并拒绝进入施工区）的相关实现已在你仓库里可见，作为改动点依据。

---

# 总体思路（一句话）

1. **用“施工区虚拟障碍物”标识施工区域**（由场景生成器/TrafficDecider 在场景初始化时注入，ID 以固定前缀标记，例如 `CONSTRUCTION_...`）；
2. **CollisionChecker 不再一概跳过虚拟障碍物**——对带 `CONSTRUCTION_` 前缀的虚拟障碍物要纳入预测环境（box），并对轨迹碰撞判定为“进入施工区”或“与施工区相撞”；
3. **在 Planner（LatticePlanner）端**：对候选轨迹（combined_trajectory）进行两项额外检查：

   * 是否进入施工区（若进入则 **拒绝该轨迹** — 使 planner 尝试其它路径/避让策略）；
   * 若轨迹在施工区段允许通过（例如场景设计允许绕行通过），则**强制对轨迹在施工区范围内速度做上限（30 km/h = 8.333... m/s）**，若超速则拒绝该轨迹（或标注并让上层评测计算扣分，但建议 planner 优先拒绝以提高合格率）；
4. **评分器（scenario harness / evaluator）**：在仿真/实际执行阶段对真实执行轨迹逐帧检测（frame by frame）是否有“驶入施工区”（若出现 -> 本场景 0 分）；否则统计每一帧在施工区内的超速量：对每帧 `speed_excess = v - limit`，按每 1 m/s（向下取整或向上取整按规则）每帧扣 2 分（你给的是“每超速 1m/s, 本场景每帧扣2分” —— 我把实现按 floor(speed_excess) * 2/帧）的方式实现并累计，最后从该题 100 分中扣除。

---

# 需要新增/修改的文件（清单）

1. 新增/修改 gflags：`modules/planning/planning_base/gflags/planning_gflags.h` / `.cc` —— 新增几个竞赛相关 flag。
2. 修改 CollisionChecker：`modules/planning/planners/lattice/behavior/collision_checker.cc` —— 不再对特定虚拟障碍（施工区）跳过，且存储施工区 box 以供查询（并增加 helper `IsTrajectoryEnteringConstructionZone`）。（基于你现有的 BuildPredictedEnvironment，实现处见你仓库。）
3. 修改 LatticePlanner：`modules/planning/planners/lattice/lattice_planner.cc` —— 在 `PlanOnReferenceLine` 对候选轨迹做施工区进入检测与施工区速度上限检查（在选轨迹处加入 reject 逻辑）。
4. 新增/修改评测器脚本（Python）：例如 `scripts/evaluator/construction_zone_scoring.py` —— 逐帧统计超速扣分/判 0 分（跑仿真 log / 行车轨迹时使用）。
5. （可选）TrafficDecider 或 Scenario Builder：在场景初始化时注入 “施工区虚拟障碍物（ID 前缀 `CONSTRUCTION_`）”，并保证该障碍物的 bounding box 覆盖施工路段；若你已有场景生成器，请在场景描述里按惯例添加该虚拟障碍。

---

# 1) gflags：新增参数（路径 & 示例）

文件路径（修改/新增）：

```
modules/planning/planning_base/gflags/planning_gflags.h
modules/planning/planning_base/gflags/planning_gflags.cc
```

示例（只展示新增部分）：

**`modules/planning/planning_base/gflags/planning_gflags.h`**

```cpp
// 文件路径: modules/planning/planning_base/gflags/planning_gflags.h
// 新增：施工区相关 flag，中文注释简要说明
DECLARE_double(construction_area_speed_limit);    // 施工区域限速 (m/s)
DECLARE_string(construction_obstacle_id_prefix);  // 施工区虚拟障碍 ID 前缀, 例如 "CONSTRUCTION_"
DECLARE_double(construction_penalty_per_frame_per_mps); // 每帧每超速1m/s扣分 (score points)
```

**`modules/planning/planning_base/gflags/planning_gflags.cc`**

```cpp
// 文件路径: modules/planning/planning_base/gflags/planning_gflags.cc
DEFINE_double(construction_area_speed_limit, 8.333333, "施工区域限速(m/s), 默认 30 km/h");
DEFINE_string(construction_obstacle_id_prefix, "CONSTRUCTION_", "施工区虚拟障碍物 ID 前缀");
DEFINE_double(construction_penalty_per_frame_per_mps, 2.0, "施工区内每帧每超速1m/s 扣分");
```

---

# 2) CollisionChecker：把施工区虚拟障碍纳入 predicted_env 并记录（示例补丁）

文件：

```
modules/planning/planners/lattice/behavior/collision_checker.cc
```

关键点与示例代码（插入/替换处在 `BuildPredictedEnvironment` 与类声明处）：

```cpp
// 文件路径: modules/planning/planners/lattice/behavior/collision_checker.cc
// 说明：修改 BuildPredictedEnvironment，使得带有 construction 前缀的虚拟障碍不被跳过，
// 并记录 construction zone 的 boxes（用于判定进入施工区）

// 1) 类成员（在 header collision_checker.h 中添加）
// std::vector<std::vector<Box2d>> predicted_bounding_rectangles_; // 已有
// 新增：与 predicted_bounding_rectangles_ 同步的布尔掩码：该点的 obstacle 是否为施工区
// std::vector<std::vector<bool>> predicted_is_construction_mask_;


// 2) 在 BuildPredictedEnvironment 中修改如下（只展示关键修改段）
// --- 开始修改段 ---
for (const Obstacle* obstacle : obstacles) {
  // 以前仓库：直接跳过虚拟障碍
  // if (obstacle->IsVirtual()) { continue; }

  // 改为：若是虚拟障碍，则仅跳过非施工区的虚拟障碍
  if (obstacle->IsVirtual()) {
    // 简单检测：ID 前缀表示施工区（场景生成器需以该约定注入）
    const std::string& oid = obstacle->Id();
    if (oid.find(FLAGS_construction_obstacle_id_prefix) != 0) {
      // 非施工区的虚拟障碍仍然跳过
      continue;
    }
    // 否则带有 construction 前缀的虚拟障碍会继续被放入 obstacles_considered，
    // 其 box 会成为 predicted_bounding_rectangles 的一部分，从而被判 collision。
  }

  obstacles_considered.push_back(obstacle);
}

// ...（后面按原逻辑对每个 obstacle 做 GetPointAtTime, GetBoundingBox）
// 但同时我们在 push_back box 时，需要同步设置一个 mask，标出该 box 是否来源于是施工区：
Box2d box = obstacle->GetBoundingBox(point);
box.LongitudinalExtend(2.0 * FLAGS_lon_collision_buffer);
box.LateralExtend(2.0 * FLAGS_lat_collision_buffer);
predicted_env.push_back(std::move(box));

// optional: 记录该 box 是否是 construction（便于后续判断轨迹是否“进入施工区”）
bool is_construction =
    (obstacle->IsVirtual() && obstacle->Id().find(FLAGS_construction_obstacle_id_prefix) == 0);
predicted_is_construction_mask_current_frame.push_back(is_construction);
// --- 结束修改段 ---
```

然后在类里新增 helper 函数（示例）：

```cpp
// 文件路径: modules/planning/planners/lattice/behavior/collision_checker.cc
// 新增：判断某条轨迹是否进入施工区
bool CollisionChecker::IsTrajectoryEnteringConstructionZone(
    const DiscretizedTrajectory& discretized_trajectory) {
  // 遍历轨迹每一点，构造 ego_box，并判断是否与 predicted_bounding_rectangles_[i]
  // 中的 construction box 有 overlap（只判断 mask 为 construction 的 boxes）
  for (size_t i = 0; i < discretized_trajectory.NumOfPoints(); ++i) {
    // 构造 ego_box (与 InCollision 相同)
    // 遍历 predicted_bounding_rectangles_[i] 与 predicted_is_construction_mask_[i]
    for (size_t j = 0; j < predicted_bounding_rectangles_[i].size(); ++j) {
      if (!predicted_is_construction_mask_[i][j]) continue;
      if (ego_box.HasOverlap(predicted_bounding_rectangles_[i][j])) {
        return true;  // 进入施工区
      }
    }
  }
  return false;
}
```

**解释与注意**：

* 我采用了“ID 前缀约定（`CONSTRUCTION_`）”作为场景注入施工区的简单实现（避免改 proto）。场景生成器在创建虚拟障碍时把 `Id` 设成 `CONSTRUCTION_<xxx>`。你也可以用 obstacle 的某个 meta tag 字段实现（更健壮）。
* 这里把施工区 box 当作普通障碍 box 来构建，因此 planner 的现有碰撞判断会阻止进入施工区（即满足“驶入施工区本场景计0分”的规则——更安全的做法是阻止进入，从而避免 0 分）。但评测器还要检测是否“被强制进入”（例如 backup trajectory）并判 0 分。
* 相关原始函数体及 box 扩展逻辑在你仓库中已有（见 BuildPredictedEnvironment 片段）。

---

# 3) Planner（LatticePlanner）端：在选轨迹前后加判定 + 施工区速度上限

文件：

```
modules/planning/planners/lattice/lattice_planner.cc
```

修改的位置：`PlanOnReferenceLine(...)` 中在合并轨迹后（即 `combined_trajectory` 生成并通过 ConstraintChecker 检查后，但在最终 accept 之前）加入两步检查：

1. `collision_checker.IsTrajectoryEnteringConstructionZone(combined_trajectory)` → 若 true，则 `continue;`（拒绝该轨迹，尝试其它轨迹）；
2. 对 `combined_trajectory` 的所有轨迹点：若某点被判定位于施工区（可以用 collision_checker 的 predicted_is_construction_mask 或重用 path->s 比对 construction zone s-range），则检查其速度 `v` 是否 ≤ `FLAGS_construction_area_speed_limit` —— 若任一点超速则 `continue;`（拒绝该轨迹）；或根据你想要让 planner 允许但记录超速，改为 `mark` 并交由评测器扣分（但建议 planner 优先拒绝超速轨迹）。

示例插入（在合并轨迹并通过 ConstraintChecker & 未碰撞后）：

```cpp
// 文件路径: modules/planning/planners/lattice/lattice_planner.cc
// 在 pick trajectory（通过 constraint & collision 检查）后插入：

// 1) 施工区进入检查
if (collision_checker.IsTrajectoryEnteringConstructionZone(combined_trajectory)) {
  ADEBUG << "Reject trajectory: would enter construction zone.";
  continue;  // 尝试下一个轨迹
}

// 2) 若轨迹在施工区内，强制速度上限检查
bool violate_construction_speed = false;
for (const auto& pt : combined_trajectory) {
  // 简单 - 判断 pt 是否在 construction region：我们可以调用 collision_checker 的 helper：
  if (collision_checker.IsPointInConstructionZone(pt.path_point().x(), pt.path_point().y(), pt.relative_time())) {
    if (pt.v() > FLAGS_construction_area_speed_limit + 1e-6) {
      violate_construction_speed = true;
      break;
    }
  }
}
if (violate_construction_speed) {
  ADEBUG << "Reject trajectory: construction-zone speed limit violated.";
  continue;
}
```

**说明**：

* `IsPointInConstructionZone(x,y,t)` 是你需要在 `CollisionChecker` 中再补一个小 helper（利用 predicted_bounding_rectangles_ 与 mask 判断某点是否在 construction box），便于快速判断单点是否位于施工区（实现类似 `IsTrajectoryEnteringConstructionZone` 的点检测但只对单点）。
* 这样改能在 planner 层面避免通过施工区时超速；若 planner 找不到任何合格轨迹（例如必须穿过施工区而无法绕行），则会 fallback 到 backup trajectory 或返回 planning error，上层评测器会据此判定（例如认为主车未避让/进入施工区 -> 0 分 或者在没有超速情形下允许通过并由评测器按帧扣分）。

代码中已有 `speed_limit = reference_line_info->reference_line().GetSpeedLimitFromS(init_s[0]); reference_line_info->SetLatticeCruiseSpeed(speed_limit);` 的地方可以作补充：当当前位置处在 construction 区段起点时，cap 初始 cruise speed（但更精确的做法是按每个轨迹点判断）。

---

# 4) 评测器（scoring harness）：逐帧扣分与 0 分规则（Python 示例）

文件（新增）：

```
scripts/evaluator/construction_zone_scoring.py
```

核心思路：读取执行日志（每帧 ego pose、速度、时间），并读取场景定义中施工区 polygon / bounding box（或读取场景中注入的 `CONSTRUCTION_...` obstacle boxes 与时间序列），然后：

* 如果任一帧 ego 的位置与施工区 box overlap -> 该场景直接得 0 分（按比赛规则）。
* 否则，对每一帧：若 ego 在施工区内（场景可能允许局部穿行绕行），计算 `excess = v - FLAGS_construction_area_speed_limit`（取正），每超速 1.0 m/s 对该帧扣 `FLAGS_construction_penalty_per_frame_per_mps` 分（你可以实现为 `penalty += floor(excess) * penalty_per_frame` 或 `ceil`，按裁判最终定义）。将累计 penalty 从 100 分中扣除（若扣完则 0 分）。

示例（精简）：

```python
# 文件: scripts/evaluator/construction_zone_scoring.py
# 中文注释：施工区计分器（示例）
import math

def point_in_box(px, py, box):
    # box: [xmin,xmax,ymin,ymax] 简化示例，实际用 Box2d 矩形判定
    xmin, xmax, ymin, ymax = box
    return xmin <= px <= xmax and ymin <= py <= ymax

def score_run(frames, construction_boxes, speed_limit=8.3333, penalty_per_frame_per_mps=2.0):
    score = 100
    for frame in frames:
        x, y, v = frame['x'], frame['y'], frame['v']
        # 若进入施工区 -> 0 分
        for cbox in construction_boxes:
            if point_in_box(x,y,cbox):
                # 如果场景规则是“驶入施工区域本场景计0分”
                return 0
    # 若未进入，但可能存在“通过施工区域但超速扣分”的规则
    # 若你的场景希望允许在施工区域内通行但扣分（题目描述里“主车通过施工区域场景，但主车未限速，速度每超速1m/s, 本场景每帧扣2分。”）
    # 那么若施工区允许通过（不直接 0 分），我们需要统计在施工区域内的每帧超速
    penalty = 0
    for frame in frames:
        x, y, v = frame['x'], frame['y'], frame['v']
        for cbox in construction_boxes:
            if point_in_box(x,y,cbox):
                excess = max(0.0, v - speed_limit)
                # 每超速 1 m/s 每帧扣 penalty_per_frame_per_mps 分（用 floor）
                penalty += math.floor(excess) * penalty_per_frame_per_mps
                break
    score = max(0, 100 - penalty)
    return score
```

**注意**：

* 这里 `frames` 应为仿真/记录器导出的 ego trajectory（每帧时间戳/位置/速度）。
* `construction_boxes` 可由场景定义加载（即场景注入的 `CONSTRUCTION_` 虚拟障碍的 bounding boxes）。
* 这里的实现示例把“驶入施工区”与“通过施工区但超速”区分开；你要按题目最终裁判规则取舍（题目中两句出现似乎有冲突：一处说 “主车驶入施工区域本场景计0分”，另一处 “主车通过施工区域场景，但主车未限速...扣分”——通常解释为：“如果不绕开而直接开到施工区域占用车道内部（即进入被占用区域）那是 0 分；但如果施工区有专门通行通道（允许通过），则通过但必须限速，否则扣分”。评测脚本要和场景定义一致）。

---

# 5) 场景/TrafficDecider：如何注入施工区（建议）

* 在场景定义阶段（Scenario Builder 或 TrafficDecider 风格的插件），用 **虚拟障碍 (virtual obstacle)** 表示施工区（bounding polygon / box），并把 `Id` 设为 `CONSTRUCTION_<scene_id>` 或者给 obstacle 一个 meta tag `construction=true`。
* 这样 CollisionChecker 在 `BuildPredictedEnvironment` 中就能识别并把施工 box 放入 `predicted_bounding_rectangles_`，planner 与评测器即可使用这些 box 做判定（代码在上面已依据 `Id` 前缀加入识别逻辑）。

---

# 6) 验证步骤（必须执行）

1. 在场景生成器中创建一个测试场景（带 `CONSTRUCTION_` 虚拟障碍，覆盖道路一段），并在场景说明里声明该区段允许通过（或不允许通过，按题目定义）。
2. 本地编译 planner：`bazel build //modules/planning/...` 并运行规划单测。
3. 运行一组回归测试：

   * 情形 A：在不进入施工区且不超速情况下通过 → 得分 100；
   * 情形 B：必须穿过施工区但车辆速度超过 8.33 m/s 若 planner 允许则评测器应扣分（按帧统计）；若 planner 被设为拒绝则 planner 返回 fallback，场景可能判 0（需与评测规则对齐）；
   * 情形 C：车辆实际进入施工区（被检测到 overlap）→ 得 0 分。
4. 仔细检查日志：`ADEBUG`/`AERROR` 中应能看到 “Reject trajectory: would enter construction zone.” 或 “Reject trajectory: construction-zone speed limit violated.”。

---

# 7) 额外建议与注意事项（实战要点）

* **定义明确**：务必和比赛/评测方明确“驶入施工区”与“通过施工区限速”两条规则的关系（是互斥、优先还是并列），以确认是 planner 拒绝还是评分器判 0/扣分。
* **场景注入规范**：建议在场景描述里提供施工区 `s` 范围（Frenet s-start, s-end）或明确 obstacle `Id` 前缀；统一约定可让实现更稳健（避免靠字符串匹配外壳）。
* **Planner fallback**：若 planner 找不到任何合格轨迹（例如所有路径都会把 ego 逼入施工区），建议使用 `BackupTrajectoryGenerator` 作为安全停靠（但评测器仍会判断是否进入施工区并给 0 分）。
* **时间分辨率 & “每帧扣分”**：评测器中的“每帧”分辨率要和仿真输出帧率一致（比如 10 Hz 则每 0.1s 一帧），并在赛前统一。
* **体积/性能**：新增的 predicted_is_construction_mask_ 数组会与 predicted_bounding_rectangles_ 同步，代价极小（只在 planning frame 内），但要注意内存/拷贝。

---

# 关键代码位置（供定位）

* CollisionChecker::BuildPredictedEnvironment（修改点）：`modules/planning/planners/lattice/behavior/collision_checker.cc`。原来对 obstacle->IsVirtual() 的直接跳过需要修改，box 的生成与扩展逻辑在此处（见仓库片段）。
* LatticePlanner::PlanOnReferenceLine（插入位置）：`modules/planning/planners/lattice/lattice_planner.cc`，在合并并通过约束/碰撞检查后接受轨迹之前加入施工区/速度检查（片段中已有对 speed_limit 的读取，可在此扩展）。

---

# 我可以继续为你做（你选一个）

A. 把上面所有代码改动做成 **patch/diff**（完整的函数/文件差分），你拿去 `git apply` 即可；
B. 只输出 **完整函数实现**（包含新增 helper）并标明修改行/文件，便于你复制粘贴回仓库；
C. 输出 **评测器完整 Python 文件**（`construction_zone_scoring.py`），可直接放到 `scripts/evaluator/` 下并能读取指定格式的仿真 log；
D. 帮你把上面的改动写成 **PR 描述模板**（包括要 test 的场景与 CI 测试项），方便提交审查。

选一个我就**直接**把对应内容完整输出（含文件路径、中文注释、可直接替换/应用的代码）。
