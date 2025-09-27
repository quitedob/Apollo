好——我将给出一套**可执行的代码/配置修改建议与校验步骤**，把你贴出的竞赛规则（加速度/减速度/向心加速度/速度上限、以及停止线前2.0–2.5 m 的强制停车）硬性落到 Apollo planning / traffic-rule 模块中。下面包含：**要改的文件、具体修改内容（可复制的代码片段）**、以及**验证/回归测试建议**。我同时把关键依据（你仓库里的定义）一并引用，便于核对和审计。

---

# 快速结论（要做的核心修改）

1. **把纵向加速度上限改为 `+3.0 m/s^2`，纵向减速度下限保留 `-6.0 m/s^2`（或更严格）**，以满足：加速度 ≤ 3、减速度 ≤ 6。
2. **把向心/横向加速度边界设为 `2.0 m/s^2`**（centripetal acceleration ≤ 2.0）。ConstraintChecker 中已有对 lateral acceleration 的校验，调整对应 flag 即可生效。 
3. **把规划速度上限改成 `16.67 m/s`（即 60 km/h）并把 city default 降至 ≤ 16.67 m/s**，避免 planner 生成更高速率轨迹。当前默认 city 值为 15.67（注意：比 16.67 小）；planning 上限默认更高，需要改。 
4. **把 StopSign 的 stop_distance 配置改为 2.0–2.5m 范围（建议 2.25m），并在 StopSign::MakeDecisions 增加“强制区间检查/修正”逻辑**（若规划出来的停车点不在 [2.0,2.5] 范围内，调整停点或触发 fail/重新规划）。当前默认为 1.0m，需要修改。

另外：强烈建议把“**竞赛评分/判定**”放到一个**独立的评估脚本**中（比赛评判器），在仿真里对每个运行轨迹做最终判定与扣分（参考 stop 距离与是否避让等），这样能避免直接改核心 planner 的其他行为造成副作用。

下面给出具体修改步骤、代码片段与验证方法。

---

# 具体修改（按文件逐条，含可复制代码与中文注释）

> 说明：以下修改点都基于你仓库中已有的 flag / 模块（我在你上传的 `modules_All.txt` 中找到了这些 flag 和 stop_sign 模块）。对应的文件片段引用如下，用于核验：StopSign 默认 conf 在 modules/planning/traffic_rules/stop_sign/conf/default_conf.pb.txt（默认 `stop_distance: 1.0`）。
> planning flags（longitudinal & lateral & speed）定义可在 planning_gflags 相关位置找到（示例行已在仓库片段中）。  

---

## 1) 修改：planning flags — 速度与加速度边界

**文件（要修改的）**：`modules/planning/common/planning_gflags.cc` 或 `modules/planning/planning_gflags.cc`（你仓库的 flags 定义位置；在你的 `modules_All.txt` 里这些 flag 定义被收录在 planning flags 段）。
（如果你的代码把 flags 放在 `modules/planning/planning_gflags.h` / `.cc`，请在对应 DEFINE_double 行上替换数值。）

**要改的行（示例）** — 将下面三行替换为给定值：

```cpp
// 文件路径: modules/planning/planning_gflags.cc
// 修改：设置纵向加速度和横向（向心）加速度、规划速度上限
// --- 以下注释为中文，说明每一项的含义 ---

// 将纵向减速度下限（制动）保持为 -6.0（m/s^2），确保不超制动限制
DEFINE_double(longitudinal_acceleration_lower_bound, -6.0,
              "The lowest longitudinal acceleration allowed.");

// 将纵向加速度上限严格设为 +3.0 m/s^2（竞赛要求）
DEFINE_double(longitudinal_acceleration_upper_bound, 3.0,
              "The highest longitudinal acceleration allowed.");

// 将横向/向心加速度限制设为 2.0 m/s^2（竞赛要求）
DEFINE_double(lateral_acceleration_bound, 2.0,
              "Bound of lateral acceleration; symmetric for left and right");

// 将 planning 的最大速度上限设为 16.67 m/s（60 km/h）
DEFINE_double(planning_upper_speed_limit, 16.67,
              "Maximum speed (m/s) in planning.");
```

**为什么改这些**：ConstraintChecker 与 trajectory 评估会读取这些 flags 来判定轨迹是否超限（你仓库的 ConstraintChecker 已用这些 flag 判断纵横向加速度与 jerk）。 

---

## 2) 修改：默认城市限速（可选/建议）

**文件**：`modules/planning/planing_gflags.cc`（或含 `default_city_road_speed_limit` 的地方）

**修改**（确保 city road 的默认 speed ≤ 16.67）：

```cpp
// 文件路径: modules/planning/planning_gflags.cc

// 当前默认（示例）：DEFINE_double(default_city_road_speed_limit, 15.67, ...)
DEFINE_double(default_city_road_speed_limit, 16.67,
              "default speed limit (m/s) for city road.");
```

> 注：你现仓库显示默认 city 值是 `15.67`（35mph）。把它设成 `16.67` 或更保守的 `15.67` 都可，只要不大于 16.67 即可。

---

## 3) 修改：StopSign 默认 stop_distance（配置文件） & 增加区间化检查逻辑

### A — 修改 conf（最小且必须）

**文件**：`modules/planning/traffic_rules/stop_sign/conf/default_conf.pb.txt`

**替换内容**（将 `stop_distance` 从 1.0 改为 2.25，作为默认值；但关键的是后面的逻辑会强制在 [2.0,2.5] 之内）：

```proto
# 文件路径: modules/planning/traffic_rules/stop_sign/conf/default_conf.pb.txt
enabled: true
# 将默认停靠距离改为竞赛要求中间值 2.25m（便于容错）
stop_distance: 2.25
```

> 当前默认原本是 `stop_distance: 1.0`，需要修改。

### B — 在 StopSign::MakeDecisions 中 **强制检查/调整** 停车点到停止线距离落在 `[2.0, 2.5]` 范围

**文件**：`modules/planning/traffic_rules/stop_sign/stop_sign.cc`

**说明**：在构造 stop decision（planner 要在 stop_line **之前**停车的位置）之后，插入一段检查代码：

* 计算 `actual_stop_distance = stop_line_s - planned_stop_s`（单位 m）
* 如果 `actual_stop_distance < 2.0`，则向后移动停靠点（把 planned_stop_s 减小，以增加距离），但不得超过地图可行范围或越过车辆最大允许逆行边界。
* 如果 `actual_stop_distance > 2.5`，则将停靠点向前移动，确保不越过停止线；如果无法，则触发一个“需要重新规划”的标记（或直接返回 fail，让 planner 重新采样更低速轨迹）。

**修改示例（仅示意 MakeDecisions 中关键片段）**：

```cpp
// 文件路径: modules/planning/traffic_rules/stop_sign/stop_sign.cc
// 修改位置: StopSign::MakeDecisions 函数内部（插入/替换停车点计算后）

// ====== 插入前：你现有代码会基于 config_.stop_distance() 生成 stop decision ======
// 插入检查与修正逻辑（中文注释说明）
double min_req = 2.0;   // 竞赛要求最小停距 (m)
double max_req = 2.5;   // 竞赛要求最大停距 (m)
double stop_line_s = /* 地图中停止线的 s 值，stop_sign_overlap.* 中获取 */ stop_sign_overlap.end_s;
double planned_stop_s = /* 当前生成的计划停靠点 s（参考你原逻辑） */ some_planned_stop_s;

// 计算停车点与停止线之间的实际距离（停止线在前，车辆停在停止线前）
double actual_stop_distance = stop_line_s - planned_stop_s;

// 如果停得太靠前（距离 < min_req），尝试将停靠点后退（增加距离）
if (actual_stop_distance < min_req) {
  double delta = min_req - actual_stop_distance;
  // 向后退 delta（注意边界和 reference line limits）
  double new_planned_stop_s = planned_stop_s - delta;
  if (new_planned_stop_s < adc_back_edge_s) {
    // 无法满足：new stop point 越过车辆后端边界或不可行
    // 这里建议标记 need_replan 或者强制使用最低速制止并记录违规（供评估器扣分）
    AERROR << "Cannot adjust stop point to meet min_req for stop_sign["
           << stop_sign_overlap.object_id << "]";
    // 触发 replan 或直接构造紧急停轨迹
  } else {
    planned_stop_s = new_planned_stop_s;
    actual_stop_distance = stop_line_s - planned_stop_s;
  }
}

// 如果停得太靠后（距离 > max_req），尝试将停靠点前移（但不能越过停止线）
if (actual_stop_distance > max_req) {
  double delta = actual_stop_distance - max_req;
  double new_planned_stop_s = planned_stop_s + delta;
  if (new_planned_stop_s >= stop_line_s) {
    // 不能越过停止线，需 replan 或降速接近再微调
    AERROR << "Adjustment would cross stop line for stop_sign["
           << stop_sign_overlap.object_id << "]";
    // 标记需要重新规划或降速并再次采样
  } else {
    planned_stop_s = new_planned_stop_s;
    actual_stop_distance = stop_line_s - planned_stop_s;
  }
}

// 最终：把 planned_stop_s 写入 stop decision 并注入 reference_line_info
// 同时，为了比赛判定，建议把 actual_stop_distance 写入 planning_context 或一个评估 topic/日志，方便赛后计算扣分
```

> 注：上面的伪码需要和你现有 `MakeDecisions` 的具体停点变量名做对接 —— 关键思想是**在 StopSign 层面把停车点 `planned_stop_s` 的计算结果约束到 [2.0, 2.5] 区间**，并在失败时触发 replan/记录违规。StopSign 模块在你的仓库中负责构造 stop decision（参见 StopSign::MakeDecisions）。

---

## 4) 强化 ConstraintChecker / TrajectoryValidator（运行时兜底）

**目的**：即使 planner 生成了轨迹，上面设置的 flags 会在 `ConstraintChecker::ValidTrajectory` 执行中被校验（如果你启用 trajectory check）。但为了比赛稳妥，建议**打开并启用轨迹检查**，且在检测到越界（例如 LON_ACCELERATION_OUT_OF_BOUND 或 LAT_ACCELERATION_OUT_OF_BOUND）时，**立即触发 replan 或发布 emergency stop**。

**配置/修改要点**：

* 在 flags 中把 `enable_trajectory_check` 设为 `true`（如 `DEFINE_bool(enable_trajectory_check, true, ...)`）。（你仓库中有该 flag，当前默认 false。）
* 在 `Planning` 节点生成轨迹后立刻调用 `ConstraintChecker::ValidTrajectory`，若返回非 VALID，则抛弃该轨迹并触发 fallback（低速慢速停止轨迹）。（你的 ConstraintChecker 已包含对应返回值枚举。）

---

## 5) 比赛评分 / 判定脚本（建议）：在仿真端做最终判定

**建议**：写一个独立脚本 `tools/competition_evaluator.py`，在每次仿真结束后读取车辆轨迹或 planner 输出，执行以下判定并输出扣分/OK：

* `max_longitudinal_acceleration` 超 3 m/s^2 -> 立即判罚（记分/终止该场景）。
* `max_longitudinal_deceleration` 小于 -6 m/s^2 -> 立即判罚。
* `max_lateral_acceleration` 超 2 m/s^2 -> 立即判罚。
* `max_speed` > 16.67 -> 立即判罚。
* 停车时：计算 `stop_distance = stop_line_s - stop_pose_s`，若不在 [2.0,2.5]，扣 20 分（或按比赛规则）。若越过停止线或未停车 -> 0 分 / fail。

**示例（伪 Python）**：

```python
# 文件路径: tools/competition_evaluator.py
# 简单说明：读取仿真轨迹（csv/rosbag），并对关键量做判断与扣分
# (此处为示例，具体读取方式根据你仿真输出调整)

def evaluate_trajectory(traj):
    # traj: list of points with t, x,y, v, ax, ay, kappa, stop_flag, obstacle_flags...
    max_ax = max(p['ax'] for p in traj)
    min_ax = min(p['ax'] for p in traj)  # 负值为制动
    max_lateral_a = max(abs(p['ay']) for p in traj)
    max_speed = max(p['v'] for p in traj)

    if max_ax > 3.0:
        return 'FAIL', 'Longitudinal acceleration exceeded 3.0'
    if min_ax < -6.0:
        return 'FAIL', 'Deceleration exceeded 6.0'
    if max_lateral_a > 2.0:
        return 'FAIL', 'Centripetal acceleration exceeded 2.0'
    if max_speed > 16.67:
        return 'FAIL', 'Speed exceeded 16.67 m/s'

    # 停车距离判断（示例）
    if traj[-1]['stopped']:
        stop_line_s = get_stop_line_s()  # 从地图或事件获取
        stop_pose_s = traj[-1]['s']
        stop_dist = stop_line_s - stop_pose_s
        if stop_dist < 2.0 or stop_dist > 2.5:
            return 'PENALTY', f'stop distance {stop_dist:.2f} outside [2.0,2.5]'
    else:
        return 'FAIL', 'Did not stop at sign'

    return 'OK', 'Passed'
```

---

# 编译 / 部署 / 验证步骤（实践操作）

1. **修改代码并提交**（按照上面文件与片段替换）。
2. **bazel build** 重新编译 planning 模块，例如（视你 repo 的 bazel target）：

```bash
# 在工作区根目录
bazel build //modules/planning:all
```

3. **启动仿真测试**：在仿真环境（Dreamview / LGSVL / 评测场景）运行包含 stop sign 场景的用例，让 planner 执行。
4. **打开 trajectory 检查**（enable_trajectory_check = true），观察日志（ConstraintChecker 报错会打印），并在出错时触发 fallback。
5. **运行 tools/competition_evaluator.py** 对记录轨迹做赛题判定并输出扣分 / PASS / FAIL。

---

# 关键引用（仓库内证据 & 官方/社区文档）

**仓库内（你上传的文件片段，说明了 flags 与 stop_sign 位置）**：

* StopSign 模块与默认配置 `stop_distance`（默认 1.0，必须改） — `modules/planning/traffic_rules/stop_sign/conf/default_conf.pb.txt`。
* ConstraintChecker 对 lateral acceleration 的校验片段（用到 lateral_acceleration_bound flag）。
* planning flags 中纵向、横向、速度上限的默认值（供参考）。  

**来自 Apollo 社区 / 文档（互联网上的参考，用于说明 StopSign 模块流程与配置项）**：

* StopSign 与 STOP 处理流程说明（社区/文档参考，说明 StopSign 是在 PRE_STOP→STOP 等阶段生效，并有 stop_distance 参数）。([Apollo开发者社区][1])
* 中文博客 / 文档对 planning-traffic-rules-stop-sign 的说明（便于理解 conf/default_conf.pb.txt 的含义与位置）。([CSDN博客][2])
* Apollo 文档中 traffic rules conf 的在线参考（示例位置与用法）。([Apollo开发者社区][3])

---

# 进一步建议与可选优化（提高鲁棒性与通过率）

1. **Trajectory sampling 时增加更低速采样点**（使 planner 能更平滑地降速到 2.0–2.5m 停止点），避免必须在 StopSign 层面做过多强行调整。可改 `num_velocity_sample` / `default_cruise_speed` / sampling lower bound。
2. **在生成 stop trajectory 时增加停止线附近精细采样（lattice_stop_buffer）**，确保规划出的停止点精度足够。
3. **对 StopSign 的 replan 策略**：当无法物理满足 [2.0, 2.5] 时，立即降速进入保守模式（低速 creeping）并标记为“场景失败/需要人工复核”，避免仓促强制越线。
4. **开发单元测试 & 仿真测试集**：构造 3 个不同初始速度/路段曲率的 stop sign 场景（高速接近/中速/低速），用自动化脚本验证所有场景均能满足规则。
5. **日志输出**：把 `actual_stop_distance`、`max_ax`、`max_ay` 写入 `planning_status`（或单独 topic），便于赛后自动化打分与回放。

---

# 小结（一句话）

把 **planning 的速度/加速度/lateral 准入阈值** 改成竞赛要求的数值（纵向上界 `+3.0`、下界 `-6.0`，横向 `2.0`，速度上限 `16.67 m/s`），并把 **StopSign 的 `stop_distance` 默认改为 2.25m**，**在 StopSign::MakeDecisions 中加入对停车点的区间化检查/修正并在失败时触发 replan 或 fallback**，同时**启用 trajectory 检查、并用独立评估脚本做最终判定**，这样就能在仿真中强制实现你贴出的竞赛规则。

---

如果你希望，我现在可以：

* 1. 直接为你生成 **StopSign::MakeDecisions** 的完整 C++ 函数实现（含中文注释、可直接替换的完整函数体），并标注应替换的文件路径与大致行号；
* 2. 或者直接为你生成 **一个 patch 文本（diff）**，包含 `planning_gflags` 的数值替换、`default_conf.pb.txt` 的改动，以及 `tools/competition_evaluator.py` 的完整脚本示例，便于你 `git apply`。

你想让我现在把哪一种产物直接给你？（我可以直接把可替换的代码/patch 贴出来——无需你再确认，我会按你仓库里已有路径和我上面提到的文件名输出。）

[1]: https://apollo.baidu.com/community/article/1294?utm_source=chatgpt.com "交通标志场景解析 - Apollo开发者社区"
[2]: https://blog.csdn.net/qq1240268067/article/details/148007096?utm_source=chatgpt.com "planning模块（5）之traffic_rule交通规则_apollo狭窄道路通行"
[3]: https://apollo.baidu.com/docs/apollo/9.x/planning_2traffic__rules_2traffic__light_2conf_2default__conf_8pb_8txt.html?utm_source=chatgpt.com "modules planning traffic_rules traffic_light"
