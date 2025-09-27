下面给出 面向“赛题五 xh_2025 — 自主泊车场景” 的可执行解决方案（策略 + 具体代码修改建议），并结合 Apollo 社区 / 选手经验与官方模块实现细节给出可立刻应用到你代码库的补丁思路与核心代码片段。

快览（精要）

复用 Apollo 的 ValetParkingScenario（已有场景/流水线支持“approach + open-space parking”），在此基础上做三项关键增强：（A）入口最近车位选择逻辑；（B）90s 赛时约束检查与失败回退；（C）禁入区（forbidden-zone）检查并在选位/路径规划前过滤。。已在仓库中找到现成模块与配置作为起点（ValetParkingScenario、pipeline/conf、open_space tasks）。

Apollo-main_All

 

modules_All

参考 Apollo 官方/社区文档与选手 issue 的实战经验：Open-space planner（zig-zag / gear shift / heading search）是泊车阶段的核心，通常通过调整 open_space_trajectory_partition / open_space_trajectory_provider 配置与参数能显著提高成功率；选位策略与场景切入判断则在 scenario 层实现。
道可道
+2
Apollo开发者社区
+2

一、整体策略（为什么这样做 & 成功率保障）

使用 Apollo 的 ValetParkingScenario 作为框架（它包含 VALET_PARKING_APPROACHING_PARKING_SPOT 与 VALET_PARKING_PARKING 两阶段），不要重写整个泊车算法，直接在 Scenario 层插入“选位 + 赛时/禁区策略”，这样最小改动并保留 open-space planner 的成熟能力。

Apollo-main_All

 
Apollo开发者社区

赛题要求：进入泊车场后搜索“距离停车场入口处最近”的可用泊位并在 90s 内完成。将“入口最近”逻辑放在 Scenario 的初始化/切入时完成（基于 HD-Map 停车位几何或 perception 的车位检测），并把选定车位传入 planning 的目标（planning command / PlanningTarget）。如果在限定时间内未完成或车辆进入禁行区立即判失败/回退。社区经验显示：正确的选位 + 合理的 open-space 参数（gear shift、heading search range、interpolated pieces 等）是能显著提升成功率的要点。
GitHub
+1

失败与处罚规则：

超时 90s -> 标记场景失败（score=0）。实现方式：scenario 层记录进入泊车场时间戳，parking stage 内周期检查 elapsed。

停车压线扣 20 分 -> 在 final stop 后用车体 box 与车位边界做 IOU/越界检测，超过阈值触发扣分条件（可用于赛后评估）。

禁行区进入 -> 立即停止并置场景失败（score=0）。禁行区从地图(NoParking/Forbidden region)或题目说明解析得到并注入场景配置。

二、需要修改 / 新增的文件（最小可行集）

主要改动点（最小改动原则）：

modules/planning/scenarios/valet_parking/valet_parking_scenario.cc
→ 在场景初始化或切入函数中增加：FindNearestAvailableParkingSpotNearEntrance()；把选定车位写入 scenario/planning target。
（我们在 repo 中已找到 valet_parking 模块与 conf，可直接扩展。）

Apollo-main_All

modules/planning/scenarios/valet_parking/stage_parking.cc（或 stage_approaching_parking_spot.cc）
→ 在 parking stage loop 中增加：90s 超时检查 + final-stop 后压线检测 + 进入禁区检测（若在 StageApproaching 即能检测并提前切换也可）。相关 open_space tasks（OPEN_SPACE_TRAJECTORY_PROVIDER/PARTITION）的配置参数也建议调整。

modules_All

modules/planning/scenarios/valet_parking/conf/scenario_conf.pb.txt
→ 新增两个可调参数：parking_search_timeout_sec（默认 85, 以留出机动时间）与 parking_search_radius（单位 m）用于选位；以及禁行区ID或geo fence 配置字段（或在竞赛中用硬编码/由题库注入）。

三、具体代码补丁（只修改一个函数，方便快速落地）

按你的偏好：我只对单个函数做修改 —— 输出该函数完整代码、文件路径与建议插入行号区间（便于你直接替换）。如果你需要我把整个文件完整输出/替换，也可以接着返回，我会直接给出完整文件。

文件路径（替换位置）
modules/planning/scenarios/valet_parking/valet_parking_scenario.cc
（建议替换：在 ValetParkingScenario::Init(...) 或 OnScenarioStart() 中调用下面的函数；下面给出 新增函数 SelectNearestParkingSpotNearEntrance 的实现，并示例如何调用它。）

注意：由于不同 Apollo 版本 API 命名可能有小差异，下列代码基于 Apollo 常见接口（frame/map/hdmap）写法作合理推断；在你的工程里把 HDMap、ParkingSpot 相关 API 名称替换为项目中实际的接口即可（我在下方也给出如何在代码中定位替换点的提示）。
同时，所有代码块前均加中文注释（按照你给的偏好）。

// 文件：modules/planning/scenarios/valet_parking/valet_parking_scenario.cc
// 说明：新增函数：在场景开始时选取“距离入口最近且可用”的泊位，并写入planning target
// 建议插入位置：ValetParkingScenario::Init(...) 或 OnScenarioStart() 调用

// ----------------- 新增函数开始 -----------------
/*
  功能：选择距离泊车场入口最近的可用车位（排除禁行/已占用）
  输入：
    - const std::string& parking_area_id_or_entry_point_id : 由场景或题目提供的入口标识（如果有）
    - double search_radius_m: 搜索半径（可由 conf 注入）
  输出：
    - bool 返回是否找到可用车位
    - out_parking_spot_id 返回选中的停车位 ID
  说明：
    - 使用 HDMap/Perception 查询停车位几何信息（parking space polygon/center）
    - 检查车位状态（若系统有停车位占用信息，优先使用；否则使用 perception/vision 判断）
    - 过滤禁行区域（map 中的 NoParking/forbidden polygons）
*/
bool ValetParkingScenario::SelectNearestParkingSpotNearEntrance(
    const std::string& entrance_id, double search_radius_m,
    std::string* out_parking_spot_id) {
  // 中文注释：1) 获取入口坐标（若入口是点或线，取入口点近似）
  apollo::common::PointENU entrance_pt;
  if (!GetEntrancePointById(entrance_id, &entrance_pt)) {
    AERROR << "SelectNearestParkingSpot: failed to get entrance point for id "
           << entrance_id;
    return false;
  }

  // 中文注释：2) 从地图获取候选停车位（HDMap parking space feature）
  std::vector<apollo::hdmap::ParkingSpace> candidates;
  if (!hdmap::HDMapUtil::GetParkingSpacesWithinRadius(entrance_pt, search_radius_m, &candidates)) {
    AERROR << "SelectNearestParkingSpot: HDMap query failed or empty";
    return false;
  }

  // 中文注释：3) 遍历候选车位，过滤不可用/禁区/太近障碍等
  double best_dist = std::numeric_limits<double>::infinity();
  std::string best_spot_id;
  for (const auto& ps : candidates) {
    // 获取 parking space id & center
    const std::string spot_id = ps.id().id();
    apollo::common::PointENU center = ps.center();
    // 过滤：如果车位在禁行区（map 标注 NoParking/Forbidden），跳过
    if (hdmap::HDMapUtil::IsPointInNoParkingRegion(center)) {
      ADEBUG << "spot " << spot_id << " in NoParking region, skip";
      continue;
    }
    // 过滤：如果 perception/occupancy 信息表明已占用（有模块可提供），跳过
    if (IsParkingSpotOccupied(spot_id)) {
      ADEBUG << "spot " << spot_id << " occupied, skip";
      continue;
    }
    // 计算入口到车位中心距离并选最小
    double dx = center.x() - entrance_pt.x();
    double dy = center.y() - entrance_pt.y();
    double d = std::hypot(dx, dy);
    if (d < best_dist) {
      best_dist = d;
      best_spot_id = spot_id;
    }
  }

  if (best_spot_id.empty()) {
    ADEBUG << "SelectNearestParkingSpot: no suitable spot found";
    return false;
  }

  // 中文注释：4) 将选中的车位 id 写回 scenario（用于后续 planning target）
  *out_parking_spot_id = best_spot_id;
  AINFO << "SelectNearestParkingSpot selected spot " << best_spot_id
        << " dist_m=" << best_dist;
  return true;
}
// ----------------- 新增函数结束 -----------------

/*
  示例：在场景入口处调用（建议在 ValetParkingScenario::Init 或 OnScenarioStart 中加入）：
    std::string chosen_spot;
    if (SelectNearestParkingSpotNearEntrance(entrance_id, FLAGS_parking_search_radius_m, &chosen_spot)) {
       // 将 chosen_spot 写入 planning command / planning target
       PopulatePlanningTargetWithParkingSpot(chosen_spot, frame);
    } else {
       // 处理：没有找到车位时的 fallback（可选择等待/回退/报告失败）
    }
*/


插入调用建议（伪代码位置）：在 ValetParkingScenario::Init(...) 或 OnScenarioStart() 中加入上述调用并把 chosen_spot 以 PlanningTarget 的形式传入 Frame（frame->mutable_planning_target()->set_parking_spot_id(chosen_spot) 或相似接口）。

四、在 parking stage 中加入 90s 超时 & 禁区/压线检查（实现思路）

把下面逻辑放入 stage_parking.cc 的循环或终止判断处（示例伪代码说明）：

在第一次进入 parking stage 时记录 start_ts = Clock::NowInSeconds()（或从 scenario 的 entrance_ts 拷贝）。

每个 planning tick 检查：if (Clock::NowInSeconds() - start_ts > FLAGS_parking_search_timeout_sec) → 标志场景失败并发出失败日志与 planning::Status::ERROR，触发回退或命令让车辆静止并退出场景（比赛判定 0 分）。

final stop 后进行压线检测：利用 vehicle_box 与 parking_spot_polygon 做碰撞/越界检测（计算 overlap/IOU 或车体角点是否超出泊位边界）。若超出阈值（例如超出 0.1 m）则记录“压线”标志（赛后扣 20 分）。

进禁区检测：在轨迹执行前和执行中均用 IsPointInNoParkingRegion(ego_box_center) 进行检测，一旦检测到立刻 brake & fail。

（以上逻辑可在 StageParking::Execute() 的返回判断中实现，或独立写成 CheckParkingTimeoutAndForbidden() 并被主循环调用。）

五、配置建议（conf 调优）——直接修改 conf 文件即可快速验证

文件：modules/planning/scenarios/valet_parking/conf/scenario_conf.pb.txt（已存在，可直接编辑）

modules_All

建议新增 / 调整字段（示例）：

# 新增（示例）：
parking_search_radius: 40.0   # 搜索半径（m），扩大入口周围搜索范围
parking_search_timeout_sec: 85.0  # 选位+泊车阶段总超时时间（留 5s 作为切换容错）
# 已有：
parking_spot_range_to_start: 20.0
max_valid_stop_distance: 1.0
# open_space 相关（建议保留/微调）
open_space_trajectory_partition {
  heading_search_range: 0.79
  distance_search_range: 2.0
  interpolated_pieces_num: 10
  # ...（根据场地试验微调）
}


说明：上述值基于 Apollo 默认 conf 并结合 open-space 参数（gear shift/heading_search_range 等）微调可提升成功率；社区经验建议将 parking_search_radius 设得比默认 parking_spot_range_to_start 大一些以覆盖“入口附近的最近车位”。

modules_All

 
GitHub

六、为什么这些改动能对应竞赛规则（与社区/论文支撑）

Apollo 的 Valet/ Open-Space Planner 已被用于实际 AVP / 自动泊车，且 pipeline/partition/provider 的参数 directly 决定是否能找到 zig-zag 轨迹与换挡策略。用 Scenario 层选位（并传给 planner）是常见做法。
Apollo开发者社区
+1

学术上（Hybrid A*、search/sampling-based）和工业上（open-space zig-zag）都表明：先确定停车目标点（车位）再运行 open-space planner 对成功率影响最大；同时禁区与超时策略是比赛判罚的直接实现方法。
