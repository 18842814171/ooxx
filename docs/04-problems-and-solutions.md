# 04 — 问题与对策记录

> 本文档整理自 2026 年 7 月局部规划与恢复机制迭代过程中的实车现象、日志分析与架构讨论。  
> 编写遵循 `重要原则.txt`：书面语、参数集中于 `config/mission.yaml`、较大改版须清理旧路径。  
> 与 `01-architecture.md`（控制权）、`02-call_chains.md`（调用链）、`03-module_index.md`（模块职责）配合阅读。

本文遵循以下原则：  
每个问题须写清**现象**（以行为为主，日志作少量佐证）、**原因**（代码行为或架构逻辑，少用变量名堆砌）、**当前版本解决方法**（须能对应到具体函数；不写「是哪一篇日志」）。

---

## 1. 总体判断

早期版本机制叠加后，曾出现多模块争用控制权、恢复与规划职责重叠等问题。经多轮实车与代码收敛，当前主链路为：

- **状态机**决定状态与重规划时机；
- **局部规划器**只选目标，不发布连续速度；
- **前沿导航器**只跟目标，按前方净空连续限速；
- **恢复**在 RECOVERY 状态独占后退与转向，完成后交还状态机重规划。

当前主要瓶颈已不在「恢复参数够不够」，而在：

1. 边界选点仍可能导入死胡同或口袋区；
2. 开阔区域目标在侧后方时，车长时间蠕行而不主动换向；
3. 局部规划在贴墙或口袋区反复选相近失败方向；
4. 覆盖率长跑未稳定达标。

迭代原则（`重要原则.txt`）：参数集中于 `mission.yaml`；每次改动只解决一个问题；先记录日志再调参；深度相机仅作前向安全增强，不参与规划评分。

---

## 2. 问题归类

### 2.1 局部规划器无有效候选

| 现象 | 日志特征 |
|------|----------|
| 角落或贴墙处长时间不动 | 连续出现 `Selected: none (all rejected)` |
| 激光认为可通的方向仍被判无效 | 大偏角净空充足，候选仍被否决 |

**原因：** 栅格路径搜索与激光净空结论不一致时，若路径失败即否决候选，候选集可能为空。

**当前解决方法：** `local_planner._evaluate` 在激光净空满足时，路径搜索失败仍用单点目标 `[(gx, gy)]` 继续评分，避免一票否决。

```647:649:scripts/local_planner.py
    path = self.grid.plan_path(rx, ry, gx, gy)
    if not path:
      path = [(gx, gy)]
```

辅以失败记忆（`record_failure`）、贴墙小角度加罚（`_wall_hug_penalty`）与统一代价权重（`local_plan_score_*`）。

---

### 2.2 探索连续无候选

| 现象 | 说明 |
|------|------|
| 连续多帧无有效候选 | 车长时间停在原地或贴墙 |
| 仅依赖位移超时 | 反应慢，与墙角停滞处理不对称 |

**原因：** 探索态缺少「连续无候选即逃离重规划」的短路径判定。

**当前解决方法：** 状态机累计 `_explore_no_candidate_count`，达 `explore_no_candidate_frames`（默认 15）后调用 `_enter_planner_deadlock_recovery()`：清局部目标、设逃离标志、转入 `PLAN_NEXT` 重选边界点。

```546:556:scripts/search_fsm.py
  def _enter_planner_deadlock_recovery(self, scan: LaserScan) -> None:
    """连续无有效候选：标记逃离后直接重规划。"""
    ...
    self._prefer_escape_frontier = True
    self._set_state(SearchState.PLAN_NEXT, 'explore deadlock, replan')
```

---

### 2.3 边界恢复与恢复后重规划

| 现象 | 说明 |
|------|------|
| 撞墙后反复在同一方向再撞 | 恢复幅度不足或恢复后立即重选失败方向 |
| 恢复与规划职责重叠 | 恢复内嵌选点导致控制权不清 |

**原因：** 恢复幅度曾按边界类型打折；恢复成功后未对失败方向施加足够约束即重规划。

**当前解决方法：**

1. **统一恢复幅度**：`_escalation_back_turn()` 直接读取 `recovery_backoff_levels` / `recovery_turn_levels`（当前单级：后退 0.12 m、转角 45°）。
2. **边界恢复流程**：`_tick_boundary_recovery` 为暂停 → 复检 → 单次后退转向 → `_finish_recovery_resume()`。
3. **恢复后约束重规划**：`_replan_after_recovery()` 调用 `arm_post_recovery_replan()`，按 `recovery_replan_max_angle_deg`（70°）、`recovery_replan_max_dist_m`（3 m）、`recovery_replan_rounds`（3）过滤候选。
4. **方向惩罚**：`note_boundary_hit()` 记录上次选中偏角；`refresh_penalty_for_replan()` 在恢复后刷新惩罚窗口（`recent_collision_penalty_sec: 3.0`）。

```675:678:scripts/search_fsm.py
  def _escalation_back_turn(self, level: int) -> Tuple[float, float]:
    cfg = self.search
    idx = min(level, len(cfg.recovery_backoff_levels) - 1)
    return cfg.recovery_backoff_levels[idx], cfg.recovery_turn_levels[idx]
```

```285:294:scripts/local_planner.py
  def refresh_penalty_for_replan(self) -> None:
    """恢复后即将重规划：刷新计时，确保首次规划仍受方向惩罚。"""
    ...
    self._penalty_pending_replan = True
```

非边界恢复（`explore_stall`、`no_frontier` 等）走 `_tick_generic_recovery()`：`no_frontier` 为一次开阔侧转向后进 `PLAN_NEXT`；其余可为递增旋转，耗尽后短时 `EXPLORE`。

---

### 2.4 导航受阻与墙角停滞

| 现象 | 说明 |
|------|------|
| 跟目标时前方受阻 | 目标不可达或净空不足 |
| 墙角位移极小、转角大 | 长时间原地调整 |

**原因：** 导航器确认受阻后，若仅停车不重规划，会卡死；墙角需逃离选点而非反复同级恢复。

**当前解决方法：**

1. 前沿导航器连续多帧确认后上报 `blocked`。
2. 状态机 `_handle_nav_blocked()`：清目标、记失败、多数情况 `→ PLAN_NEXT`；前方仍堵时轻退 0.10 m 后再重规划。
3. `_update_corner_stall()` 在探索与跟目标两态均 `→ PLAN_NEXT`，并设 `_prefer_escape_frontier` 触发边界逃离选点。

```1641:1682:scripts/search_fsm.py
  def _handle_nav_blocked(
      self,
      gx: Optional[float],
      gy: Optional[float],
      scan: LaserScan,
  ) -> None:
    """导航受阻：前方仍通畅则只重规划，真挡路才轻后退。"""
    ...
    self._set_state(SearchState.PLAN_NEXT, reason)
```

---

### 2.5 模块争用（已收敛项与残留）

| 冲突 | 现状 |
|------|------|
| 规划器与前沿导航器 | 规划器不自动强制重规划；导航反馈只读 |
| 恢复与规划器 | RECOVERY 期间不调用局部规划器与前沿导航器；重规划由状态机在恢复完成后触发 |
| 多状态驱动规划 | 重规划入口收敛到状态机（`PLAN_NEXT`、受阻、逃离等） |

**残留：** `_handle_nav_blocked` 在前方仍堵时状态机直接轻退，与前沿导航器受阻逻辑仍有职责重叠，待收敛。

---

### 2.6 前沿导航：开阔侧不转向

| 现象 | 说明 |
|------|------|
| 目标在侧后方 | 车沿原航向长时间蠕行 |
| 限速偏晚 | 约 `center < 0.34 m` 才明显减速 |

**原因：** 前沿导航器按前方窄扇区 `center` 连续限速（`_simple_path_clear`），大偏角时虽带最低线速度转向，但未主动换全局目标。

**当前解决方法：** 统一一般避障：蠕行带速转向 + 连续帧确认后上报 `blocked`，由状态机重规划换点。参数：`nav_forward_stop_margin`、`nav_open_align_arc_speed`、`boundary_dist`。

**待优化：** 侧向开阔时更早触发重规划或放宽跟目标航向容差（`nav_drive_heading_*`）。

---

### 2.7 边界选点导入死区

| 现象 | 日志特征 |
|------|----------|
| 反复进同一口袋区 | `no reachable frontier`、同一区域多次 `Boundary hit` |
| 逃离后仍选近失败区 | `Frontier pick` 总代价仍偏低角近失败点 |

**原因：** 首条可达即停或仅按距离排序时，易反复选中不可行或已失败区域。

**当前解决方法：** `OccupancyGrid.nearest_frontier()` 对排序后前 N 个候选均做路径搜索，取总代价最低者；总代价含路径长度、狭窄惩罚（`_path_narrow_penalty`）、历史失败软惩罚（`goal_failure_cost`）。导航受阻或恢复时 `record_failure()` 写入失败记忆；`blocked` / 墙角停滞时 `_prefer_escape_frontier` 触发逃离排序。

配置：`frontier_eval_top_n`、`frontier_narrow_penalty_weight`、`frontier_failure_penalty_weight`、`failure_memory_*`。

---

### 2.8 激光盲区与低矮障碍

| 现象 | 说明 |
|------|------|
| 前方净空接近阈值仍擦碰 | 低矮障碍或相机支架外凸未进入窄扇区 |
| 与恢复链不同类 | 属测距盲区，非恢复幅度问题 |

**原因：** 激光窄扇区未覆盖车体前向外凸部分。

**当前对策：** 收紧 `nav_forward_stop_margin`、增大车体等效包络参数（`robot_radius`、`robot_front_overhang`）。按 `重要原则.txt`，深度相机后续仅补前向测距，不参与规划评分。

---

## 3. 误判与纠正

| 早期判断 | 纠正 |
|----------|------|
| 恢复参数过小是主因 | 更常见是规划先无可行候选，或恢复后立刻重选失败方向 |
| 宽缝撞障是航向增益问题 | 多为栅格否决候选；已通过路径失败回退与统一代价缓解 |
| 按边界类型差异化恢复幅度 | 统一读配置表，由 `_escalation_back_turn` 取值 |
| 连续两次小角度扣分促扫图 | 前方持续最优时应允许直行 |
| 深度参与规划评分 | 深度仅作安全增强，不参与选点评分 |
| 增加通道/角落独立模式 | 收敛为连续代价与一般避障，避免状态机膨胀 |

---

## 4. 当前版本对策（按模块）

配置均集中于 `config/mission.yaml`，经 `scripts/config_loader.py` 加载。

### 4.1 局部规划器

| 对策 | 代码要点 | 日志验证 |
|------|----------|----------|
| 路径失败仍可评分 | `_evaluate` 中 `plan_path` 失败用 `[(gx, gy)]` | `Selected: ±XX°`、分项日志 |
| 失败记忆 | `record_failure()`、`_failure_memory_penalty()` | `FailureMemory 记录` |
| 贴墙小角度加罚 | `_wall_hug_penalty()` | `wall=` 分项 |
| 恢复后多轮约束 | `arm_post_recovery_replan()`、`_filter_post_recovery_candidates()` | `恢复后第N轮 \|偏角\|<=` |
| 纯激光选路（可选） | `_pick_clearance_escape()`，`local_clearance_escape_enabled` 控制 | `纯激光选路` |

重规划仅由状态机触发；`tick()` 内有目标时只跟导航。

### 4.2 状态机与恢复

| 对策 | 代码要点 | 日志验证 |
|------|----------|----------|
| 恢复独占 | `_tick_recovery` 不调用规划器与导航器 | `FSM … -> RECOVERY` 后无跟目标日志 |
| 单级边界恢复 | `_tick_boundary_recovery` ACT 一次后退转向 | `unified level=0 back=… turn=…` |
| 恢复后重规划 | `_complete_recovery_resume` → `_replan_after_recovery` | `恢复后第N轮` |
| 无候选逃离 | `_enter_planner_deadlock_recovery` | `逃离重规划` |
| 受阻重规划 | `_handle_nav_blocked` | `Nav blocked … replan` |
| 墙角逃离选点 | `corner_stall` → `PLAN_NEXT` + `_prefer_escape_frontier` | `Corner stall … replan` |
| 无边界点开阔侧转向 | `_tick_generic_recovery`（`no_frontier`） | `open-side turn` |

### 4.3 前沿导航器

| 对策 | 代码要点 | 日志验证 |
|------|----------|----------|
| 按 center 连续限速 | `_simple_path_clear`、`_simple_drive_speed` | `NavFeedback: … creep/drive` |
| 受阻确认 | `_handle_center_blocked` 多帧后 `blocked` | `NavFeedback: … blocked` |
| 大偏角带速转向 | 线速度下限 `open_align_arc_speed` | 开阔区非零线速度转向 |

### 4.4 边界选点

| 对策 | 代码要点 | 日志验证 |
|------|----------|----------|
| 多候选总代价 | `nearest_frontier` + `frontier_eval_top_n` | `Frontier pick: … astar_eval=… total=…` |
| 狭窄与失败软惩罚 | `_path_narrow_penalty`、`goal_failure_cost` | `narrow=… failure=…` |
| 逃离排序 | `_prefer_escape_frontier` 传入选点 | `escape=True` |

---

### 5. 设计约束

| 事项 | 原因 |
|------|------|
| 不增加通道/角落等独立模式 | 避免状态机膨胀，行为难解释 |
| 恢复期间不调用规划器与导航器 | 保证速度发布单一路径 |
| 深度不参与规划评分 | 见 `重要原则.txt`；相机只作前向引导 |
| 每次改动只解决一个问题 | 便于日志归因与回退 |

---

## 6. 当前遗留问题

### 6.1 覆盖率长跑未稳定达标

**现象：** 覆盖率长期低于完成阈值（默认 92%），不能完全遍历场地。实车可沿一条主通道走一圈回到物理起点，也能进入两排物块后方，但运动呈单向、带状扩展：沿当前航向推进多，横向扫角少，物理四角与贴边未访问格长期留存。物理回到起点附近不等于 `RETURN_HOME` 链生效；任务仍以栅格 `visited / free` 为准。

**可能原因：**

1. **边界目标贪心沿已知通道延伸。** `nearest_frontier()` 总代价以路径长度为主，叠加航向代价与狭窄惩罚，缺少「远离已访问区质心」「靠近场地未探索角」一类项。车一旦拉出一条已访问带，侧向与对角边界点的路径代价偏高，规划倾向继续拉长当前通道，而非折向角落。

2. **导航器对侧后方目标不主动换向（见 §2.6）。** 目标落在大偏角时，前沿导航器仍以前方窄扇区 `center` 连续限速跟点，长时间蠕行而不上报 `blocked`；状态机迟迟不重选边界，横向格网推进慢，覆盖率爬升呈线性而非面状。

3. **局部探索态偏直行。** `EXPLORE` 下局部规划候选角以 `0°` 为首，实车日志常见连续 `记录边界候选偏角 0°`；前方净空充足时直行得分占优，墙角与侧向口袋需靠受阻、无候选或 `corner_stall` 才触发 `PLAN_NEXT`，缺乏主动扫角动机。

4. **口袋区与狭窄通道占用边界预算。** 物块间窄缝多次 `Boundary hit` 与恢复，均速约 0.08–0.11 m/s；时间在「当前走廊内脱困」与「重选邻近失败点」之间消耗，难以把位姿预算用于抵达场地远端或对角。

5. **逃离选点仅被动触发。** `_prefer_escape_frontier` 多在导航受阻、墙角停滞、连续无候选时置位，正常推进时仍优先前方可达边界。角落格往往不在「近失败区逃离」语义内，缺少周期性强制大角度或侧向边界点的机制。

6. **建图引导期默认关闭。** `bootstrap_local_plan: false`，起步即走全局边界探索，无短程宽扇区局部扫图冷启动；早期地图稀疏时，A* 与边界排序更易锁死在首条走廊（与 §6.2 起步走廊选择叠加时更明显）。

**与已完成对策的关系：** §2.7 多候选总代价、§2.2 无候选逃离、§2.4 墙角重规划已缓解「卡死」与「反复进同一口袋」，但未解决「主动趋向全场角落」与「面状覆盖」问题；控制权与恢复主链路不是当前主瓶颈。

**待尝试方向（每次只改一项，须先记日志）：**

| 方向 | 说明 | 涉及模块 |
|------|------|----------|
| 侧向更早重规划 | 目标偏角或侧向净空持续开阔时，由状态机提前 `PLAN_NEXT`，不等到导航 `blocked` | 前沿导航器、状态机 |
| 边界代价加探索奖励 | 在 `nearest_frontier` 总代价中增加「距已访问质心」「距地图边缘未探索区」项，削弱纯路径最短 | `occupancy_grid` |
| 局部大偏角加权 | `EXPLORE` 在开阔区提高大角度候选探索分，避免长期 0° 直行 | 局部规划器、`mission.yaml` |

**验收建议：** 以 60 s `运行统计` 中覆盖率曲线、均速、边界/恢复次数为主；辅以地图快照查看四角与贴边未访问格是否减少。不宜仅以「能否回到物理原点」判断任务完成。

---

### 6.2 固定障碍下起步朝向决定首条走廊（与覆盖率不同类）

**现象：** 场地物块位置固定时，同一段任务在不同起步 yaw 下行为分叉明显，且与最终覆盖率高低并非一一对应。朝向合适：能绕前排物块进入后方或侧向通道，后续探索有继续深入的可能。朝向不佳：可能只贴一侧绕过前排、始终进不了物块后方；或前排窄缝反复 `Boundary hit` 后在开阔地带绕圈，首段即耗尽在起点附近。两类结果都可能是「覆盖率低」，但根因是**首条可行走廊被起步姿态锁定**，而非车已深入场地后的扫图策略不足。

**原因：**

1. **首帧边界选点强依赖当前航向。** `nearest_frontier()` 在非逃离态下 `prefer_forward=True`：候选先按航向偏差排序，并丢弃超出 `forward_max_deg`（约 110°）的边界点。起步时地图几乎空白，排序与过滤实质上在「当前朝向扇区」内选最近边界，第一条全局路径即定下后续数分钟的推进走廊。

2. **早期地图稀疏放大路径偏差。** 物块刚被观测到时，栅格与 A* 对窄缝宽度、可通行性的估计不稳定；同一物理缝在不同 yaw 下可能一次通过、一次判为不可达，失败记忆与恢复后又因朝向约束重选相近方向，开阔区形成绕圈。

3. **缺少起步朝向无关的冷启动。** `bootstrap_local_plan` 默认关闭，无「先宽扇区局部试探再切边界」的缓冲；人工摆放误差直接传入 `PLAN_NEXT` 的第一次 `GlobalPlanner` / `nearest_frontier` 调用。

**与 §6.1 的区分：** §6.1 关注车**已进入某条走廊之后**如何面状扩展、扫角与提覆盖率；本节关注**任务前几十秒**能否选对入口。修复 §6.2 不保证覆盖率达标，但可减少「同一赛道、换摆放角度就完全进不了后方」的不可复现性。

**待尝试方向（每次只改一项，须先记日志）：**

| 方向 | 说明 | 涉及模块 |
|------|------|----------|
| 起步若干轮放宽前方过滤 | 前 N 次 `PLAN_NEXT` 设 `prefer_forward=False`，或临时提高 `forward_max_deg`，避免首点锁死在当前朝向 | 状态机、边界选点 |
| 短程建图引导 | 启用 `bootstrap_local_plan`，在 `visited` 未达阈值前走 `EXPLORE` 宽扇区，再切边界探索 | 状态机 |
| 受控绕场冷启动 | 短距离 `perimeter` 或固定角速度扫视，再进入 `frontier` 模式 | 全局规划器、状态机 |
| 首段失败即强制逃离 | 起步后连续 K 次边界恢复仍困在起点邻域时，置 `_prefer_escape_frontier` 并放宽恢复后重规划偏角 | 状态机、局部规划器 |

**验收建议：** 固定物块布局，记录至少 3 组起步 yaw（如 0°、±45°、±90°），对比首 60 s 内是否进入物块后方、开阔绕圈次数及首条 `Frontier pick` 目标方位；不以单次长跑覆盖率为唯一指标。

---

### 6.3 其他残留

| 项 | 说明 |
|----|------|
| `_handle_nav_blocked` 与导航器职责重叠 | 前方仍堵时状态机直接轻退，与导航受阻确认链部分重复（§2.5） |
| 激光物理包络 | 前向外凸未完全进入窄扇区，贴障时偶发擦边（§2.8） |
| 返航与完成条件 | 覆盖率未达标时不进入 `RETURN_HOME`；需与课程评分标准对齐预期 |