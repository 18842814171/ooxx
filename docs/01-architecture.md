# 01 — 系统架构与控制权

> 本文遵守以下规定：
>
> **所有模块均可提出「建议」，只有状态机可以做「决策」，只有当前控制权拥有者可以发布非零速度。**
>
> 1. **任何时刻只有一个模块有权向 `/controller/cmd_vel` 发布非零速度。**
> 2. **状态机状态决定控制权归属，而不是各子模块自行判断。**
> 3. **局部规划器只选目标，前沿导航器只跟目标，恢复动作只做脱困——三者不同时「开车」。**
> 4. **课程实验目标是稳定、能完成任务、易调参；不是恢复策略越复杂越好。**
> 5. **若一个动作跨越两个以上模块，优先收敛职责，而不是继续打补丁。**

---

## 1. 分层总览

```
┌─────────────────────────────────────────────────────────────┐
│  ooxx_node.py          20Hz 主循环                           │
│    └── SearchFSM.tick()   ← 唯一调度入口                      │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│  SearchFSM（状态机）                                            │
│    决定：当前处于 Explore / Recovery / Plan / …               │
│    决定：每一帧谁可以调用 MoveController                      │
└─────────────────────────────────────────────────────────────┘
         │
    ┌────┴────┬──────────────┬─────────────┐
    ▼         ▼              ▼             ▼
 LocalPlanner  FrontierNavigator  Recovery动作  其他
 （选目标）     （跟目标发速度）   （阻塞脱困）   （返航等）
    │              │                │
    └──────────────┴────────────────┘
                   ▼
         MoveController → /controller/cmd_vel
```

全局规划器启用且初始模式为绕场时，状态机另有 `GLOBAL_PLAN` 状态；当前默认配置为边界探索，初始状态为 `PLAN_NEXT`。

---

## 2. 开发阶段

| 阶段 | 状态 | 要点 |
|------|------|------|
| P0 控制权收敛 | 已完成 | 恢复期间不调度规划器与导航器；重规划由状态机触发 |
| P1 恢复收尾 | 已完成 | 恢复入口记录基线；恢复后多轮规划约束；单级边界恢复 |
| P2 局部规划器 | 已编码 | 评分权重、失败记忆、贴墙小角度加罚、引导期退出 |
| P3 导航器 | 已编码 | 统一一般避障；按前方净空连续限速与带速转向 |
| P3b 边界逃离 | 已编码 | 被困时逃离选点、无边界点开阔侧转向、墙角停滞重规划 |
| 边界多候选评分 | 已编码 | 多候选总代价选点；狭窄与历史失败软惩罚 |


---

## 3. 目标控制权模型（当前实现）

### 3.1 正常探索 / 导航

```
PLAN_NEXT / EXPLORE / MOVE_TO_GOAL
        │
        ▼
   LocalPlanner.tick()
        │
        ├─ 无目标 → replan_local() 选目标 → nav.set_path()
        │
        └─ 有目标 → FrontierNavigator.tick()  ← 唯一速度发布者
                        │
                        ▼
                   publish_twist()
```

- 局部规划器每帧被调用时只跟目标导航；重规划由状态机在状态切换或受阻时触发。
- 导航器返回 `drive` / `creep` / `blocked`。前方窄扇区不足时以蠕行带速转向为主，连续多帧确认后才上报 `blocked`。
- 导航 `blocked` 时，状态机 `_handle_nav_blocked()` 清目标、记失败，多数情况转入 `PLAN_NEXT` 重规划；仅前方仍不通时轻退 0.10 m 后再重规划。不由规划器在导航过程中擅自换目标。
- 探索连续无有效候选达阈值时，`_enter_planner_deadlock_recovery()` 转入 `PLAN_NEXT` 逃离重规划。
- 墙角停滞（`corner_stall`）在探索与跟目标两态均转入 `PLAN_NEXT` 重规划。

### 3.2 恢复独占

```
RECOVERY 开始
        │
        ▼
   frontier_nav.clear_goal()     ← 导航器失效
   _tick_recovery(scan)         ← 仅此运行；不调用局部规划器与前沿导航器
        │
        ▼
   Recovery 状态机独占
   move_distance_x / rotate_angle / stop_robot
        │
        ▼
   Recovery 结束 → _try_escape_or_replan()
        │
        ▼
   _complete_recovery_resume()
        ├─ arm_post_recovery_replan（见 mission.yaml：|偏角|、距离、轮数）
        ├─ replan_local(force=True)（按上下文）
        └─ 交还 EXPLORE / MOVE_TO_GOAL / PLAN_NEXT
        │
        ▼
   Navigator + Planner 恢复
```

- 边界恢复（`_tick_boundary_recovery`）：暂停 → 复检 → 单次后退与转向（当前配置后退 0.12 m、转角 45°），然后 `_finish_recovery_resume()`。
- `_snapshot_recovery_entry()` 在入口记录窄扇区中心距离，供日志参考。
- 非边界恢复（`explore_stall`、`no_frontier` 等）走 `_tick_generic_recovery()`：`no_frontier` 为一次开阔侧转向后重规划；其余原因可为递增旋转脱困。

### 3.3 状态 → 控制权对照表

| 状态机状态 | 速度发布者 | 局部规划器 | 前沿导航器 | 备注 |
|----------|-----------|---------|-----------|------|
| PLAN_NEXT | 无（停车） | 选全局前沿点 | 清空或待设 | 仅规划，不前进；全局规划启用时为常见入口 |
| EXPLORE | 导航器（经局部规划器） | 状态机许可时重规划 | 跟目标 | 无候选 → 逃离重规划至 PLAN_NEXT |
| MOVE_TO_GOAL | 导航器 | 同左 | 跟目标 | blocked → 重规划；corner_stall → PLAN_NEXT |
| RECOVERY | **仅 Recovery 动作** | **不调用** | **不调用** | 阻塞式 move_* |
| GLOBAL_PLAN | 无或绕场导航 | 全局规划器 | 绕场时跟航点 | 仅 `perimeter_enabled: true` 时进入 |
| RETURN_HOME | 导航器或状态机角速度 | 不调用 | 跟返航点 | 覆盖率或边界预算等条件触发 |
| DONE | stop_robot | 不调用 | 不调用 | |

---

## 3A. 全局模式与目标选取

### 运行模式

配置见 `config/mission.yaml` 之 `global_planner` 段：

| 配置项 | 当前值 | 含义 |
|--------|--------|------|
| `enabled` | 真 | 启用全局规划入口 |
| `initial_mode` | `frontier` | 名义模式为边界探索 |
| `perimeter_enabled` | 假 | 绕场不启用；默认走边界探索 |

无边界点时的开阔侧转向在 `SearchFSM._tick_generic_recovery` 之 `no_frontier` 分支完成，不切换全局模式。

### 任务结束

目标识别已关闭。结束依据：覆盖率阈值（默认 92%）、边界事件预算、返航配置。

### 边界探索选点

`PLAN_NEXT` 调用 `OccupancyGrid.nearest_frontier()`：

1. 找边界点（已知自由格邻未知格）。
2. 过滤拉黑区与过近候选。
3. **排序**：正常时偏角小者优先；逃离时按距失败区最远排序（不限前方扇区）。
4. **多候选评分**（`frontier_eval_top_n`，默认 10）：对排序后前 N 个候选均做路径搜索，取**总代价最低**者。
5. **总代价** = 路径长度 + `frontier_narrow_penalty_weight` × 路径狭窄代价 + `frontier_failure_penalty_weight` × 历史失败软惩罚（`LocalPlanner.goal_failure_cost`）+ 偏角代价 − 逃离奖励。

狭窄代价：路径沿途占用格 5×5 邻域统计（`_path_narrow_penalty`）。失败软惩罚：导航受阻或恢复时 `record_failure` 写入，仅抬价、非硬禁入。

配置见 `config/mission.yaml` 之 `map.frontier_*` 与 `scripts/config_loader.py` 之 `MapConfig`。

触发逃离选点：导航 `blocked`、墙角停滞、无边界点首次重试（`SearchFSM._plan_frontier_goal`）。

**实车日志示例**：`Frontier pick: … astar_eval=N total=… narrow=… failure=…`

### 被困后续

1. 逃离选点仍失败 → 一次开阔侧转向（`boundary_corner_turn_deg`）后重规划。
2. 仍无边界点 → `EXPLORE` 局部规划选方向。

### 局部导航

前沿导航器每帧被调用时：朝路径点转向，按前方窄扇区净空 `center` 连续限速（`_simple_path_clear`）。大偏角时带最低线速度转向，避免原地纯旋转。

### 建图引导期（可选）

`bootstrap_local_plan` 当前为假（由全局边界规划替代）。若为真且 `visited < bootstrap_visited`（40），状态机在 `PLAN_NEXT` 与 `EXPLORE` 间走局部引导；退出条件含访问格满、超时低覆盖、边界过多等，由 `_maybe_exit_bootstrap()` 一次性切至 `PLAN_NEXT`。

---

## 4. 已知行为瓶颈（非控制权类）

以下现象来自实车与代码对照，供调参与后续迭代参考：

1. **导航器停车与转向**：`center < boundary_dist + nav_forward_stop_margin`（默认约 0.34 m）才明显限速；开阔区域目标在侧后方时，车可能长时间沿原航向蠕行而不主动换向。
2. **物理包络不足**：激光窄扇区未覆盖相机支架等前向外凸，贴障碍时可能擦边。
3. **回原点**：覆盖率未达标时通常不出现 `RETURN_HOME`；物理上回到起点附近不等于任务完成返航。
4. **边界选点**：失败区拉黑与逃离选点仍可能反复导入死胡同或口袋区，需结合失败记忆与局部评分继续优化。
5. **纯激光选路**：`local_clearance_escape_enabled` 为假时，前方受阻仅走地图评分选路；为真时可不经地图评分直接选最开阔方向。

---

## 5. 模块职责边界

### SearchFSM
- **拥有**：状态转换、Recovery 进出、边界事件计数、地图更新、导航受阻与逃离重规划决策、恢复后重规划决策。
- **不拥有**：连续速度控制律（除返航角速度、受阻轻退等少数例外）。
- **相关接口**：`_replan_after_recovery()`、`_snapshot_recovery_entry()`、`_handle_nav_blocked()`。

### LocalPlanner
- **拥有**：局部候选评分、选目标与路径、`nav.set_path()`；恢复后多轮候选过滤。
- **不拥有**：每帧连续速度。
- **相关接口**：`arm_post_recovery_replan()`、`_filter_post_recovery_candidates()`、`record_failure()`。

### FrontierNavigator
- **拥有**：有目标时的 `cmd_vel`（`drive` / `creep`；受阻确认后为 `blocked`）。
- **不拥有**：换目标、Recovery、全局前沿选择。
- **输出**：`NavExecutionFeedback`（建议状态），不是状态机命令。

### MoveController
- **唯一硬件出口**：所有 `cmd_vel` 经此发布。
- **流式**：`publish_twist`（每帧，导航器用）
- **阻塞式**：`move_distance_x` / `rotate_angle`（Recovery 与受阻轻退用，占满多帧）

---

## 6. 与课程目标的对齐

| 目标 | 当前 | 下一瓶颈 |
|------|------|----------|
| 控制稳定 | 导航态单发布者为主 | 收敛 `_handle_nav_blocked` 与导航器职责 |
| 恢复可用 | 单级边界恢复 + 重规划；非边界有通用旋转 | 物理脱困幅度与死胡同逃离 |
| 完成任务 | 覆盖率长跑未稳定达标 | 边界选点、局部方向评分、口袋区 |
| 易调参 | 控制权与 mission.yaml 分层 | 导航净空阈值、规划权重、包络参数 |

**一句话**：控制权与恢复主链路已收敛；任务行为瓶颈在边界选点导入死区、导航侧向开阔时不换向、以及局部规划在口袋区的方向评分。
