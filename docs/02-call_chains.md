# 02 — 关键调用链

> 本文遵守以下规则
>
> - 读链时始终问：**这一行有没有 publish_twist / move_distance / rotate_angle？**
> - 若一条链路上出现 **两个以上** 速度发布点，即控制权冲突，优先标记收敛，不修参数。
> - 状态机每帧调度为单线程同步模型：阻塞式 `move_*` 会占满多个主循环周期。

---

## 1. 主循环

```
ooxx_node.run()  [20Hz]
  └── fsm.tick()
        ├── 激光更新栅格地图（可通行 / 障碍）
        ├── 标记当前格为已访问
        ├── 更新覆盖率
        └── handlers[state]()    ← 按状态分发
```

全局规划器启用且非绕场模式时，初始状态为 `PLAN_NEXT`；绕场启用时为 `GLOBAL_PLAN`。

建图与障碍标记说明见 `docs/03-module_index.md` 第 4.1 节。

**文件**：`scripts/ooxx_node.py`，`scripts/search_fsm.py`，`scripts/occupancy_grid.py`

---

## 2. PLAN_NEXT 链（边界选点）

```
_tick_plan_next()
  ├── [bootstrap_local_plan 为真 且 visited < bootstrap_visited 且未退出引导]
  │     └── state = EXPLORE（建图引导）
  │
  ├── _plan_frontier_goal() → grid.nearest_frontier()
  │     ├── goal 为空 → _handle_no_frontier()
  │     │     ├── 可逃离重试 → 再规划
  │     │     ├── 首次 → 开阔侧转向 → RECOVERY(no_frontier)
  │     │     └── 否则 → EXPLORE 局部探索
  │     └── goal 有效 → _apply_frontier_plan()
  │           ├── frontier_nav.set_path(path, goal)
  │           └── state = MOVE_TO_GOAL
  │
  └── 本状态不发速度
```

当前默认配置 `bootstrap_local_plan: false`，通常直接走边界选点分支。

---

## 3. EXPLORE 链（occupancy_grid 模式）

### 3.1 正常探索

```
_tick_explore(scan)
  ├── [无目标 且 visited >= bootstrap_visited]
  │     └── state = PLAN_NEXT
  │
  ├── _check_explore_timeout() ──true──► _enter_recovery('explore_stall')
  │
  ├── _update_corner_stall() ──true──► PLAN_NEXT（逃离重规划）
  │
  ├── _is_boundary_confirmed(scan) ──true──► _on_boundary(scan) → RECOVERY
  │
  └── local_planner.tick(scan, move, cruise_speed)
        ├── [有目标]
        │     ├── frontier_nav.tick(scan, cruise_speed)
        │     │     └── move.publish_twist(cmd_v, w)     ← 速度发布 ①
        │     ├── execution_feedback()                    ← 只读
        │     └── return status（drive / creep / blocked 等）
        │
        ├── status == 'blocked'
        │     └── _handle_nav_blocked() → PLAN_NEXT（多数情况；前方仍堵则轻退 0.10 m）
        │
        ├── status == 'no_candidate'
        │     └── 连续达阈值 → _enter_planner_deadlock_recovery() → PLAN_NEXT
        │
        └── status == 'reached' → _reset_explore_timer()
```

---

### 3.2 建图引导阶段（可选）

仅当 `bootstrap_local_plan: true` 且 `visited < bootstrap_visited`（默认 40）且 `_bootstrap_exited` 为假：

- `PLAN_NEXT` 转入 `EXPLORE`，仅靠 `local_planner.replan_local()` 选局部方向。
- 满足超时、低覆盖、边界过多等条件时，`_maybe_exit_bootstrap()` 一次性切回 `PLAN_NEXT`。

当前默认配置下该分支不启用。

---

## 4. MOVE_TO_GOAL 链

```
_tick_move_to_goal(scan)
  ├── _is_boundary_confirmed(scan) ──true──► _on_boundary(scan)
  │
  ├── _update_corner_stall() ──true──► PLAN_NEXT（逃离重规划）
  │
  └── local_planner.tick(...)
        ├── frontier_nav.tick → publish_twist          ← 速度发布 ①
        ├── status == 'reached' → PLAN_NEXT 或 GLOBAL_PLAN（按模式）
        ├── status == 'blocked' → _handle_nav_blocked() → PLAN_NEXT
        └── status == 'odom_fault' → _enter_recovery('odom_fault')
```

`_handle_nav_blocked` 在前方仍不通时，状态机会 `move_distance_x(-0.10)` 轻退后再重规划。

---

## 5. RECOVERY 链

### 5.1 进入

```
_on_boundary(scan)  或  _enter_recovery(reason)
  ├── frontier_nav.clear_goal()
  ├── local_planner.note_boundary_hit()（边界路径）
  └── 边界：_start_boundary_recovery(scan, kind)
        ├── _snapshot_recovery_entry(scan)    ← 入口基线（日志）
        ├── _recovery_phase = PAUSE
        └── state = RECOVERY

  非边界（explore_stall、odom_fault 等）：
        ├── _recovery_phase = None
        └── state = RECOVERY → _tick_generic_recovery()
```

### 5.2 边界恢复循环

```
_tick_recovery(scan)
  └── _tick_boundary_recovery(scan)     ← 不调用局部规划器与前沿导航器
        ├── PAUSE: stop_robot
        ├── RECHECK: 窄扇区可通行? → _finish_recovery_resume
        └── ACT:
              ├── _snapshot_recovery_action_start()   ← 仅日志位移
              ├── _execute_boundary_action()
              │     ├── move_distance_x(-back)        ← 阻塞 ①（当前 back=0.12 m）
              │     └── rotate_angle(turn)            ← 阻塞 ②（当前 turn=45°）
              └── _finish_recovery_resume()
```

### 5.3 通用恢复（非边界）

```
_tick_generic_recovery(scan)
  ├── reason == 'no_frontier'
  │     └── rotate_angle(开阔侧) → PLAN_NEXT
  │
  └── 其他（explore_stall 等）
        ├── rotate_angle（递增步进，最多 _max_recovery_attempts 次）
        └── 耗尽 → EXPLORE（recovery exhausted）
```

**文件**：`scripts/search_fsm.py` 中 `_tick_recovery`、`_tick_boundary_recovery`、`_tick_generic_recovery`、`_complete_recovery_resume`

---

## 6. Recovery 退出链

```
_finish_recovery_resume(scan)
  └── _try_escape_or_replan(scan)
        └── _complete_recovery_resume(scan)
              ├── refresh_penalty_for_replan()
              ├── _replan_after_recovery(scan)（按上下文）
              │     ├── arm_post_recovery_replan（mission.yaml：|偏角|≤70°、d≤3 m、3 轮）
              │     └── replan_local(force=True)
              └── state = EXPLORE | MOVE_TO_GOAL | PLAN_NEXT（按上下文）

后续各帧（若进入跟目标态）：
  └── 调用局部规划器 → 调用前沿导航器
```

**日志样例**：

```
Recovery(boundary_front_wall): 入口基线 center=0.32
Recovery(boundary_front_wall): unified level=0 back=0.12m turn=45deg
LocalPlanner: 恢复后第1轮 |偏角|<=70° d<=3.00m 候选 …
Frontier pick: … astar_eval=N total=… narrow=… failure=…
```

---

## 7. RETURN_HOME 链

```
_is_search_complete() ──true──► _begin_return_home()
  ├── 条件：覆盖率 ≥ 92% | 边界事件 ≥ 80 | 牛耕完成 等
  └── state = RETURN_HOME
        └── _tick_return_home → navigator.tick 或角速度对齐
              └── 到位 → DONE
```

目标识别已关闭；覆盖率未达标时通常不触发。物理回到起点附近不等于本链生效。

---

## 8. 控制权状态一览

| 调用链 | 速度发布者 | 说明 |
|--------|-----------|------|
| PLAN_NEXT / EXPLORE / MOVE_TO_GOAL → 前沿导航器 | 导航器 | 正常跟目标 |
| EXPLORE/MOVE blocked → _handle_nav_blocked | 导航器为主；仍堵时状态机轻退 | → PLAN_NEXT 重规划 |
| corner_stall / 无候选 / 逃离重规划 | — | → PLAN_NEXT |
| RECOVERY → move_distance_x / rotate_angle | Recovery | 独占，阻塞式 |
| _handle_nav_blocked 轻退 | 状态机直连 | 前方仍堵时后退 0.10 m |

---

## 9. 调试：如何读日志

| 日志关键词 | 所在链 | 含义 |
|-----------|--------|------|
| `FSM init:` | 启动 | 模式、初始状态、全局规划是否启用 |
| `Recovery: 入口基线 center=` | 边界恢复入口 | 窄扇区基线 |
| `unified level=0 back=… turn=…` | 边界恢复动作 | 单次后退与转向 |
| `恢复后第N轮 \|偏角\|<=` | 恢复后规划 | 多轮约束生效 |
| `Frontier pick:` | PLAN_NEXT 选点 | 多候选评分与 A* 评估数 |
| `Nav blocked … replan` | blocked 链 | 清目标后重规划 |
| `探索：连续 N 帧无有效候选，逃离重规划` | 无候选链 | → PLAN_NEXT |
| `Corner stall … replan` | 墙角停滞 | → PLAN_NEXT |
| `NavFeedback: … reason=` | 导航器反馈 | creep / drive / blocked 等 |
| `Boundary hit` | _on_boundary | 进 RECOVERY |
| `Recovery(no_frontier): open-side turn` | 无边界点 | 一次开阔侧转向 |
| `运行统计(60s):` | 统计 | 边界/恢复/角落停滞/覆盖率 |

**运行日志保存**：

```bash
rosrun ooxx ooxx_logrun.sh roslaunch ooxx chassis_search.launch
```

→ 自动保存到 `log/M-D.txt` 或带标签的 `log/M-D-标签.txt`（见 `使用方法.txt`）。

---

## 10. 目标调用链（应然）

### 探索 / 导航

```
状态机(PLAN_NEXT) → 边界选点 → MOVE_TO_GOAL
状态机(EXPLORE)   → 规划器.replan?（仅状态机许可时）
                 → 前沿导航器 → cmd_vel
受阻 / 停滞 / 无候选 → PLAN_NEXT 重规划（逃离选点）
```

### 恢复

```
状态机(RECOVERY) → Recovery 动作（独占 cmd_vel）
               → 完成 → arm_post_recovery_replan → replan（按上下文）
               → 状态机(PLAN_NEXT | EXPLORE | MOVE_TO_GOAL)
```

**当前约束**：恢复期间不调用局部规划器与前沿导航器；重规划由状态机触发。

**待做**：阻塞式 Recovery 改限时或非阻塞；收敛 `_handle_nav_blocked` 与导航器职责。
