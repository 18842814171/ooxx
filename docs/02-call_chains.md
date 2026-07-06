# 02 — 关键调用链

> **重要原则**
>
> - 读链时始终问：**这一行有没有 publish_twist / move_distance / rotate_angle？**
> - 若一条链路上出现 **两个以上** 速度发布点，即控制权冲突，优先标记收敛，不修参数。
> - 状态机 `tick()` 是单线程同步模型：阻塞式 `move_*` 会占满多个循环。

> **当前构建**：`FSM_BUILD_ID = 20260705-p1-recovery`（`scripts/search_fsm.py:33`）

---

## 1. 主循环

```
ooxx_node.run()  [20Hz]
  └── fsm.tick()
        ├── grid.update_scan / mark_robot
        └── handlers[state]()    ← 按状态分发，无脱离窗口抢先发速
```

**文件**：`scripts/ooxx_node.py`，`scripts/search_fsm.py:238`

---

## 2. EXPLORE 链（occupancy_grid 模式）

### 2.1 正常探索（当前）

```
_tick_explore(scan)
  ├── [无目标 且 visited >= bootstrap_visited]
  │     └── state = PLAN_NEXT
  │
  ├── _check_explore_timeout() ──true──► _enter_recovery('explore_stall')
  │
  ├── _is_boundary_confirmed(scan) ──true──► _on_boundary(scan)
  │
  └── local_planner.tick(scan, move, cruise_speed)
        ├── [有目标]
        │     ├── frontier_nav.tick(scan, cruise_speed)
        │     │     └── move.publish_twist(cmd_v, w)     ← 速度发布 ①
        │     ├── execution_feedback()                    ← 只读，不触发自动重规划
        │     └── return status
        │
        ├── status == 'hold'
        │     └── _track_nav_hold(status)                 ← 状态机累计
        │           └── 达 limit → _on_boundary()       ← 进 Recovery，非规划器改目标
        │
        ├── status == 'blocked' → _on_boundary()
        ├── status == 'no_candidate' → _enter_planner_deadlock_recovery
        └── status == 'reached' → _reset_explore_timer()
```

**hold 仲裁**（P0，`search_fsm.py:541`）：

```
frontier_nav.tick() → status='hold'
  └── _track_nav_hold()
        ├── _nav_hold_streak += 1
        └── streak >= limit（默认 30 tick）
              └── _on_boundary(scan) → RECOVERY
```

**与 P0 前对比**：规划器 `tick()` **不再**调用 `_maybe_replan_on_exec_cost()`；无目标时**不再**调用 `reactive_drive()`。

---

### 2.2 建图引导阶段（bootstrap）

当 `bootstrap_local_plan: true` 且 `visited < bootstrap_visited`（默认 40）：

- 状态机**停留** EXPLORE，不进入 `PLAN_NEXT`。
- 仅靠 `local_planner.replan_local()` 选局部方向。
- 10-24 日志：车在左下角墙角反复 Recovery，覆盖率 60s 仅 6%。

此行为是**设计使然**，但在物理角落易形成「局部规划 ↔ 恢复」短循环。P2 拟抑制 0° 贴墙；或考虑提前切换前沿规划。

---

## 3. RECOVERY 链（边界恢复）

### 3.1 进入

```
_on_boundary(scan)  或  _enter_recovery(reason)
  ├── frontier_nav.clear_goal()
  ├── local_planner.note_boundary_hit()
  └── _start_boundary_recovery(scan, kind)
        ├── _snapshot_recovery_entry(scan)    ← P1：入口基线
        ├── _recovery_phase = PAUSE
        └── state = RECOVERY
```

### 3.2 恢复循环

```
_tick_recovery(scan)                    ← P0：本状态不 tick 规划器/导航器
  └── _tick_boundary_recovery(scan)
        ├── PAUSE: stop_robot
        ├── RECHECK: 窄通道可通行? → _finish_recovery_resume
        └── ACT:
              ├── _snapshot_recovery_action_start()   ← 仅日志位移
              ├── _execute_boundary_action()
              │     ├── move_distance_x(-back)        ← 阻塞 ①
              │     └── rotate_angle(turn)            ← 阻塞 ②
              ├── _recovery_success(scan)?
              │     └── center - entry >= 0.05        ← P1 入口基线
              └── level2 重试（最多 2 次）/ 强制结束
```

**文件**：`scripts/search_fsm.py:471-531`，`scripts/search_fsm.py:720-784`

---

## 4. Recovery 退出链（当前）

```
_finish_recovery_resume(scan)
  └── _try_escape_or_replan(scan)       ← P0：无脱离窗口直连
        └── _complete_recovery_resume(scan)
              ├── refresh_penalty_for_replan()
              ├── _replan_after_recovery(scan)         ← P1
              │     ├── arm_post_recovery_replan(35°, 2m)
              │     ├── _filter_post_recovery_candidates()
              │     └── replan_local(force=True)
              └── state = EXPLORE | MOVE_TO_GOAL | PLAN_NEXT

后续 EXPLORE tick:
  └── local_planner.tick → navigator.tick
```

**P1 日志样例**（10-24）：

```
Recovery(boundary_front_wall): 入口基线 center=0.32
Recovery(boundary_front_wall): 判定成功 level=1 center=0.42 entry=0.32 gain=0.10
LocalPlanner: 恢复后首轮 |偏角|<=35° d<=2.00m 候选 9→5
Selected: 0° goal=(1.86,1.48) d=0.84
```

---

## 5. MOVE_TO_GOAL 链

```
_tick_move_to_goal(scan)
  ├── _update_corner_stall → 可能 _enter_recovery('corner_stall')
  └── local_planner.tick(...)
        ├── _track_nav_hold / blocked → _on_boundary 或 _handle_nav_blocked
        └── navigator.tick → publish_twist
```

**遗留**：`_handle_nav_blocked` 中状态机直接 `move_distance_x(-backoff)` 后退，与导航器 blocked 逻辑并存（未收敛）。

---

## 6. PLAN_NEXT 链

```
_tick_plan_next(scan)
  ├── grid.nearest_frontier() → goal, path
  ├── frontier_nav.set_path(path, goal)
  └── state = MOVE_TO_GOAL | EXPLORE（bootstrap 未满时）
```

本状态不发速度。速度发布在下一状态导航器链。

---

## 7. RETURN_HOME 链

```
_is_search_complete() ──true──► _begin_return_home()
  ├── 条件：任务完成 | 覆盖率 ≥ 92% | 边界事件 ≥ 80 | 牛耕完成
  └── state = RETURN_HOME
        └── _tick_return_home → navigator.tick 或角速度对齐
              └── 到位 → DONE
```

10-24 日志**未触发**此链；车回到起点附近是墙角螺旋的物理结果，非正式返航。

---

## 8. 控制权状态一览（当前）

| 调用链 | 速度发布者 | P0/P1 后状态 |
|--------|-----------|-------------|
| EXPLORE → navigator.tick | 导航器 | 正常 |
| hold → _track_nav_hold → Recovery | 状态机仲裁 | 已收敛 |
| RECOVERY → move_distance_x | Recovery | 独占，仍阻塞 |
| 脱离窗口直连 | — | **已删除** |
| 规划器 exec_cost 强制重规划 | — | **已禁用** |
| reactive_drive | — | **已禁用** |
| _handle_nav_blocked 后退 | 状态机直连 | 仍存 |

---

## 9. 历史链（P0 前，仅供对照）

<details>
<summary>展开：hold → replan 链（9-17 根因，已修复）</summary>

```
frontier_nav.tick() → hold → exec_cost=0.75
local_planner._maybe_replan_on_exec_cost() → replan_local(force=True)
下一 tick：跟新目标再撞墙
```

</details>

<details>
<summary>展开：脱离窗口链（已删除）</summary>

```
_finish_recovery_resume → _start_escape_forward()
_tick_escape_forward() → publish_twist(0.08, 0)  绕过导航器
```

</details>

---

## 10. 调试：如何读日志

| 日志关键词 | 所在链 | 含义 |
|-----------|--------|------|
| `FSM init: build=` | 启动 | 版本确认 |
| `Recovery: 入口基线 center=` | P1 恢复入口 | 增益基线 |
| `判定成功 … gain=` | P1 恢复成功 | 入口基线判定 |
| `恢复后首轮 \|偏角\|<=` | P1 恢复后规划 | 首轮约束生效 |
| `FSM: Navigator hold N ticks` | hold 仲裁 | 即将进 Recovery |
| `NavFeedback: … reason=center` | 导航器 hold | 前方窄扇区不足 |
| `NavFeedback: … stress=` | 导航器应力 | 贴边累积，P3 重点 |
| `Boundary hit` | _on_boundary | 进 Recovery |
| `FSM EXPLORE -> RECOVERY` | 状态交接 | 规划器/导航器应停止 |
| `exec_cost=… replan` | 规划器自动重规划 | **不应再出现** |
| `脱离窗口` | 旧脱离链 | **不应再出现** |
| `运行统计(60s):` | 统计 | 边界/恢复/覆盖率 |

**运行日志保存**：`rosrun ooxx ooxx_logrun.sh roslaunch ooxx chassis_search.launch` → `log/H-M.txt`（见 `使用方法.txt`）。

---

## 11. 目标调用链（P2/P3 重构方向）

### 探索 / 导航

```
状态机(EXPLORE) → 规划器.replan?（仅状态机许可时）
               → 导航器.tick → cmd_vel
```

### 恢复

```
状态机(RECOVERY) → Recovery.step（独占 cmd_vel）
               → 完成 → arm_post_recovery_replan → replan
               → 状态机(EXPLORE) → 导航器.tick → cmd_vel
```

**禁止**（已落实）：恢复期间规划器/导航器 tick；脱离窗口直连；hold 期间规划器 force replan。

**待做**（P2/P3）：贴墙 0° 抑制；开阔侧向换向；车体包络挡障。
