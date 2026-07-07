# 03 — 模块索引

> **重要原则**
>
> - 本表标注每个文件的**应有职责**与**现状**；改代码前先查「谁有权发速度」。
> - 配置优先改 `config/mission.yaml`；**控制权冲突不能靠调参解决**。
> - 构建标识：`search_fsm.py` 顶部 `FSM_BUILD_ID`，实车日志应核对。

---

## 1. 入口与调度

| 文件 | 职责 | 发布 cmd_vel? | 备注 |
|------|------|---------------|------|
| `scripts/ooxx_node.py` | 节点主循环、订阅 `/scan` | 仅退出时停车 | 唯一调用 `fsm.tick()` |
| `scripts/search_fsm.py` | **总状态机**、控制权仲裁 | Recovery / blocked 后退 / 返航 | build `20260705-p2-planner` |
| `scripts/config_loader.py` | 读取 `mission.yaml` | 否 | SearchConfig / MapConfig |
| `config/mission.yaml` | 运行参数 | 否 | 边界距离、恢复级别、引导参数等 |
| `scripts/ooxx_logrun.sh` | 启动命令包装，日志写入 `log/` | 否 | `rosrun ooxx ooxx_logrun.sh ...` |
| `scripts/ooxx_cleanup.sh` | 清理残留节点 | 否 | 分步启动前可选 |

### 状态机状态一览

| 状态 | 处理函数 | 控制权（当前） |
|------|----------|--------------|
| EXPLORE | `_tick_explore` | 导航器；hold 由 `_track_nav_hold` 仲裁 |
| PLAN_NEXT | `_tick_plan_next` | 无（仅规划） |
| MOVE_TO_GOAL | `_tick_move_to_goal` | 导航器；blocked 可能触发状态机后退 |
| RECOVERY | `_tick_recovery` | Recovery 独占；不 tick 规划器/导航器 |
| RETURN_HOME | `_tick_return_home` | 导航器或状态机角速度 |
| DONE | tick 内 stop | 无 |

---

## 2. 运动控制

| 文件 | 职责 | 发布 cmd_vel? | 调用方 |
|------|------|---------------|--------|
| `scripts/move_controller.py` | **硬件唯一出口** | 是 | 状态机、导航器、牛耕规划器 |

### 接口分类

| 方法 | 类型 | 阻塞? | 适用场景 |
|------|------|-------|----------|
| `publish_twist` | 流式 | 否 | 导航器每 tick |
| `stop_robot` / `publish_stop_brief` | 流式 | 否 | 停车、恢复间隙 |
| `move_distance_x/y` | 阻塞 | **是** | Recovery 后退 |
| `rotate_angle` | 阻塞 | **是** | Recovery 转向 |
| `emergency_stop` | 阻塞 | 是 | 退出信号 |

**风险**：阻塞调用期间状态机无法切换策略；P3 拟改限时或非阻塞。

---

## 3. 导航与规划

| 文件 | 应有职责 | 当前实现 | 关键符号 |
|------|----------|----------|----------|
| `scripts/frontier_navigator.py` | 跟航点发速度；输出执行反馈 | hold/应力挡障；不触发规划器 | `tick()`, `_handle_center_blocked()` |
| `scripts/local_planner.py` | 局部选角、set_path | 不自动重规划；恢复后首轮过滤 | `replan_local()`, `arm_post_recovery_replan()` |
| `scripts/nav_feedback.py` | 反馈数据结构 | 否 | `NavExecutionFeedback`, `GoalStatus` |

### 模块契约（当前）

| 方向 | 设计意图 | 当前状态 |
|------|----------|----------|
| 导航器 → 规划器 | 只读 `execution_feedback` | 已落实；无自动 force replan |
| 规划器 → 导航器 | `set_path()` 设目标 | 仅状态机触发重规划 |
| 状态机 → 二者 | 状态决定谁运行 | RECOVERY 期间二者均不 tick |
| 状态机 hold 仲裁 | hold 超限进 Recovery | `_track_nav_hold()` 已实现 |

### 已清理（P2）

| 符号 | 文件 | 说明 |
|------|------|------|
| `_maybe_replan_on_exec_cost()` | — | 已删除 |
| `reactive_drive()` | — | 已删除 |
| `escape_forward_*` 配置 | mission.yaml | 已移除 |

### P2 新增（local_planner.py）

| 符号 | 说明 |
|------|------|
| `record_failure()` | Failure Memory 写入 |
| `_failure_memory_penalty()` | 空间+偏角降权 |
| `_wall_hug_penalty()` | 贴墙零度加罚 |
| `arm_post_recovery_replan(rounds=N)` | 恢复后多轮约束 |
| **纯激光选路**（`20260707`） | 前方受阻时比各方向净空，取最大者设短目标，跳过地图评分 |

---

## 4. 感知与地图

| 文件 | 职责 | 发布 cmd_vel? |
|------|------|---------------|
| `scripts/lidar_utils.py` | 扇区测距、边界判定、`clearance_profile` | 否 |
| `scripts/occupancy_grid.py` | 栅格地图、前沿点、`nearest_frontier` | 否 |
| `scripts/grid_planner.py` | A* 路径 | 否 |
| `scripts/pose_estimator.py` | 位姿、里程计健康 | 否 |
| `scripts/mission_state.py` | 目标计数、覆盖率 | 否 |

### 4.1 局部栅格地图与障碍标记

**作用**：维持一块以车身为中心的固定大小局部地图，记录已探索区域与障碍物，供路径搜索和边界选点使用。实现文件为 `scripts/occupancy_grid.py`，由状态机主循环每帧调用。

**地图范围**：首次运行时以起始位置为地图中心。边长与格宽见 `config/mission.yaml` 中地图一节（当前边长 12 米，格宽 0.08 米）。

**格的四种状态**：

| 状态 | 含义 |
|------|------|
| 未知 | 尚未被激光观测 |
| 可通行 | 激光射线途经 |
| 障碍 | 激光回波终点 |
| 已访问 | 小车曾经过 |

**每帧更新**（`search_fsm.py` 主循环）：

1. 读取激光测距，逐条束线更新地图：射线途经的未知格标为可通行，终点标为障碍；超出地图范围的束线丢弃。
2. 将当前车位对应的格标为已访问。
3. 统计覆盖率。

激光方向与车体前向通过对齐角统一，该参数与导航测距共用，见配置文件搜索一节（当前取圆周率）。

**障碍标记规则**：采用射线填充法——途经为可通行，终点为障碍。障碍格一旦写入不再被后续射线改回；仅未知格可被标为可通行。不做多帧融合或概率更新。

**碰撞代价图层**：撞墙后，除障碍标记外，还在当前位置周围累加碰撞代价（软惩罚，不直接把格标为障碍）。同一区域短期内多次撞墙时，代价进一步升高并可能写入拉黑区。参数见配置文件搜索一节。该图层影响局部方向选取、路径搜索权重和边界选点排序。

**与实时测距的区别**：扇区测距每帧判定能否前进；栅格地图累积环境信息供规划使用。二者读取同一激光话题，职责分开。相机只作前向识别，不参与建图。

---

### 4.2 边界判定双路径

**边界判定双路径**（仍并存，未统一）：

- `is_boundary()` → 状态机 `_is_boundary_confirmed` → `_on_boundary`
- 导航器 `hold/blocked` → 状态机 `_track_nav_hold` / `_on_boundary`

两套入口在墙角可能先后触发，但均导向 Recovery，暂无控制权冲突。

---

## 5. Recovery 相关

| 区域 | 职责 | 发布 cmd_vel? |
|------|------|---------------|
| `_start_boundary_recovery` | 进入 RECOVERY、入口基线 | stop |
| `_tick_boundary_recovery` | PAUSE/RECHECK/ACT | 经 `move_*` **是** |
| `_recovery_success` | `center - entry >= gain` | 否 |
| `_replan_after_recovery` | 恢复后首轮约束重规划 | 否 |
| `_try_escape_or_replan` | 交还状态机，直接重规划 | 否（无脱离窗口） |
| `_tick_generic_recovery` | 非边界恢复（旋转） | rotate_angle |
| `_run_strong_blocked_recovery` | 强后退+转+前进 | **阻塞是** |
| `_handle_nav_blocked` | 导航 blocked 时状态机后退 | **阻塞是（遗留）** |

### Recovery 配置（`config/mission.yaml`）

| 参数 | 含义 | 当前值 |
|------|------|--------|
| `recovery_backoff_levels` | 后退距离 | [0.10, 0.18, 0.28] |
| `recovery_turn_levels` | 转向角度 | [25, 35, 55] |
| `recovery_success_center_gain` | 成功阈值 | 0.05 |
| `recovery_max_level_retries` | 同级重试 | **2**（P1 自 5 改为 2） |
| `recovery_replan_max_angle_deg` | 恢复后首轮偏角上限 | 35.0 |
| `recovery_replan_max_dist_m` | 恢复后首轮距离上限 | 2.0 |
| `escape_forward_time/speed` | 脱离窗口 | 配置仍存，**代码已不使用** |
| `bootstrap_local_plan` | 建图引导期局部规划 | true |
| `bootstrap_visited` | 引导期栅格 visited 阈值 | 40 |
| `nav_forward_stop_margin` | 直行停车余量 | 0.04（center < 0.34 禁前进） |

---

## 6. 视觉与其它

| 文件 | 职责 | 影响运动? |
|------|------|-----------|
| `scripts/perception/topic.py` | 读 `/target_current` | 仅任务完成判定 |
| `scripts/target_detector.py` | 相机检测 | 独立节点 |
| `scripts/boustrophedon_planner.py` | 牛耕条带（非默认模式） | 经 move 阻塞调用 |

**当前实验**：`chassis_search.launch` 不启相机；视觉模块不参与运动决策。

---

## 7. 日志与调试

| 路径 | 内容 |
|------|------|
| `log/*.txt` | 实车运行日志（`ooxx_logrun.sh` 自动保存） |
| `plan.md` | 开发阶段与关键绩效指标 |
| `使用方法.txt` | 编译、启动、日志、构建标识核对 |
| `docs/01-architecture.md` | 架构与控制权 |
| `docs/02-call_chains.md` | 调用链 |
| `docs/03-module_index.md` | 本文件 |

### 关键日志模式

```
FSM init: build=20260705-p2-planner     # 版本确认
LocalPlanner: FailureMemory 记录        # P2 失败区域记忆
LocalPlanner: 恢复后第N轮               # P2 多轮约束
... fm= / wall= / dist=                 # P2 评分加罚
Bootstrap exit                          # P2 引导期切 Frontier
FSM: Navigator hold N ticks              # hold 仲裁
NavFeedback: … reason= / stress=         # 导航器状态
Boundary hit                             # 进 Recovery
运行统计(60s): 边界= 恢复= 覆盖率=       # 阶段绩效
```

### 10-24 实车摘要

| 项 | 值 |
|----|-----|
| 构建 | `20260705-p1-recovery` |
| 60s 边界/恢复 | 6 / 6 |
| 60s 覆盖率 | 6% |
| 增益成功 | 多次 level 1/2 判定成功 |
| RETURN_HOME | 未触发 |
| 主要问题 | 左下角墙角局部规划死循环；开阔区域在侧后方不转向 |

---

## 8. 改代码前检查清单

- [ ] 是否新增 `publish_twist` / `move_*` 调用点？
- [ ] RECOVERY 期间是否仍调用 `local_planner.tick` 或 `navigator.tick`？
- [ ] 导航器反馈是否被规划器当成「命令」而非「建议」？
- [ ] 是否存在状态机与导航器双路径发速度？
- [ ] 阻塞 Recovery 是否超过 20s？
- [ ] 死代码是否一并清理？

**若前四项任一为是 → 先收敛控制权，再调参数。**

---

## 9. 开发优先级（与 plan.md 对齐）

| 优先级 | 动作 | 涉及模块 | 状态 |
|--------|------|----------|------|
| P0 | 删除脱离窗口；hold 状态机仲裁；规划器不自动重规划 | search_fsm, local_planner | ✅ |
| P1 | 入口基线增益；重试 2 次；恢复后首轮约束 | search_fsm, local_planner, mission.yaml | ✅ |
| P2 | 局部规划器优化 | local_planner, search_fsm, mission.yaml | ✅ 待实车 |
| P3 | 车体包络；提前停车；应力侧向脱困；阻塞恢复改限时 | frontier_navigator, move_controller | 待做 |
| P4 | 长跑稳定性；正式返航验证 | search_fsm, mission.yaml | 待做 |
| 清理 | 删除废弃重规划与 reactive_drive | — | ✅ P2 |

---

## 10. 模块记忆

```
ooxx_node      → 只跑循环
SearchFSM      → 谁有权动（仲裁者）
Navigator      → 怎么跟目标走
LocalPlanner   → 往哪走（只选点，不抢方向盘）
MoveController → 油门出口
Recovery       → 撞了怎么办（恢复期间独占）
ooxx_logrun    → 命令与日志一并保存
```
