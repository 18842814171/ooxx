# 03 — 模块索引

> 本文遵守以下规则
>
> - 本表标注每个文件的**职责**与**现状**；改代码前先查「谁有权发速度」。
> - 配置优先改 `config/mission.yaml`；**控制权冲突不能靠调参解决**。

---

## 1. 入口与调度

| 文件 | 职责 | 发布 cmd_vel? | 备注 |
|------|------|---------------|------|
| `scripts/ooxx_node.py` | 节点主循环、订阅 `/scan` | 仅退出时停车 | 唯一调用 `fsm.tick()` |
| `scripts/search_fsm.py` | **总状态机**、控制权仲裁 | Recovery / 受阻轻退 / 返航 | |
| `scripts/config_loader.py` | 读取 `mission.yaml` | 否 | SearchConfig / MapConfig |
| `config/mission.yaml` | 运行参数 | 否 | 边界距离、恢复级别、引导参数等 |
| `scripts/ooxx_logrun.sh` | 启动命令包装，日志写入 `log/` | 否 | `rosrun ooxx ooxx_logrun.sh ...` |
| `scripts/ooxx_cleanup.sh` | 清理残留节点 | 否 | 分步启动前可选 |

### 状态机状态一览

| 状态 | 处理函数 | 控制权（当前） |
|------|----------|--------------|
| PLAN_NEXT | `_tick_plan_next` | 无（仅规划）；全局规划启用时为常见入口 |
| EXPLORE | `_tick_explore` | 前沿导航器；无候选 → 逃离重规划 |
| MOVE_TO_GOAL | `_tick_move_to_goal` | 前沿导航器；受阻 → 重规划 |
| RECOVERY | `_tick_recovery` | Recovery 独占；不调用局部规划器与前沿导航器 |
| GLOBAL_PLAN | `_tick_global_plan` | 绕场时跟航点；仅 `perimeter_enabled: true` |
| RETURN_HOME | `_tick_return_home` | 前沿导航器或状态机角速度 |
| DONE | 主循环内停车 | 无 |

---

## 2. 运动控制

| 文件 | 职责 | 发布 cmd_vel? | 调用方 |
|------|------|---------------|--------|
| `scripts/move_controller.py` | **硬件唯一出口** | 是 | 状态机、前沿导航器、牛耕规划器 |

### 接口分类

| 方法 | 类型 | 阻塞? | 适用场景 |
|------|------|-------|----------|
| `publish_twist` | 流式 | 否 | 前沿导航器每帧 |
| `stop_robot` / `publish_stop_brief` | 流式 | 否 | 停车、恢复间隙 |
| `move_distance_x/y` | 阻塞 | **是** | Recovery 后退、受阻轻退 |
| `rotate_angle` | 阻塞 | **是** | Recovery 转向 |
| `emergency_stop` | 阻塞 | 是 | 退出信号 |

**风险**：阻塞调用期间状态机无法切换策略；后续拟改限时或非阻塞。

---

## 3. 导航与规划

| 文件 | 职责 | 现状 | 关键符号 |
|------|------|------|----------|
| `scripts/frontier_navigator.py` | 跟航点发速度；输出执行反馈 | 按前方净空连续限速；受阻确认后上报 blocked | `tick()`, `_handle_center_blocked()`, `_simple_path_clear()` |
| `scripts/local_planner.py` | 局部选角、`set_path` | 重规划由状态机触发；恢复后多轮候选过滤 | `replan_local()`, `arm_post_recovery_replan()`, `record_failure()` |
| `scripts/global_planner.py` | 全局模式与绕场/边界入口 | 默认边界探索 | `plan()`, `update()` |
| `scripts/perimeter_controller.py` | 绕场贴墙状态 | 配置关闭时不参与 | `PerimeterController` |
| `scripts/nav_feedback.py` | 反馈数据结构 | 否 | `NavExecutionFeedback`, `GoalStatus` |

### 模块契约（当前）

| 方向 | 设计意图 | 现状 |
|------|----------|------|
| 前沿导航器 → 局部规划器 | 只读 `execution_feedback` | 已落实 |
| 局部规划器 → 前沿导航器 | `set_path()` 设目标 | 重规划由状态机触发 |
| 状态机 → 二者 | 状态决定谁运行 | RECOVERY 期间二者均不调用 |
| 状态机 → 受阻处理 | 清目标、记失败、重规划 | `_handle_nav_blocked()` |

### 局部规划器扩展能力

| 符号 | 说明 |
|------|------|
| `record_failure()` | 失败区域记忆写入 |
| `_failure_memory_penalty()` | 空间与偏角降权 |
| `_wall_hug_penalty()` | 贴墙小角度加罚 |
| `arm_post_recovery_replan(rounds=N)` | 恢复后多轮约束 |
| `_pick_clearance_escape()` | 纯激光选路（`local_clearance_escape_enabled` 控制） |

---

## 4. 感知与地图

| 文件 | 职责 | 发布 cmd_vel? |
|------|------|---------------|
| `scripts/lidar_utils.py` | 扇区测距、边界判定、`clearance_profile` | 否 |
| `scripts/occupancy_grid.py` | 栅格地图、前沿点、`nearest_frontier` | 否 |
| `scripts/grid_planner.py` | 路径搜索 | 否 |
| `scripts/pose_estimator.py` | 位姿、里程计健康 | 否 |
| `scripts/mission_state.py` | 目标计数、覆盖率 | 否 |

### 4.1 局部栅格地图与障碍标记

**作用**：维持一块以车身为中心的固定大小局部地图，记录已探索区域与障碍物，供路径搜索和边界选点使用。实现文件为 `scripts/occupancy_grid.py`，由状态机主循环每帧更新。

**地图范围**：首次运行时以起始位置为地图中心。边长与格宽见 `config/mission.yaml` 地图一节（当前边长 15 米，格宽 0.08 米）。

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

激光方向与车体前向通过对齐角统一，该参数与导航测距共用，见配置文件搜索一节。

**障碍标记规则**：采用射线填充法——途经为可通行，终点为障碍。障碍格一旦写入不再被后续射线改回；仅未知格可被标为可通行。不做多帧融合或概率更新。

**碰撞代价图层**：撞墙后，除障碍标记外，还在当前位置周围累加碰撞代价（软惩罚，不直接把格标为障碍）。同一区域短期内多次撞墙时，代价进一步升高并可能写入拉黑区。该图层影响局部方向选取、路径搜索权重和边界选点排序。

**与实时测距的区别**：扇区测距每帧判定能否前进；栅格地图累积环境信息供规划使用。二者读取同一激光话题，职责分开。相机只作前向识别，不参与建图。

---

### 4.2 边界与受阻入口

两套入口并存，职责不同：

- **边界确认**：`is_boundary()` → `_is_boundary_confirmed` → `_on_boundary` → RECOVERY（边界恢复）
- **导航受阻**：前沿导航器 `blocked` → `_handle_nav_blocked` → PLAN_NEXT 重规划（多数情况）

墙角可能先后触发，但速度发布点仍由状态机按态仲裁。

---

## 5. Recovery 相关

| 区域 | 职责 | 发布 cmd_vel? |
|------|------|---------------|
| `_start_boundary_recovery` | 进入 RECOVERY、记录入口基线 | stop |
| `_tick_boundary_recovery` | 暂停 / 复检 / 单次后退转向 | 经 `move_*` **是** |
| `_snapshot_recovery_entry` | 记录窄扇区入口基线 | 否 |
| `_replan_after_recovery` | 恢复后多轮约束重规划 | 否 |
| `_try_escape_or_replan` | 恢复完成，交还状态机并重规划 | 否 |
| `_tick_generic_recovery` | 非边界恢复（开阔侧转向或递增旋转） | rotate_angle |
| `_handle_nav_blocked` | 导航受阻清目标；仍堵时轻退 | 仍堵时 **是**（后退 0.10 m） |
| `_enter_planner_deadlock_recovery` | 连续无候选，逃离重规划 | 否 |

### Recovery 配置（`config/mission.yaml`）

| 参数 | 含义 | 当前值 |
|------|------|--------|
| `recovery_backoff_levels` | 后退距离 | [0.12] |
| `recovery_turn_levels` | 转向角度 | [45] |
| `recovery_success_center_gain` | 成功阈值（日志参考） | 0.05 |
| `recovery_max_level_retries` | 同级重试 | 0 |
| `recovery_replan_max_angle_deg` | 恢复后规划偏角上限 | 70.0 |
| `recovery_replan_max_dist_m` | 恢复后规划距离上限 | 3.0 |
| `recovery_replan_rounds` | 恢复后约束持续轮数 | 3 |
| `bootstrap_local_plan` | 建图引导期局部规划 | false |
| `bootstrap_visited` | 引导期 visited 阈值 | 40 |
| `nav_forward_stop_margin` | 直行停车余量 | 0.04（约 0.34 m 起明显限速） |
| `explore_no_candidate_frames` | 连续无候选帧数阈值 | 15 |

---

## 6. 视觉与其它

| 文件 | 职责 | 影响运动? |
|------|------|-----------|
| `scripts/perception/topic.py` | 读 `/target_current` | 仅任务完成判定 |
| `scripts/target_detector.py` | 相机检测 | 独立节点 |
| `scripts/boustrophedon_planner.py` | 牛耕条带（非默认模式） | 经 move 阻塞调用 |
| `scripts/depth_projection.py` | 深度像素投影 | 视觉去重，不参与运动 |

**当前实验**：`chassis_search.launch` 不启相机；视觉模块不参与运动决策。

---

## 7. 日志与调试

| 路径 | 内容 |
|------|------|
| `log/*.txt` | 实车运行日志（`ooxx_logrun.sh` 自动保存） |
| `使用方法.txt` | 编译、启动、日志 |
| `docs/01-architecture.md` | 架构与控制权 |
| `docs/02-call_chains.md` | 调用链 |
| `docs/03-module_index.md` | 本文件 |

### 关键日志模式

```
FSM init: …                           # 启动：模式、初始状态
LocalPlanner: FailureMemory 记录        # 失败区域记忆
LocalPlanner: 恢复后第N轮               # 多轮约束
Frontier pick: … astar_eval=…         # 边界多候选选点
Nav blocked … replan                  # 受阻重规划
探索：连续 N 帧无有效候选，逃离重规划
Corner stall … replan                 # 墙角停滞重规划
NavFeedback: … reason=                # 前沿导航器状态
Boundary hit                          # 进 RECOVERY
Recovery: 入口基线 center=…
Recovery(no_frontier): open-side turn
运行统计(60s): 边界= 恢复= 角落停滞= 覆盖率=
```

---

## 8. 改代码前检查清单

- [ ] 是否新增 `publish_twist` / `move_*` 调用点？
- [ ] RECOVERY 期间是否仍调用局部规划器或前沿导航器？
- [ ] 前沿导航器反馈是否被局部规划器当成「命令」而非「建议」？
- [ ] 是否存在状态机与前沿导航器双路径发速度？
- [ ] 阻塞 Recovery 是否超过 20s？

**若前四项任一为是 → 先收敛控制权，再调参数。**

---

## 9. 模块记忆

```
ooxx_node        → 只跑主循环
SearchFSM        → 谁有权动（仲裁者）
FrontierNavigator → 怎么跟目标走
LocalPlanner     → 往哪走（只选点，不抢方向盘）
GlobalPlanner    → 全局边界/绕场选点
MoveController   → 油门出口
Recovery         → 撞了怎么办（恢复期间独占）
ooxx_logrun      → 命令与日志一并保存
```
