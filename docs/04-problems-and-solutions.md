# 04 — 问题与对策记录

> 本文档整理自 2026 年 7 月局部规划与恢复机制迭代过程中的实车现象、日志分析与架构讨论。  
> 编写遵循 `重要原则.txt`：书面语、参数集中于 `config/mission.yaml`、较大改版须清理旧路径。  
> 与 `01-architecture.md`（控制权）、`03-module_index.md`（模块职责）配合阅读。

---

## 1. 总体判断

早期版本功能较少，行为相对可预测。后续为解决现场问题，陆续叠加边界处理、应力阶梯、通道判断、角落脱困、空间冷却、重试、恢复、建图绕行、局部规划、前沿选点等机制。各机制单独看均合理，组合后**互相争用控制权**，系统难以理解与调试。

经多轮实车日志（`ooxx-1.txt`、`log/8.05.txt`、`log/8.27.txt`、`log/1229.txt`、`log/21-14.txt`、`log/20-43.txt`、`log/8-2.txt`）与代码对照，结论如下：

1. **主因不在恢复参数偏小**，而在恢复触发之前，局部规划器已无法产生可执行候选，或恢复成功后立即重选同一失败方向。
2. **前沿导航器在多数情况下行为正确**：目标在侧前方时先转向再前进，属于控制器应然响应；宽缝撞障往往源于规划器未允许朝通道方向行进。
3. **不宜继续增加「通道模式」「角落模式」等分支**；应通过连续量统一代价与模块契约收敛复杂度。
4. **迭代原则**：每一处修改可单独回退，日志能明确验证效果；冻结期内除缺陷修复外不新增功能。
5. **推进方式调整**（2026-07-06）：架构层（恢复链、Bootstrap、不推块）基本收敛后，**按验收问题（Issue）推进**，不再按 P3d/P3e 等模块编号叠加；Issue-1 完成前不改 Planner、Recovery 评分，深度相机最后介入。

**当前构建标识**：`scripts/search_fsm.py` 顶部 `FSM_BUILD_ID = '20260706-corridor-commit-lifecycle'`。实车以日志 `FSM init: build=...` 为准。

---

## 2. 问题归类（合并同类）

### 2.1 局部规划器无有效候选（角落停滞、贴墙停滞、宽缝撞障的共同根因）

| 现象 | 日志特征 |
|------|----------|
| 角落或贴墙处长时间不动 | `Selected: none (all rejected)` 连续出现 |
| 面对宽缝人为转弯仍撞障 | 大偏角方向 `clear` 充足，仍 `INVALID` |
| 反应式直行无效 | `reactive` 低速顶墙 → `Boundary hit` → `RECOVERY` |

**根因（规划器层）：**

栅格可达性检查与激光测距结论不一致。`local_planner._evaluate` 中，激光认为可通行的方向，若 `path_is_clear` 或 `plan_path` 失败，候选即被否决，形成：

```
激光可通 → 栅格判堵 → 全部无效 → 反应式慢速顶墙 → 边界 → 恢复
```

```548:561:scripts/local_planner.py
    if center < limit:
      return _Candidate(
          angle_deg, 0.0, gx, gy, [], dist, clearance, 0.0, False, ScoreBreakdown(),
      )
    ...
    path = self.grid.plan_path(rx, ry, gx, gy)
    if not path:
      path = [(gx, gy)]
```

**与阈值微调的关系：** `boundary_dist = 0.30` 等参数差异不会单独导致数十秒停滞；真正致滞的是候选集为空。

---

### 2.2 探索状态脱困滞后

| 现象 | 说明 |
|------|------|
| 规划失败持续数十秒才恢复 | 原 `explore_stall_sec = 30` 依赖位移判定 |
| 机器人已知连续多帧无候选 | 仍等待位姿几乎不变 |

探索状态缺少「连续无候选即脱困」的判定，与导航状态已有 `corner_stall` 不对称。

---

### 2.3 恢复动作不足与成功判定过松

| 现象 | 日志特征 |
|------|----------|
| 恢复后随即再撞 | `back=0.05m turn=15°`，前方余量几乎无改善 |
| 永远停留在最低恢复级别 | `gain=0.00` 或 `gain=-0.01` 仍判定结束 |
| 同位置多次边界事件 | 按边界类型（侧墙、角落、重复）差异化缩放后退量，实际削弱动作 |

**根因：**

- 恢复幅度按 `kind` 打折（如侧墙 `back *= 0.6`），与配置表不一致。
- 成功判定曾允许仅凭位移 `disp ≥ 0.06m` 通过，而前方窄扇区余量未改善（`center_gain < 0.05`）。
- 最高级恢复失败后立即强制结束，未给同级重试机会。

---

### 2.4 恢复后重复选择失败方向

| 现象 | 说明 |
|------|------|
| 脱困后回到原路径再撞 | 重规划仍选 0° 或刚撞过的偏角 |
| 脱离窗口加剧贴墙 | 恢复完成后强制直线前进 0.4s，角落中等价于多顶墙一次 |

**根因：**

- 方向惩罚有效期过短（曾 1s），未覆盖整轮恢复周期。
- 惩罚记录的是目标方位而非上次选中候选偏角，与真实撞障方向不一致。
- 脱离窗口曾绕过导航器直连速度，在窄扇区未畅通时仍前进（后已删除）。

---

### 2.5 模块争用与信息断层

| 冲突 | 说明 |
|------|------|
| 规划器与导航器 | 规划器认为目标可行，导航器因 `wide` 过小已停车 |
| 恢复与规划器 | 恢复结束后再规划、清目标、拉黑，职责重叠 |
| 多状态入口 | `EXPLORE`、`PLAN_NEXT`、`MOVE_TO_GOAL` 均可间接驱动规划 |

---

### 2.6 导航层临界距离与直行撞障（独立层级）

| 现象 | 说明 |
|------|------|
| `center` 接近边界阈值仍前进撞障 | 如 `NavFeedback center=0.30` 时撞上低矮障碍 |
| 与角落恢复不同类 | 属于导航器挡障与测距盲区，非恢复链问题 |

按 `重要原则.txt`，此类问题由深度相机补齐激光盲区，**不参与规划评分**。

---

### 2.7 导航层行为缺口（Bootstrap、缝隙、开阔区、宽距停车的合并归类）

恢复链与候选链路改善后，实车日志暴露出**导航器层**的独立问题，与 2.1 规划器无候选不同类：

| 现象簇 | 典型日志 | 根因 |
|--------|----------|------|
| 建图引导反复进入 | `map bootstrap` 多次出现，覆盖率停滞 | Bootstrap 退出条件分散，无一次性标志 |
| 缝隙模式未主动进入 | 仅恢复后出现 `PassageMode armed`；正常导航长期 `hold` | 缺少 `passage_score` 与几何判据的主动入口 |
| 开阔区长时间原地转向 | `align` 无线速度，位姿几乎不变 | 前方 `center` 充足仍纯旋转对齐 |
| 宽距误停 | `center≥0.5` 但 `wide` 偏低即 `hold` | `wide hold` 与前方净空未解耦 |
| 缝内 arm→done 短周期振荡 | `done ticks=9` 后立即再 `armed` | 以单帧 score/几何丢失即退出，无承诺期 |

**与恢复链的关系**：上述现象导致边界事件减少缓慢、覆盖率偏低，易被误判为「恢复力度不足」；实车 `log/21-14.txt`、`log/20-43.txt` 中缝 A（物块+物块）可慢速通过，缝 B（物块+墙）则 `score=0.00` 停住，属导航层缝隙逻辑缺口，非规划器候选为空。

---

### 2.8 块+墙单通道（第二个缝）问题簇

一侧物块或墙近、另一侧开阔的几何（`L≈0.3–0.5, R≥0.75`）在 `log/8-2.txt`（build=`issue1-channel`）中集中暴露：

| 现象 | 日志特征 | 根因 |
|------|----------|------|
| 识别后极慢、推块 | `PassageMode armed (channel)`，`R=0.29`，均速约 0.04 m/s | 通道内仍侧移顶物块；近侧未单独限速 |
| 短周期假退出 | `PassageMode done ticks=9/10` | 单帧几何抖动即退出，无最短持有 |
| 偏角拒入 | `channel_heading=111deg` 拒 arm | channel 入口误用目标偏角门槛 |
| 穿出后难再入 | arm→done→arm 振荡 | 穿出未保持入口航向 |
| 开关未开 | `channel_auto_arm_disabled`（`log/1229.txt`） | `nav_passage_channel_auto_arm: false` 验收期默认关闭 |

**完成标准**（Issue-1）：同一墙+物块缝 **连续 3 次通过**，无推块、无 `corner_stall`、无 9 tick 级假退出。

---

## 3. 误判与纠正

| 早期判断 | 纠正 |
|----------|------|
| 恢复参数过小是主因 | 恢复频繁是因为规划器先制造了不可行目标或恢复后重选失败方向 |
| 宽缝撞障是航向增益问题 | 日志显示大偏角 `clear` 充足仍无效，根因在栅格否决 |
| 需按边界类型差异化恢复 | **否决**；统一读取 `recovery_backoff_levels` / `recovery_turn_levels` |
| 脱离窗口可稳定脱困 | 窄扇区盲目前进反而延长贴墙；**已删除** `_tick_escape_forward` |
| 连续两次 0° 扣分促扫图 | **否决**；前方持续最优时应直行 |
| 深度参与规划评分 | **否决**；深度仅作安全增强 |
| 缝 B 停住是规划器问题 | **否决**；`L=0.45 R=1.80` 时大偏角候选可通，根因在通道评分与 arm 条件 |
| 通道内应侧移避障 | **否决**（块+墙缝）；侧移顶物块，应仅降速、禁止朝近侧横移 |
| 按 P3d/P3e 模块并行推进 | **否决**；改为 Issue-1～4 逐项实车验收 |

---

## 4. 迭代对策与验证（按轮次）

配置项均集中于 `config/mission.yaml`，经 `scripts/config_loader.py` 加载。

### 4.1 第一轮：打通候选链路（P0–P2）

| 编号 | 对策 | 代码要点 | 日志验证 |
|------|------|----------|----------|
| P0 | 取消栅格路径对候选的一票否决 | `_evaluate` 仅保留 `center < limit`；`plan_path` 失败时用 `[(gx, gy)]` 继续评分 | 出现 `Selected: ±XX°` 与 `score breakdown` |
| P1 | 探索连续无候选立即恢复 | `_explore_no_candidate_count`，阈值 `explore_no_candidate_frames: 15` | `planner deadlock recovery` |
| P2 | 加大恢复后退与转角 | `recovery_backoff_levels: [0.10, 0.18, 0.28]`，`recovery_turn_levels: [25, 35, 55]` | `unified level=N back=... turn=...` |
| P3 | 宽缝边转边走 | `frontier_navigator` 窄通道条件下带线速度 `align_arc` | `align_arc` 伴随非零线速度 |

**实车评估（`log/8.05.txt`）：** 候选恢复、贴墙与宽缝有改善；但角落徘徊加剧，脱离窗口与宽松成功判定引入新问题。

---

### 4.2 第二轮：恢复链收敛

| 对策 | 要点 | 后续处置 |
|------|------|----------|
| 统一恢复动作 | `_escalation_back_turn` 不再按 `kind` 缩放 | 保留 |
| 脱离窗口 | 恢复后 0.4s 直线前进再重规划 | **已删除**；`_try_escape_or_replan` 直接重规划 |
| 方向惩罚 | `note_boundary_hit` 记录选中偏角；`recent_collision_penalty_sec: 3.0` | 保留并增强 |
| 收紧成功判定 | 仅 `center_gain ≥ recovery_success_center_gain`（0.05）可判成功 | 保留 |
| 最高级同级重试 | `recovery_max_level_retries: 2`，Level 2 未达增益可再试 | 保留 |

成功判定核心逻辑：

```642:675:scripts/search_fsm.py
  def _recovery_success(self, scan: LaserScan) -> bool:
    ...
    center_gain = center - self._recovery_entry_center
    ok = center_gain >= self.search.recovery_success_center_gain
    ...
    return ok
```

方向惩罚在恢复后重规划前刷新：

```278:287:scripts/local_planner.py
  def refresh_penalty_for_replan(self) -> None:
    ...
    self._last_boundary_time = rospy.Time.now()
    self._penalty_pending_replan = True
```

**实车评估（`log/8.27.txt`）：** 恢复可升级至 Level 2，方向惩罚日志出现；但 `gain≥0.05` 仍极少，最高级后被迫结束。

**实车评估（`log/1229.txt`，最新）：**

- 恢复链基本正常：`升级至 level 1/2`、`判定成功 ... gain=0.09~0.24` 多次出现。
- 方向惩罚生效：`恢复后刷新方向惩罚`、`首次重规划应用方向惩罚 pen=0.25`。
- 遗留：`no reachable frontier` 与原地旋转恢复循环；部分区域仍反复 `boundary_front_wall`。

---

### 4.3 规划器与导航器契约（持续）

| 项目 | 内容 |
|------|------|
| 导航器反馈 | `execution_cost`、`reason`、`metrics`、`GoalStatus` |
| 重规划滞后 | `execution_cost` 高于阈值且持续约 2s 才全局重选 |
| 远距离目标降权 | `local_plan_progress_dist_cap: 2.0`，`d>2m` 额外扣分 |
| 失败记忆 | `failure_memory_*` 对热区与近期失败偏角降权 |
| 贴墙零度加罚 | `wall_hug_zero_penalty` 抑制临界 clearance 时选 0° |

---

### 4.4 导航闭环与 Issue-1（P3b / P3c / channel / CorridorCommit）

恢复链收敛后，本轮迭代聚焦导航器，**不修改** `local_planner` 评分与 Recovery 成功判定。

| 构建 | 对策 | 代码要点 | 日志验证 |
|------|------|----------|----------|
| `20260705-p3b-closed-loop` | Bootstrap 一次性退出 | `_bootstrap_exited`；`PLAN_NEXT` 统一入口检查 | `Bootstrap exit` 仅一次 |
| 同上 | 缝隙主动进入 | `passage_score` + 两侧间距 + 目标方向 `auto-arm` | `PassageMode auto-arm` |
| 同上 | 开阔带速对齐 | `center` 充足时 `align_arc` 带线速度 | 开阔区 `drive`/`align_arc` 为主 |
| `20260705-p3c-efficiency` | 宽距停车的放宽 | `center≥0.70` 忽略 `wide hold` | `hold` 减少、`center` 限速生效 |
| 同上 | 缝隙可观测 | `PassageMode not armed because …` | 拒入原因可追踪 |
| `20260705-issue1-channel` | 块+墙通道评分 | `lidar_utils.passage_score` 一侧近一侧远 | `armed (channel)` 出现 |
| 同上 | channel 跳过 score 门槛 | `_is_block_wall_channel` → 直接 arm | 不再因 `score=0.00` 拒入 |
| `20260706-issue1-final` | 通道内不侧移 | `_apply_channel_lateral` 仅降速、`strafe=0` | 无侧向顶块 |
| 同上 | 持有与穿出航向 | 最短 15 tick + 连续 6 帧丢失；穿出 2 s 保持 yaw | 无 9 tick 假退出；无 `channel_heading` 拒入 |
| `20260706-corridor-commit-lifecycle` | 缝隙承诺生命周期 | `CorridorCommit`：固定入口航向、按前进距离退出；承诺期内规划器/换目标冻结 | `CorridorCommit start/done`；`traverse`/`timeout` |
| 同上 | channel 侧移约束 | `_apply_commit_centerline` 禁止朝物块侧横移；近侧停车 | `open=left/right`；`final_err` |
| 同上 | 验收开关 | `nav_passage_channel_auto_arm: false` 默认关 | `channel_auto_arm_disabled`（`log/1229.txt`） |

channel 几何判据（`frontier_navigator._is_block_wall_channel`）：

```501:514:scripts/frontier_navigator.py
  def _is_block_wall_channel(self, profile: Dict[str, float]) -> bool:
    ...
    if left_close and not right_close and right >= self.passage_channel_open_min:
      return True
    if right_close and not left_close and left >= self.passage_channel_open_min:
      return True
```

**实车评估摘要：**

| 日志 | 构建 | 结论 |
|------|------|------|
| `log/8-2.txt` | `issue1-channel` | 缝 B 可 arm 但极慢、有推块；9 tick 假退出；60 s 均速 0.05 m/s、覆盖率 6% |
| `log/21-14.txt` | — | 缝 A 慢通；缝 B `score=0.00` 于 `L=0.45 R=1.80` |
| `log/1229.txt` | `corridor-commit-lifecycle` | 恢复链正常；channel 因开关关闭未 arm；gap 缝可 `CorridorCommit` |

---

## 5. 架构共识

### 5.1 三模块职责

```
规划器   — 只负责「去哪」：统一代价、选候选、持有当前目标
导航器   — 只负责「怎么走」：同一套控制律、反馈 execution_cost
恢复     — 只负责「脱困」：后退 → 转向 → 验证（不承担重规划）
```

```
规划器 ──目标──▶ 导航器
导航器 ──状态+execution_cost──▶ 规划器
导航器 ──持续失败──▶ 恢复
恢复 ──完成──▶ 状态机 ──▶ 规划器（触发重选，恢复本身不规划）
```

### 5.2 明确不做的事项

| 事项 | 原因 |
|------|------|
| 通道/角落/隧道等独立模式 | 易导致状态机膨胀与行为不可解释 |
| 脱离窗口直连速度 | 绕过导航器，角落顶墙；已清理 |
| 恢复内嵌规划、前沿、冷却 | 与规划器职责重叠 |
| 深度参与规划评分 | 见 `重要原则.txt` |
| 单纯延长路径生命周期而不验证可达性 | 无法解决夹缝内不可行目标 |
| Issue-1 完成前改 Planner/Recovery | 与 `重要原则.txt`「一天只解决一个问题」冲突 |
| 用深度修 Planner 选点 | 深度仅补 LiDAR 盲区，不参与规划决策 |

---

## 6. 当前遗留问题

| 问题 | 现象 | 状态 / 方向 |
|------|------|-------------|
| **Issue-1** 块+墙缝稳定通过 | `issue1-channel` 部分通过；`channel_auto_arm` 默认关 | 开启 `nav_passage_channel_auto_arm` 后实车验收；见 §7 |
| **Issue-2** 缝内不断链 | arm→done 短周期振荡 | `CorridorCommit` 承诺期已加；与 Issue-1 同测 |
| **Issue-3** 缝内速度 | 均速约 0.04–0.05 m/s | Issue-1/2 过关后仅调 `passage_creep_factor` 等 |
| **Issue-4** 覆盖率长跑 | 60 s 覆盖率 6% | 前三项过关后再做 10 分钟长跑 |
| 前沿不可达循环 | `no reachable frontier` 旋转恢复 | 检查 `blocked_goals`、A* 拒绝率 |
| 同区域反复贴墙 | 惩罚后仍选相近偏角 | 提高 `recent_collision_penalty` 或探索增益 |
| 直行临界撞障 | `center` 贴近 `boundary_dist` 仍前进 | 深度安全增强或 `nav_forward_stop_margin` |

---

## 7. 下一步计划

与 `重要原则.txt`「一天只解决一个问题」及 `plan.md` Issue 表一致：**先稳定通过，再提速**；每项单独实车、单独验收。

| Issue | 目标 | 完成标准 | 允许改动 | 禁止 |
|-------|------|----------|----------|------|
| **Issue-1** | 块+墙缝稳定通过 | 同缝连续 3 次通过，无推块 | 通道评分、arm 条件、航向锁定、hold 放宽 | Planner、Recovery、深度 |
| **Issue-2** | 缝内不断链 | 无 9 tick 级 arm→done 振荡 | 承诺持有（`CorridorCommit` / `min_hold_ticks`） | 同时改速度 |
| **Issue-3** | 缝内速度 | 均速 ≥ 0.08 m/s | 仅 passage 速度参数 | Issue-1/2 未过关 |
| **Issue-4** | 覆盖率长跑 | 连续 10 分钟稳定搜场 | 全局参数微调 | 前三项未过关 |

**Issue-1 实车步骤：**

1. `config/mission.yaml` 设 `nav_passage_channel_auto_arm: true`
2. 编译部署后确认 `FSM init: build=20260706-corridor-commit-lifecycle`
3. 仅测墙+物块缝，日志应见 `CorridorCommit start … channel` → `done reason=traverse`；不应见 `channel_auto_arm_disabled`、9 tick 级 `done`

**深度相机介入时机**（`重要原则.txt` 第二步）：Issue-1～4 与低姿态障碍推块仍失败时，深度仅补 `center/left/right` 测距，不参与规划评分。

**调参原则：** 先记录数十次真实运行日志，再调整 `mission.yaml`；优先验证行为闭环，而非并行改多个子系统。

---

## 8. 部署与日志核对

Python 包修改后须编译安装，不可仅替换文件夹：

```bash
cd ~/experiment_ws && catkin_make --pkg ooxx && source devel/setup.zsh
```

| 检查项 | 预期 |
|--------|------|
| 构建标识 | `FSM init: build=20260706-corridor-commit-lifecycle` |
| 有效候选 | `LocalPlanner score breakdown`，非持续 `Selected: none` |
| 恢复升级 | `升级至 level 1/2`，`unified level=N back=... turn=...` |
| 成功判定 | `判定成功 ... gain≥0.05`（非仅 `disp`） |
| 方向惩罚 | `记录边界候选偏角`、`恢复后刷新方向惩罚`、`pen=0.25` |
| 脱离窗口 | **不应再出现** `脱离窗口：...前进` |
| Bootstrap | `Bootstrap exit` **仅一次** |
| 缝隙进入 | `PassageMode auto-arm` 或 `CorridorCommit start` |
| 块+墙缝 | `channel=True`；**不应**长期 `channel_auto_arm_disabled`（验收时开关须为 true） |
| 缝内完成 | `CorridorCommit done reason=traverse`（非 9 tick 级 `timeout`） |
| 导航反馈 | `NavFeedback: status=... exec_cost=...` |

---

## 9. 主要配置索引

| 配置项 | 含义 |
|--------|------|
| `recovery_backoff_levels` / `recovery_turn_levels` | 恢复后退与转角级别 |
| `recovery_success_center_gain` | 恢复成功所需前方余量增益（米） |
| `recovery_max_level_retries` | 最高级未达增益时的同级重试次数 |
| `recent_collision_penalty*` | 近期撞障方向惩罚 |
| `explore_no_candidate_frames` | 连续无候选帧数阈值 |
| `local_plan_progress_dist_cap` | 进度评分距离上限 |
| `local_plan_score_*` | 统一代价各分项权重 |
| `failure_memory_*` | 失败记忆与热区降权 |
| `planner_debug` | 是否输出候选分项日志 |
| `nav_passage_*` | 缝隙模式阈值、主动进入、channel 几何与持有 |
| `nav_passage_channel_auto_arm` | 块+墙 channel 自动 arm（验收期开关） |
| `nav_corridor_commit_*` | 缝隙承诺距离、冷却、承诺期、航向 abort |

---

## 10. 相关文档

| 文档 | 内容 |
|------|------|
| `docs/01-architecture.md` | 控制权模型、构建版本表 |
| `docs/02-call_chains.md` | 调用链 |
| `docs/03-module_index.md` | 模块职责与状态一览 |
| `重要原则.txt` | 文档语气、配置集中、深度边界、迭代节奏 |
| `plan.md` | Issue 验收表与构建版本 |
| `使用方法.txt` | 实车编译、启动与日志样例 |

---

*文档版本：整理自 2026-07-04 ~ 2026-07-06 讨论、实车日志与代码现状；含 Issue-1 收尾与 CorridorCommit 生命周期；构建号以 `search_fsm.py` 源码为准。*
