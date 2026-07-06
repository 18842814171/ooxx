# 开发规划（以找齐目标总耗时为 KPI）

## P0：控制权收敛 ✅

build=`20260705-p0-control` — 文档 `docs/01-architecture.md`

## P1：Recovery 收尾 ✅

build=`20260705-p1-recovery` — 实车验证 `log/10-24.txt`

| 项 | 内容 |
|----|------|
| gain 快照 | 相对 Recovery **入口** center，非每次 ACT |
| retry | 5→2 |
| 恢复后首轮规划 | FSM 约束：\|偏角\|≤35°、d≤2m；不强制 PLAN_NEXT |

**验收**：`判定成功 … gain≥0.05` ✓；单次 Recovery <20s ✓；`恢复后首轮` ✓

**遗留**：墙角局部规划死循环、开阔侧方不转向 → P2/P3

## P2：局部规划器优化 ✅

build=`20260705-p2-planner` — 实车 `log/10-51.txt` 暴露墙角死循环

## P2b：恢复后目标质量 + 墙角专项 ✅

build=`20260705-p2b-planner` — 待实车验证

| 项 | 内容 |
|----|------|
| 恢复后约束 | d≤3m；墙角破格大角度（\|偏角\|≥50°） |
| 贴墙加罚 | ±25°；clearance 0.65 / alignment 0.07 |
| Failure Memory 聚类 | 同区 3 次 Recovery → 扣 0.5 |
| Bootstrap 提前出口 | 边界≥3、连续 Recovery≥3、90s 覆盖停滞 |
| open_side 平局 | 前扇区比较 + 上次转向取反 |
| 伪角落 | repeat/side_wall/single_wall → blacklist |

**验收**：5min 边界<8；60s 覆盖率>20%；日志 `墙角破格候选`

## P3：导航器包络 + 缝隙模式 ✅

build=`20260705-p3-passage` — 待实车验证

| 项 | 内容 |
|----|------|
| Bootstrap 修复 | 有目标时也检查退出 |
| Passage Mode | 窄通道恢复后锁定航向 creep 穿越 |
| 缝隙规划 | 恢复后 \|偏角\|≤70° ×2 轮 |
| 车体包络 | half_width + front_overhang 膨胀停车/侧移 |

**验收**：日志 `Bootstrap exit`；`PassageMode armed`；`缝隙规划`；`-70°` 可选中

## P3b：闭环修复（Bootstrap / Passage / 开阔导航）

build=`20260705-p3b-closed-loop` — 待实车验证

| 项 | 内容 |
|----|------|
| Bootstrap 一次性退出 | `_bootstrap_exited` 标志；PLAN_NEXT 不再回 `map bootstrap`；统一入口检查 |
| Passage 主动进入 | `passage_score`+两侧间距+目标方向自动 `arm`；缝隙期忽略 hold |
| 开阔导航 | `center≥0.8` 带速 `align_arc`；远目标动态放宽航向容差 |

**验收**：`Bootstrap exit` 仅一次；`PassageMode auto-arm`/`done`；开阔区 `drive/creep/align_arc` 为主；边界↓覆盖率↑

## P3c：导航效率闭环

build=`20260705-p3c-efficiency` — 待实车验证

| 项 | 内容 |
|----|------|
| wide hold 放宽 | `center≥0.70` 忽略 wide hold，限速按 center 计 |
| Passage 可观测 | `PassageMode not armed because …` 调试日志；门槛略降 |
| 开阔带速 | `open_align_center_min=0.65`，`open_align_arc_speed=0.10` |

**验收**：均速≥0.08 m/s；60s 覆盖率≥10%；日志见 `not armed because` 或 `auto-arm`/`done`

---

## 当前阶段：按验收问题推进（非模块推进）

架构层已收敛（P0/P1 ✅，Bootstrap ✅，不推块 ✅，Passage 已跑通 ✅）。后续**只修行为缺口**，每项单独实车、单独验收。

| Issue | 目标 | 完成标准 | 允许改动 | 禁止 |
|-------|------|----------|----------|------|
| **Issue-1** | 第二个缝（块+墙）稳定通过 | 该场景 **100% 连续通过** | score 判据、arm 条件、yaw 锁定、hold 放宽 | Planner、Recovery、深度 |
| **Issue-2** | Passage 保持不断链 | 不再 arm→done→arm 短周期振荡 | 先仅加 **最短 15 tick 持有**，看日志 | 同时改 creep/速度 |
| **Issue-3** | Passage 速度 | 缝内均速 **≥0.08 m/s** | 仅调 passage 速度参数 | Issue-1/2 未过关前不做 |
| **Issue-4** | 覆盖率长跑 | 连续 **10 分钟**稳定搜场 | 全局参数微调 | 前三个 Issue 未过关前不做 |

### Issue-1 实车记录

build=`20260705-issue1-channel` — 部分通过（`log/8-2.txt`）

build=`20260706-issue1-final` — 待实车验证

| 改动 | 内容 |
|------|------|
| passage_score | 块+墙单通道评分（一侧近一侧远） |
| channel arm | 识别 channel 后跳过 score 门槛，锁定当前 yaw |
| hold 放宽 | channel 下降低 center 停车阈值 |
| 通道内不侧移 | `_apply_channel_lateral` 仅降速、禁止侧向顶块 |
| channel 持有 | 最短 15 tick + 连续 6 帧丢失才退出 |
| 穿出航向保持 | 2 s 内沿用穿出 yaw 再入缝，取消 channel 偏角拒绝 |

**验收**：墙+物块缝连续 3 次通过、无推块、无 `channel_heading` 拒入、无 9 tick 假退出

**原则**：先稳定通过，再提速；一天只验证一个 Issue 的一项改动。

实车依据：`log/21-14.txt`（缝 A 慢通、缝 B `score=0.00` 停住于 `L=0.45 R=1.80 center=0.41`）。

## P4：长跑、回原点

前置：Issue-1～4 完成后再做。
