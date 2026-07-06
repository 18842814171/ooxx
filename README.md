# ooxx — 合并版实验包

**ooxx_old 底盘搜索** + **search_navigation 相机识别**，各取所长。

## 模块分工

### 来自 ooxx_old（底盘）
- `move_controller.py` — 底盘运动原语
- `search_fsm.py` — LiDAR 覆盖搜索状态机
- `occupancy_grid.py` / `frontier_navigator.py` — 占据栅格与前沿导航
- `boustrophedon_planner.py` — 牛耕式条带扫描
- `lidar_utils.py` — 边界检测

### 来自 search_navigation（相机）
- `target_detector.py` — Orbbec RGB→BGR + LAB 颜色识别 + 多帧确认（与 search_navigation 一致）
- 发布 `/target_current`（当前颜色）、`/target_detected`（新目标事件）、`/detection_image`（标注画面）

### 桥接层
- `perception/topic.py` — `TopicVisionBackend`，在 STOP_SCAN 时读取 `/target_current` 并登记目标

## 编译与运行

```zsh
cd ~/experiment_ws
catkin_make --pkg ooxx
source devel/setup.zsh
roslaunch ooxx ooxx.launch
```

## 与旧版区别

| | ooxx_old | 新 ooxx |
|---|----------|---------|
| 相机 | ooxx_node 内嵌 classical 感知 | 独立 target_detector 节点 |
| 图像 | numpy buffer 直读 | RGB→BGR + numpy buffer（search_navigation 方案） |
| 调试 | cv2.imshow | show_debug:=true 时 OpenCV 窗口 |
| 底盘 | SearchFSM + 占据栅格 | 保持不变 |

## mission.yaml

`vision.backend: topic`（默认）使用 search_navigation 相机方案。
若需回退内嵌识别，改为 `classical`。

## 常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| 分步启动后雷达「断开」/算法像模拟在跑 | 旧版 `require_scan` 虚拟扫描残留，不依赖真实 /scan | 已删除虚拟扫描；`catkin_make --pkg ooxx` 后重试 |
| 终端 3 重复起雷达 | 未加 `launch_lidar:=false` 用了 chassis_search | 改用 `ooxx_node.launch`，或加 `launch_lidar:=false` |
| /scan 有、/odom 无 | 底盘驱动挂了/串口被占 | 清理进程，重启底盘节点 |
| 分步第一次失败，重启小车后好 | 上次残留进程 + 串口坏状态 | 启动前先 `rosrun ooxx ooxx_cleanup.sh` |
| 算法转圈不走 | /odom 无或 yaw 不变 | 先修底盘，再谈算法 |
