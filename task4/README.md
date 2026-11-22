# 任务四：Hello SLAM & Planning（第 5 周）

基于 Ubuntu 22.04 + ROS 2 Humble，在 Gazebo 中部署 RMUL 赛场仿真，用 Point-LIO 完成建图并保存点云，再把点云转成 Nav2 可用的栅格地图，最后配置 AMCL 做重定位验证。

## 环境

- 操作系统：Ubuntu 22.04.5 LTS（VMware 虚拟机，4 核 / 8 GB）
- ROS 2：Humble
- 仿真：Gazebo Classic 11 + `PB_RMSimulation`（`world:=RMUL`）
- 建图算法：Point-LIO
- 定位：AMCL（`nav2_amcl`）

## 流程

| 步骤 | 做什么 | 产物 |
|---|---|---|
| 1 | 启动赛场仿真 + Point-LIO，RViz 查看点云与 IMU | — |
| 2 | 键盘绕场一周建图，`Ctrl+C` 保存 | `scans.pcd`（38 MB） |
| 3 | 点云投影成二维占据栅格 | `rmul_map.pgm` + `rmul_map.yaml` |
| 4 | 地图装入 `rm_nav_bringup`，`mode:=nav localization:=amcl` 重定位 | 位姿收敛 |

## 编译与运行

### 1. 编译仿真工作空间

```bash
cd ~/ws_pb_rm
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

### 2. 启动建图

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL mode:=mapping lio:=pointlio lio_rviz:=True nav_rviz:=False
```

实测 `/livox/imu` 约 100 Hz、`/livox/lidar/pointcloud` 约 8.8 Hz、`/Odometry` 约 8.9 Hz。

### 3. 键盘驱动绕场

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4. 保存点云

建图结束后**只按一次 Ctrl+C**，等 20 秒让 Point-LIO 写盘，然后确认文件：

```bash
ls -lh ~/ws_pb_rm/src/rm_localization/point_lio/PCD/scans.pcd
```

### 5. 点云转栅格地图

```bash
cd ~/bjfu-nav-zhuyuxi/task4
python3 tools/pcd2pgm.py \
  ~/ws_pb_rm/src/rm_localization/point_lio/PCD/scans.pcd \
  map/rmul_map.pgm map/rmul_map.yaml \
  --resolution 0.05
```

### 6. 装载地图并启动导航

```bash
cd ~/ws_pb_rm/src/rm_nav_bringup
cp ~/bjfu-nav-zhuyuxi/task4/map/rmul_map.pgm map/RMUL.pgm
cp ~/bjfu-nav-zhuyuxi/task4/map/rmul_map.yaml map/RMUL.yaml
cd ~/ws_pb_rm && colcon build --symlink-install --packages-select rm_nav_bringup
source install/setup.bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL mode:=nav localization:=amcl lio:=pointlio lio_rviz:=False nav_rviz:=True
```

### 7. RViz2 给初始位姿

工具栏 **2D Pose Estimate** 在场地内点选一个与真实位置有偏差的位姿。AMCL 的 `update_min_d` 为 0.25 m、`update_min_a` 为 0.2 rad，必须推车超过这个量才会触发更新。

## 建图结果

- 点云：`scans.pcd`，1215842 点，38 MB
- 栅格：297 × 243 格，分辨率 0.05 m/格，覆盖 14.9 m × 12.2 m
- 占用 / 空闲 / 未知：13193 / 46767 / 12211
- 地图原点（左下角）：`[-4.492, -5.166, 0.0]`

![建图 RViz](docs/mapping_rviz.png)

![栅格地图](docs/map_pgm.png)

## 转换脚本说明

`tools/pcd2pgm.py` 把 PCD 投影成 map_server 可直接读取的 `.pgm` + `.yaml`。

| 参数 | 含义 | 默认 |
|---|---|---|
| `--resolution` | 栅格分辨率（米/格） | 0.05 |
| `--z-offset` | 障碍物相对地面的最低高度（米） | 0.15 |
| `--z-max` | 障碍物相对地面的最高高度（米） | 2.0 |
| `--min-cluster` | 连通块最小格数，小于此值当噪点删除 | 4 |
| `--inflate` | 障碍物膨胀格数 | 1 |

处理流程：

1. 解析 PCD 头部，支持 `ascii` 与 `binary` 两种存储，按 `FIELDS / SIZE / TYPE / COUNT` 还原坐标
2. 用 z 方向直方图峰值估计地面高度，避免手写 `z_min`
3. 取 `[地面+0.15, 地面+2.0]` 高度区间的点作为障碍物，投影到栅格
4. 连通域过滤：孤立的 1~3 格噪点删除，墙体和障碍物保留
5. 膨胀 1 格，封住墙体上的小缺口
6. 障碍物外接矩形内部、未被占据的格子填为可通行——Nav2 在 `track_unknown_space: true` 时不会规划进未知区，不填会走不出几步

## 重定位验证

- 配置：`localization:=amcl`，`nav2_params_sim.yaml` 中 `max_particles: 2000`、`min_particles: 500`、`max_beams: 60`、`laser_model_type: likelihood_field`
- 方法：初始位姿故意给偏约 1 m / 40°，推车 1.5 m 并原地转 45°，用 `/amcl_pose` 的协方差判断是否收敛
- 判据：x、y 方差降到 0.05 以下，yaw 方差降到 0.01 量级，同时 RViz 中粒子云收拢、激光扫描贴合墙体

> 说明：该参数文件的 `recovery_alpha_slow` / `recovery_alpha_fast` 均为 0，全局重定位关闭，只能做小偏差局部收敛，偏差不要超过 1 m / 45°。

## 踩坑记录

- `pointlio_mid360_sim.yaml` 里 `pcd_save_en` 默认是 `false`，不改就永远不落盘（`fastlio_mid360_sim.yaml` 那份默认是 `true`，容易混淆）
- Point-LIO 把点云写到源码目录下的 `PCD/scans.pcd`，**目录不存在时写入静默失败、不报错**，必须先 `mkdir -p`
- 收到建图结束信号后连按两次 Ctrl+C 会跳过写盘流程，只按一次再等 20 秒
- `mid360.xacro` 里 `samples` 过大会把实时率拖到 0.2，改成 8000；传感器层的 `<visualize>` 不能直接关掉，关掉会失去触发条件，需要补 `<always_on>true</always_on>`
- 残留 `gzserver` 会占住 11345 端口，表现为节点、话题都在但全无数据；用 `ss -tulpn | grep 11345` 判断，先清场再启动

## 代码结构

```text
task4/
├── README.md
├── tools/
│   └── pcd2pgm.py          # 点云转栅格地图
├── map/
│   ├── rmul_map.pgm        # 栅格地图
│   └── rmul_map.yaml       # map_server 配置
└── docs/
    ├── mapping_rviz.png    # 建图过程截图
    └── map_pgm.png         # 转换后的地图
```
