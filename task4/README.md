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

实测 `/livox/lidar/pointcloud` 约 8.8 Hz、`/imu/data` 约 80~85 Hz、`/Odometry` 约 8.9 Hz。

> `mid360.xacro` 本身不含 IMU 传感器，`/livox/imu` 恒无数据；Point-LIO 订阅的是仿真侧 `complementary_filter` 推出的 `/imu/data`。

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

工具栏 **2D Pose Estimate** 在场地内点选一个与真实位置有偏差的位姿。AMCL 的 `update_min_d` 为 0.25 m、`update_min_a` 为 0.2 rad，必须移动超过这个量才会触发更新。

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

### 参数

`localization:=amcl`，`rm_nav_bringup/config/simulation/nav2_params_sim.yaml`：

| 参数 | 值 | 说明 |
|---|---|---|
| `laser_model_type` | `likelihood_field` | 似然场模型 |
| `max_beams` | 90 | 每帧参与打分的激光束数 |
| `sigma_hit` | 0.12 | 似然场高斯宽度（m） |
| `z_hit` / `z_rand` | 0.5 / 0.1 | 命中 / 随机项权重 |
| `alpha1~4` | 0.1 | 运动模型噪声 |
| `update_min_d` | 0.25 | 位移超过 0.25 m 才更新 |
| `update_min_a` | 0.2 | 转向超过 0.2 rad 才更新 |
| `max_particles` / `min_particles` | 2000 / 500 | 粒子数 |
| `recovery_alpha_slow` / `fast` | 0 / 0 | **全局重定位关闭** |

### 测试方法

真值取 Point-LIO 输出的 `odom → base_link_fake` 变换（建图阶段已确认与地图对齐）。两种注入方式对比：

- **真值起步**：偏差置 0，只验证滤波器本身不发散
- **带偏差起步**：在真值上叠加 x/y 各 0.35 m（合成 0.495 m）偏移、yaw 15°

用 `/initialpose` 注入，协方差取 x/y = 0.25 m²、yaw = 0.068 rad²。**协方差不能全 0**：全 0 时 2000 个粒子叠在同一个位姿上，权重完全平坦，滤波器退化成纯里程计外推，什么都收敛不了。

偏差必须**相对真值**注入。直接给一个固定绝对坐标，车一动偏差就会自己漂大，测出来的是假数据。

### 结果

**真值起步（偏差 0）**

| 采样 | 位置误差 (m) | yaw 误差 (°) | σ_xy (m) | σ_yaw (°) |
|---|---|---|---|---|
| T0 | 0.026 | 0.2 | 0.478 | 14.2 |
| T1 | 0.052 | -1.4 | 0.473 | 14.7 |
| T2 | 0.049 | -1.5 | 0.473 | 14.7 |
| T3 | 0.150 | 0.2 | 0.482 | 15.0 |

位置误差 0.026 → 0.150 m，yaw 误差全程 ≤1.7°，**满足 ≤0.2 m / ≤5° 的判据**。这组数据同时证明地图与真实世界严格对齐：扫描能稳定匹配到墙面上。

但 `σ` 从 0.478 到 0.482 基本没动——这就是后面收敛不动的第一个线索（见"为什么收敛不动"）。

**带偏差起步（0.495 m / 15°）**

| 采样 | 位置误差 (m) | yaw 误差 (°) |
|---|---|---|
| T0 | 0.496 | 15.7 |
| T1 | 0.397 | 25.1 |
| T2 | 0.499 | 13.7 |
| T3 | 0.478 | 5.3 |
| T4 | 0.372 | 13.9 |
| T5 | 0.399 | 9.3 |
| T6 | 0.410 | 14.0 |
| T7 | 0.411 | 16.0 |

位置误差 0.496 → 0.411 m，总体在收（前段最低到 0.372 m），但**来回摆动、不单调**；yaw 误差中段一度收到 5.3°，末段又回到 16.0°。`σ_xy` 0.490 → 0.464、`σ_yaw` 14.8 → 17.5，降幅很小。

**末态 0.411 m / 16.0°，未满足 ≤0.2 m / ≤5° 判据。**

### 结论

1. **真值附近定位稳定**：误差 0.15 m、yaw 1.7° 以内，地图、TF 链、AMCL 参数链路都是通的。
2. **从 0.5 m / 15° 偏差注入无法收敛到位**：只能从 0.496 m 收到 0.411 m。
3. 由此可以给出该场景下 AMCL 的**吸引域小于 0.5 m / 15°**。`recovery_alpha_slow/fast = 0` 时 AMCL 没有全局重定位能力，只能做局部爬山，初值必须落在吸引域内；超出这个范围，靠推车走动拉不回来。

### 为什么收敛不动

粒子滤波的收敛靠的是**分数差**：扫描对好位姿给高分、对差位姿给低分，重采样才能把粒子云压过去。如果所有粒子的分数都差不多，重采样等于随机抽签。这里三条同时不满足：

- **判别力不足**：真值起步那一组的 `σ` 全程不动（0.478 → 0.482），说明峰附近似然面太平——粒子云已经罩住真值了，但没有梯度把它们往峰顶压。
- **可观测性不足**：场地可走区域以长直通道为主，横向偏差和小幅朝向偏差在扫描里几乎被"两堵平行墙的间距差"吸收掉。另外底盘角速度被 `fake_vel_transform` 替换成固定 5 rad/s，`teleop` 按 `j` 只会原地疯转，**没法精确转 15°**，yaw 只能靠平移时的扫描图案变化来约束，信息量天然弱。
- **迭代次数不足**：`update_min_d` 取 0.25 m，一轮测试只触发 6~8 次更新。概率质量要靠一次次重采样滚过去，次数太少来不及。

> `update_min_d` 不能调小了硬凑——运行时改它会触发 `reinit_laser`（重建激光订阅器和索引，但不重置 `pf_odom_pose_`），下一帧算出异常位移，粒子云被运动模型直接推飞（实测爆到 x = -71 m，再爆到 x = 954 m、方差 9.8e5）。实测已回滚到 0.25。

### 判据

- 位置误差 ≤ 0.2 m，yaw 误差 ≤ 5°
- 且 `σ` 显著下降（标准差口径，单位 m / rad），RViz 中粒子云收拢、扫描贴合墙体

### 后续可改进方向

1. 先提高 `/scan` 的信息量（有效回波数、距离覆盖范围），这是判别力的根本来源
2. 测试路线要包含横向大位移并经过角点/异形障碍，让被估自由度"看得见"
3. 以上两条做到之后，最后才动粒子数、`sigma_hit` 这类参数
4. 若仍不满足，可切到 ICP：scan-to-map 直接把当前扫描配准到 `scans.pcd`，不依赖粒子爬山，对初始偏移的容忍度通常更大；代价是需要一份 `PCD/RMUL.pcd` 先验点云和 `icp_registration_sim.yaml`

## 踩坑记录

- `pointlio_mid360_sim.yaml` 里 `pcd_save_en` 默认是 `false`，不改就永远不落盘（`fastlio_mid360_sim.yaml` 那份默认是 `true`，容易混淆）
- Point-LIO 把点云写到源码目录下的 `PCD/scans.pcd`，**目录不存在时写入静默失败、不报错**，必须先 `mkdir -p`
- 收到建图结束信号后连按两次 Ctrl+C 会跳过写盘流程，只按一次再等 20 秒
- `mid360.xacro` 里 `samples` 过大会把实时率拖到 0.2，改成 8000；传感器层的 `<visualize>` 不能直接关掉，关掉会失去触发条件，需要补 `<always_on>true</always_on>`
- 残留 `gzserver` 会占住 11345 端口，表现为节点、话题都在但全无数据；用 `ss -tulpn | grep 11345` 判断，先清场再启动
- **时间基不一致会让 AMCL 静默失效**：`/scan` 的时间戳用仿真时间（`stamp.sec` 约 1000 量级），而 TF 用现实时间（1.79e9 量级），AMCL 的 `tf2_ros::MessageFilter` 按扫描时间戳查不到变换，每条扫描被直接丢弃，表现为 `/amcl_pose` 一条都不发。修法：`ros2 param set /laserMapping use_sim_time true`，并确认 `/amcl`、`/fake_vel_transform` 也是 `true`
- **地板混进激光扫描**：`pointcloud_to_laserscan` 的 `min_height: -1.0 / max_height: 0.1` 会把地面点整包进来，`base_link_fake → livox_frame` 测得雷达离地只有 0.175 m，扫描于是退化成车周半径约 0.35 m 的一圈"假墙"。改成 `min_height: -0.05 / max_height: 0.15`
- **IMU 话题错配**：`mid360.xacro` 不含 IMU 传感器，`/livox/imu` 恒无数据，Point-LIO 在等它 → 永不初始化、`odom` 永不出现。`pointlio_mid360_sim.yaml` 的 `imu_topic` 改为 `/imu/data`
- **重复 `complementary_filter` 节点会毁掉 IMU**：曾查出 `/imu/data` 有 9 个同名发布者，300 条里 265 条时间戳倒退（88%）。Point-LIO 每秒 `imu loop back, clear deque` 几十次，速度估计每帧重初始化，静止也漂、一动就飞到两万米。清场时必须把 `complementary_filter` / `imu_complementary` 一起 `pkill`
- **TF 断成两棵树**：URDF 根是 `dummy`，LIO 又从另一侧给 `base_link` 挂父节点，`base_link` 出现两个父节点、根分裂。补两节静态变换 `odom → lidar_odom`、`odom → dummy` 即可，不要挂 `base_link_fake → livox_frame`（那会再造一个双父节点）

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
