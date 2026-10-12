# FAST-LIVGO 详细复现流程

## 1. 文档目标

本文档用于指导在 ROS 2 Humble 环境下，以 FAST-LIVO2 为局部里程计基础，复现论文：

> FAST-LIVGO: A Degeneracy-Robust LiDAR-Inertial-Visual-GNSS Fusion Odometry, arXiv:2606.19190v2.

论文原文保存在：

```text
documents/2606.19190v2.pdf
documents/extracted/2606.19190v2/2606.19190v2.md
```

复现目标是实现论文描述的在线紧耦合系统，而不是只将 RTK/PVT 位置作为 FAST-LIVO2 的附加位置约束。最终系统必须包含：

1. FAST-LIVO2 原有 LiDAR、IMU、视觉顺序 ESIKF 更新。
2. 原始 GNSS 观测、星历和卫星状态预处理。
3. 基于 Doppler 速度模的 DTW 时间对齐。
4. local-to-ENU 变换与 IMU-to-GNSS 杆臂联合初始化。
5. GNSS 接收机多星座时钟偏差和时钟漂移状态。
6. 原始 Doppler 观测更新。
7. 固定锚点 Time-Differenced Carrier Phase（TDCP）更新。
8. GNSS 内部 RAIM 完好性检查。
9. 基于 LIVO Hessian 最小特征值的退化检测。
10. LIVO 正常/退化两种模式下的自适应 GNSS 拒真。

本文档把论文明确规定的行为与复现时需要补充的工程假设分开描述。论文没有给出的阈值和噪声参数必须通过数据实验确定，不应被表述为论文原始参数。

## 2. 当前代码基线

### 2.1 仓库职责

当前工作区中的相关仓库如下：

```text
code-space/
|-- FAST-LIVO2/       # ROS 2 FAST-LIVO2 基线
|-- FAST-LIVO2-RTK/   # ROS 2 RTK/PVT 扩展，建议作为工程起点
|-- GVINS/            # GNSS-VIO 初始化、观测筛选和残差参考
`-- gnss_comm/        # GNSS 消息、星历、卫星状态和 SPP 工具
```

建议从 `FAST-LIVO2-RTK` 的 ROS 2 `humble` 分支建立独立开发分支，原因是该仓库已经完成：

- `gnss_comm` ROS 2 依赖接入。
- GNSS 话题和配置入口。
- GeographicLib、GTSAM 和 DTW 相关工程配置。
- HH-LVGO/AGV-LVGO 启动配置。
- ROS 2 bag 和自定义 Livox 消息兼容。

但现有 `FAST-LIVO2-RTK` 不是论文 FAST-LIVGO 实现。当前在线路径消费 `GnssPVTSolnMsg`，状态仍为 19 维，并把 PVT 位置残差加入 LIO 更新；离线后端则使用 GTSAM 对 PVT/RTK 位置进行优化。以下现有逻辑不能作为论文结果验收依据：

```text
src/LIVMapper.cpp               rtk_cbk(), initializeRTK()
src/IMU_Processing.cpp          GetRTKCorrectedState()
src/voxel_map.cpp               StateEstimation(..., rtk_good, rtk_position)
src/optimization.cpp            离线 PVT/RTK GTSAM 后端
```

### 2.2 基线验证状态

当前环境已经确认：

- `gnss_comm` 可以在 ROS 2 Humble 下构建。
- `FAST-LIVO2-RTK` 可以构建。
- `mapping_hh.launch.py` 可以启动并正常退出。
- `libgeographic-dev` 和 `ros-humble-gtsam` 已安装。

构建时存在系统 `/usr/lib` 与 `/usr/local/lib` 中 `libglog` 的潜在搜索路径冲突。开发和 CI 应只保留一个 ABI 一致的 glog 来源，避免出现“构建成功、运行时加载了另一份库”的问题。

## 3. 复现范围与验收层次

复现按三个层次验收，不能只以“节点能启动”作为完成标准。

### 3.1 L1：数学正确

- 坐标、时间、频率、载波相位和时钟单位明确。
- 卫星位置、速度、钟差和钟漂可重复计算。
- Doppler 和 TDCP 残差的解析雅可比通过有限差分检查。
- 状态 `boxplus/boxminus` 和协方差传播通过单元测试。

### 3.2 L2：系统正确

- 没有 GNSS 时，输出应退化为 FAST-LIVO2，轨迹不能出现显著回归。
- 只有 Doppler 时，速度和长距离漂移应得到抑制。
- 加入 TDCP 后，相邻 GNSS 历元间相对位移残差应明显降低。
- GNSS 中断后系统继续依靠 LIVO，恢复后不发生状态跳变或滤波发散。
- GNSS 异常和 LIVO 退化时，自适应模式能够按设计切换。

### 3.3 L3：论文结果接近

- 在论文使用的 M3DGR 序列上计算 ATE RMSE。
- 完成 FAST-LIVO2、Doppler-only、Doppler+TDCP、无拒真、完整系统消融。
- 报告 GNSS 预处理、完好性检查和 GNSS 更新耗时。
- 分析与论文表格差异，区分数据版本、传感器标定、实现假设和参数调优造成的影响。

## 4. 最终系统架构

建议保持 FAST-LIVO2 的 LIO/VIO 实现边界，在其外部增加独立 GNSS 子系统：

```text
Raw GNSS topics
      |
      v
GnssFrontend
  - ephemeris buffer
  - observation validation
  - satellite states
  - SPP / RAIM
  - cycle-slip tracking
      |
      +--------------------+
      |                    |
      v                    v
GnssInitializer        IntegrityMonitor <---- LIVO Hessian
  - DTW time offset       - RAIM
  - ECEF/ENU anchor       - chi-square gate
  - R_EG, p_EG            - mode selection
  - IMU-GNSS lever arm
      |                    |
      +----------+---------+
                 v
             GnssUpdater
              - Doppler
              - TDCP
              - ESIKF
                 |
                 v
FAST-LIVO2 shared state and covariance
```

建议新增以下代码，而不是继续把所有逻辑堆入 `LIVMapper.cpp`：

```text
include/gnss/gnss_types.hpp
include/gnss/gnss_frontend.hpp
include/gnss/gnss_initializer.hpp
include/gnss/gnss_updater.hpp
include/gnss/integrity_monitor.hpp
include/gnss/tdcp_tracker.hpp
src/gnss/gnss_frontend.cpp
src/gnss/gnss_initializer.cpp
src/gnss/gnss_updater.cpp
src/gnss/integrity_monitor.cpp
src/gnss/tdcp_tracker.cpp
```

每个模块先建立纯 C++ 接口和单元测试，再接 ROS 2 订阅。GNSS 数学层不应直接依赖 `rclcpp::Node`，以便使用记录数据做离线测试。

## 5. 坐标、时间与单位约定

这一部分必须在编码前固定，否则最常见的现象是残差数值看似合理，但状态更新方向相反。

### 5.1 坐标系

采用论文记号：

- `{G}`：FAST-LIVO2 局部世界坐标系。
- `{E}`：以 GNSS 锚点为原点的 ENU 坐标系。
- `{I}`：IMU 坐标系。
- `{L}`：LiDAR 坐标系。
- `{C}`：相机坐标系。
- `{r}`：GNSS 天线相位中心。

固定以下变换定义：

```text
R_EG: 将 G 系向量旋转到 E 系
p_EG: G 系原点在 E 系中的位置
t_Ir: GNSS 天线相位中心在 I 系中的杆臂
R_GI: 将 I 系向量旋转到 G 系
p_GI: IMU 在 G 系中的位置
```

GNSS 天线位置为：

```text
p_Er = R_EG * (p_GI + R_GI * t_Ir) + p_EG
```

考虑杆臂后的天线速度为：

```text
v_Gr = v_GI + R_GI * (omega_I x t_Ir)
v_Er = R_EG * v_Gr
```

其中 `omega_I` 必须使用已经减去陀螺零偏的 IMU 角速度，并与当前 GNSS 时间对齐。

### 5.2 时间

内部统一使用秒，推荐统一到 GPS time 或 Unix time 后再进入同步器，禁止混用：

- ROS header Unix/UTC 秒。
- GPS week + time-of-week。
- 带/不带闰秒的时间。

定义并记录：

```text
t_local = t_gnss + delta_t_rI
```

`delta_t_rI` 的正负号必须通过一条人工构造的时间序列测试锁定。DTW 输出不能直接在多处正负取反。

### 5.3 GNSS 数值单位

- 位置：m。
- 速度：m/s。
- 载波频率：Hz。
- 载波相位：cycle。
- 波长：m/cycle。
- Doppler：Hz。
- 接收机时钟偏差状态：m，即论文中的 `c * delta_t`。
- 接收机时钟漂移状态：m/s，即论文中的 `c * delta_dot_t`。

在状态中直接保存米和米每秒，可避免在每个雅可比中重复乘光速。

## 6. 数据准备

### 6.1 必需话题

论文级复现至少需要：

| 数据 | ROS 2 类型 | 作用 |
|---|---|---|
| 原始多星观测 | `gnss_comm/msg/GnssMeasMsg` | 伪距、载波相位、Doppler、C/N0、LLI |
| GPS/BDS/Galileo 星历 | `gnss_comm/msg/GnssEphemMsg` | 卫星位置、速度、钟差、钟漂 |
| GLONASS 星历 | `gnss_comm/msg/GnssGloEphemMsg` | GLONASS 卫星状态 |
| 电离层参数 | `gnss_comm/msg/StampedFloat64Array` | SPP 大气改正 |
| GNSS 时间脉冲 | `gnss_comm/msg/GnssTimePulseInfoMsg` | 时钟同步，可选但推荐 |
| IMU | `sensor_msgs/msg/Imu` | 状态传播与杆臂速度 |
| LiDAR | `fast_livgo/msg/CustomMsg` 或对应 PointCloud2 | FAST-LIVO2 |
| 图像 | `sensor_msgs/msg/Image` | FAST-LIVO2 视觉更新 |
| RTK 真值 | 数据集定义的 Pose/NavSatFix | 仅用于评测，不进入滤波 |

现有 HH-LVGO bag 只有 `GnssPVTSolnMsg`，不能计算论文的 Doppler/TDCP。它应继续用于验证旧 RTK 路径和 ROS 2 数据管线，但不能用于 FAST-LIVGO 最终验收。

### 6.2 数据获取顺序

1. 优先获取论文表 II 使用的 M3DGR 原始数据版本。
2. 确认数据是否包含原始 GNSS 观测和广播星历，而不仅是 RTK ground truth。
3. 若 ROS 1 转 ROS 2，必须注册所有 `gnss_comm` 类型并保留数组字段。
4. 转换后比较每个话题的消息数、起止时间和采样率。
5. 随机抽取至少 10 个 GNSS 历元，检查每颗卫星数组长度一致。
6. 验证每个观测历元都能找到时间有效的星历。

数据审计至少输出：

```text
dataset_name
topic_name
message_type
message_count
start_time
end_time
mean_rate
max_gap
satellite_count_per_epoch
ephemeris_match_rate
valid_psr/cp/dopp_rate
```

### 6.3 数据切分

为缩短开发反馈周期，每个数据集制作三个不改原始 bag 的播放区间：

- `smoke`：20--30 秒，包含系统完成初始化所需运动。
- `debug`：2--3 分钟，包含转弯、加减速和少量遮挡。
- `full`：论文完整序列。

所有单元测试和日常调试优先使用 `smoke`；只有阶段验收时才运行完整序列。

### 6.4 GREAT campus-01 实测审计（2026-07-15）

当前已将 `datasets/GREAT_DATASET/campus-01` 的五个 ZIP 完整解压，保留原 ZIP，并逐文件比对 ZIP 清单、解压后文件大小和 CRC。数据目录约占 21 GiB，解压后磁盘剩余约 43 GiB。审计结果如下：

| 数据 | 数量 | 频率 | GPST TOW 范围 | 结果 |
|---|---:|---:|---:|---|
| LiDAR | 6009 帧 | 9.9147 Hz | 180589.9170--181195.8837 | 连续，无倒序或缺口 |
| 左图 `img0` | 12120 帧 | 20 Hz | 180589.90--181195.85 | 连续 |
| 右图 `img1` | 12120 帧 | 20 Hz | 180589.90--181195.85 | 与左图逐帧同时间戳 |
| MEMS IMU | 60611 条 | 100 Hz | 数据集公共区间全覆盖 | 连续 |
| Tactical IMU | 121221 条 | 200 Hz | 数据集公共区间全覆盖 | 连续 |
| GNSS 原始观测 | 607 个可对齐历元 | 1 Hz | 约 180589--181195 | 连续 |
| RTK ground truth | 6070 条 | 10 Hz | 约 180589--181195 | 连续 |

公共有效区间约为 605 s。GPS TOW 转换到传感器文件名 Unix 时间后的误差小于 0.051 ms，未发现整秒、时区或闰秒错位。真值轨迹长度约 2092 m，平均/最大速度约 3.447/9.124 m/s；状态包含 5834 条 Fixed 和 236 条 Float。`groundtruth.txt` 表头声明了 `NS`，但数据行末尾实际没有有效 `NS` 字段，解析时应按前 27 个有效字段处理。

LiDAR 二进制点格式已验证为 22 字节：

```cpp
struct Point {
  float x;
  float y;
  float z;
  float intensity;
  uint16_t ring;
  float relative_time;
};
```

ring 范围为 0--15，帧内相对时间约为 -0.099533--0.001306 s。转换到当前 FAST-LIVO2 Velodyne 预处理前，必须将时间归一化到扫描起点，并按现有字段约定发布微秒偏移。双目图像均为 `1024x768` 灰度 JPEG。

GNSS 在首选同频信号、`C/N0 >= 35 dB-Hz` 且 LLI 有效的条件下，平均每历元有 18.62 组完整伪距/载波/Doppler，最少 5 组；相邻历元平均有 18.00 组可形成 TDCP，605 对相邻历元中 596 对至少有 5 组。观测到的 GPS/Galileo/GLONASS/BeiDou 共 35 颗卫星均能找到广播星历，最近星历最大年龄分别约为 1212/2100/311/1196 s，均小于 `EPH_VALID_SECONDS=7200 s`。

已使用修正后的 `gnss_comm` RINEX 3.x 解析器进行全文件实测：

```text
aligned epochs              607/607
valid pseudorange SPP       607/607
position RMSE               4.630 m
position max error          30.825 m
valid Doppler velocity      607/607
velocity RMSE               0.684 m/s
velocity max error          8.101 m/s
```

可使用以下命令重复运行解析和数值回归测试；未设置三个真实数据环境变量时，相关用例自动跳过：

```bash
GNSS_COMM_REAL_RINEX_OBS=$PWD/datasets/GREAT_DATASET/campus-01/GNSS_RAW_DATA/campus-day.20O \
GNSS_COMM_REAL_RINEX_NAV=$PWD/datasets/GREAT_DATASET/campus-01/GNSS_RAW_DATA/BRDM00DLR_S_20203010000_01D_MN.rnx \
GNSS_COMM_REAL_GROUNDTRUTH=$PWD/datasets/GREAT_DATASET/campus-01/groundtruth.txt \
colcon test --packages-select gnss_comm \
  --build-base /tmp/gnss_comm_rinex_build \
  --install-base /tmp/gnss_comm_rinex_install \
  --event-handlers console_direct+
```

因此，1 Hz GNSS 不阻塞 FAST-LIVGO 开发：高频状态传播仍由 IMU/LIVO 提供，GNSS 用于低频绝对约束，且本序列的卫星数和 TDCP 连续性充足。限制是高速动态的修正带宽较低，1 s TDCP 对多路径变化和周跳更敏感，当前最大误差也证明必须实现 C/N0/LLI、残差门限和 RAIM 筛选。该序列适合完成前端、初始化、融合和退化鲁棒性验证，但它不是论文表 II 指定的 M3DGR 序列，不能单独作为论文最终精度复现依据。

### 6.5 独立转换包与 smoke bag

数据转换器属于数据集适配层，不应增加 FAST-LIVO2 算法包的依赖和构建时间。当前已建立独立 ROS 2 包：

```text
code-space/great_dataset_tools/
|-- CMakeLists.txt
|-- package.xml
|-- README.md
`-- src/great_dataset_to_rosbag.cpp
```

该包依赖 `gnss_comm`、`rosbag2_cpp`、`rosbag2_compression`、OpenCV 和标准传感器消息，但不依赖 `fast_livgo`。在现有工作空间中接入并构建：

```bash
ln -s $PWD/code-space/great_dataset_tools \
  $PWD/fast_livgo2_rtk_ws/src/great_dataset_tools
cd fast_livgo2_rtk_ws
colcon build --packages-up-to great_dataset_tools --symlink-install
source install/setup.bash
```

生成 30 s smoke bag：

```bash
ros2 run great_dataset_tools great_dataset_to_rosbag \
  --dataset $PWD/../datasets/GREAT_DATASET/campus-01 \
  --output $PWD/../datasets/GREAT_DATASET/campus-01/rosbags/campus01_smoke_30s \
  --duration 30
```

转换器默认输出左目图像、MEMS IMU、LiDAR、GNSS 原始观测、广播星历和电离层参数；`--include-right` 可加入右目图像。输出使用 zstd 文件压缩，已有输出目录会被拒绝，不会覆盖或删除既有 bag。

当前 smoke bag 已生成并完成实际回放/订阅验证：

```text
path                         datasets/GREAT_DATASET/campus-01/rosbags/campus01_smoke_30s
compressed size              508 MiB
duration                     30.083 s
total messages               3960
/imu/data                    3000  sensor_msgs/msg/Imu
/lidar/points                 298  sensor_msgs/msg/PointCloud2
/camera/left/image_raw        600  sensor_msgs/msg/Image
/gnss/observations             30  gnss_comm/msg/GnssMeasMsg
/gnss/ephemeris                26  gnss_comm/msg/GnssEphemMsg
/gnss/glonass_ephemeris         5  gnss_comm/msg/GnssGloEphemMsg
/gnss/iono_params               1  gnss_comm/msg/StampedFloat64Array
```

时间记录范围的 Unix 秒为 `1603764571.817466880--1603764601.900000000`。`ros2 bag info` 按本机 Asia/Shanghai 时区显示为 10:09:31--10:10:01；对应 UTC 为 02:09:31--02:10:01，不是 8 h 时间偏差。GPS TOW 到 UTC 写入时显式扣除了 18 s 闰秒。

LiDAR 回放消息已确认字段为：

```text
x@0 float32, y@4 float32, z@8 float32, intensity@12 float32,
time@16 float32, ring@20 uint16, point_step=22
```

每帧 header 取最早点时刻，`time` 为从最早点开始的微秒偏移，因此当前 `velodyne_handler()` 中 `time / 1000.0` 得到毫秒。MEMS 文件未声明角速度单位，但其最大值约 35，若解释为 rad/s 将不符合车辆运动。将其按 deg/s 与 10 Hz 真值航向差分交叉验证，在转动片段上的相关系数为 `-0.999511`、幅值比例为 `-0.996067`；负号来自航向角顺时针定义与 IMU z 轴右手定则相反。因此转换器将角速度乘 `pi/180` 写入 ROS 的 rad/s，线加速度保持 m/s^2。

压缩 bag 在 ROS 2 Humble 回放时会在 bag 目录留下解压后的 `.db3`，验证完成后可删除该派生文件，仅保留 `.db3.zstd` 和 `metadata.yaml`。

## 7. 阶段 P0：冻结 FAST-LIVO2 基线

### 7.1 建立开发分支

先提交或妥善保存当前未提交修改，再建立新分支。不要在脏工作区中执行可能覆盖文件的切换操作。

```bash
git -C code-space/FAST-LIVO2-RTK status --short
git -C code-space/FAST-LIVO2-RTK switch -c fast-livgo-reproduction
```

### 7.2 建立独立工作空间

```bash
mkdir -p fast_livgo_ws/src
ln -s /absolute/path/to/code-space/gnss_comm \
  fast_livgo_ws/src/gnss_comm
ln -s /absolute/path/to/code-space/FAST-LIVO2-RTK \
  fast_livgo_ws/src/FAST-LIVO2-RTK

git -C code-space/FAST-LIVO2-RTK submodule update --init --recursive

cd fast_livgo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

### 7.3 记录基线

分别运行：

1. GNSS 关闭的 FAST-LIVO2。
2. 当前 PVT/RTK 在线融合。
3. 当前离线 GTSAM 后端。

每次运行固定：

- Git commit。
- YAML 配置副本。
- bag 名称和哈希。
- 启动命令。
- TUM 轨迹。
- 地图输出。
- CPU、内存和各更新耗时。

P0 退出条件：同一配置重复运行两次，轨迹长度、首尾时间、ATE 和处理帧数一致；GNSS 关闭后与原 FAST-LIVO2 没有显著回归。

## 8. 阶段 P1：GNSS 原始观测前端

### 8.1 ROS 2 订阅

在 `LIVMapper` 或独立 ROS 2 adapter 中订阅原始观测和星历，但立即转换为 `gnss_comm` 内部类型：

```text
GnssEphemMsg      -> EphemPtr
GnssGloEphemMsg   -> GloEphemPtr
GnssMeasMsg       -> vector<ObsPtr>
```

参考 GVINS 的 `inputEphem()` 和 `processGNSS()`，按卫星维护：

```text
satellite ID -> ordered ephemeris list
satellite ID -> toe to ephemeris index
satellite ID + frequency/code -> tracking state
```

回调只负责入队，不在 ROS 回调线程内执行 SPP、DTW 或 ESIKF 更新。

### 8.2 星历匹配

对每颗观测卫星：

1. 根据卫星系统确定星历类型。
2. 找到观测时间之前且年龄合法的最近星历。
3. 检查 `health` 和星历有效期。
4. 调用 `sat_states()` 计算卫星位置、速度、钟差、钟漂。
5. 保存观测使用的星历 ID 和年龄，便于调试。

不得使用未来星历，也不能在找不到星历时用零状态继续更新。

### 8.3 频点选择和物理筛选

第一版只使用每个星座的主频，保持与 `gnss_comm::filter_L1()` 和 GVINS 一致。每个信号依次检查：

- 伪距、载波相位、Doppler 对应 `status` bit 是否有效。
- 数组下标是否越界，频率是否大于零。
- `lli == 0`。
- `C/N0 >= 35 dB-Hz`，这是论文明确阈值。
- 卫星仰角 `>= 15 deg`，这是论文明确阈值。
- `psr_std`、`cp_std`、`dopp_std` 是否有限且小于配置上限。
- 星历是否健康。

将每条拒绝原因计数，不能只输出“有效卫星数不足”。

### 8.4 前端输出结构

推荐生成不可变的 `GnssEpoch`：

```text
local_timestamp
gnss_timestamp
observations[]
satellite_states[]
azimuth_elevation[]
spp_solution
raim_status
rejection_statistics
```

P1 退出条件：在一段原始数据上，至少 95% 的有效观测历元能完成星历匹配；所有卫星状态和残差均为有限数；重复回放输出一致。

## 9. 阶段 P2：SPP 与 RAIM

论文使用伪距做 GNSS 内部完好性检查，而不是将伪距直接作为最终滤波观测。

### 9.1 SPP 状态

使用：

```text
x_spp = [p_ECEF(3), clock_bias_GPS, BDS, Galileo, GLONASS]
```

只为当前实际出现的星座建立可观测时钟列。若使用四星座固定 7 维状态，未观测星座必须加零锚定约束，参考 GVINS `anchor_refinement()`。

### 9.2 加权最小二乘

1. 使用 `gnss_comm::psr_res()` 构造伪距残差和雅可比。
2. 权重至少考虑消息提供的 `psr_std`。
3. 可选地按仰角和 C/N0 调整权重，但必须在配置和实验报告中注明。
4. 迭代更新至位置增量和时钟增量收敛。
5. 将 ECEF 解转换为 LLA/ENU，仅用于初始化和完整性检查。

### 9.3 RAIM 建议实现

论文只说明“显著偏离整体分布的伪距残差被强制排除”，未给出具体检验。复现第一版建议：

1. 计算白化残差和全局卡方统计量。
2. 若全局检验通过，接受当前卫星集合。
3. 若失败，排除标准化残差绝对值最大的卫星。
4. 重新求解 SPP 并重复检查。
5. 卫星数不足以提供至少一个冗余观测时，标记 RAIM 不可用，不进行 GNSS 更新。

建议把显著性水平、最大排除数和最小冗余数做成参数。初始可尝试 `alpha=0.001`，但它属于复现假设，不是论文给定值。

P2 退出条件：开阔环境 SPP 轨迹连续；人为加入一个大伪距偏差后 RAIM 能定位并排除对应卫星；排除过程不会访问错误的观测/星历下标。

## 10. 阶段 P3：DTW 时间对齐

### 10.1 输入序列

构造两个速度模序列：

```text
v_livo[k] = norm(v_GI[k])
v_gnss[k] = norm(v_doppler_E[k])
```

GNSS 速度必须由原始 Doppler 求解，不能使用 PVT 消息中的接收机速度替代论文输入。速度模不受坐标旋转影响，因此可在空间对齐前使用。

### 10.2 有效运动窗口

只在满足以下条件的窗口执行 DTW：

- LIVO 和 GNSS 均已连续输出。
- 速度变化具有足够激励，不能全程静止或匀速。
- 有效持续时间达到配置下限。
- GNSS RAIM 通过率达到下限。

对两个序列重采样到共同时间间隔，进行轻度低通或中值滤波。滤波不能引入未补偿的相位延迟。

### 10.3 从 DTW 路径估计单一时间偏差

DTW 会产生多对多匹配路径，而系统最终需要一个标量时间偏差。推荐：

1. 对每个匹配点计算 `t_livo - t_gnss`。
2. 使用中位数作为初值。
3. 用 MAD 排除异常匹配。
4. 对内点计算加权均值作为最终偏差。
5. 记录偏差标准差和有效匹配数。

初始化后默认固定该偏差。若要在线重新估计，必须设置最小重估间隔、变化上限和稳定性检查，防止时间偏差跳变直接污染 TDCP。

P3 退出条件：人工加入已知时间偏移后能够恢复；补偿后速度模相关误差明显下降；重复窗口估计结果稳定。

## 11. 阶段 P4：空间变换和杆臂初始化

### 11.1 初始化变量

论文式（3）联合优化：

```text
X_init = {R_EG, p_EG, t_Ir}
```

残差为：

```text
r_k = p_Er_observed[k]
      - (R_EG * (p_GI[k] + R_GI[k] * t_Ir) + p_EG)
```

GNSS 位置观测用于初始化，可以来自 RAIM 通过后的 SPP。若有 RTK/PVT，可只用于生成更稳定初值或离线诊断，不能替代最终原始观测更新。

### 11.2 求解顺序

1. 使用 SPP 粗定位得到 ENU 原点/ECEF 锚点。
2. 用 Doppler 速度与 LIVO 速度估计初始航向，参考 GVINS `yaw_alignment()`。
3. 用点集配准估计 `R_EG, p_EG` 初值。
4. 使用标定值初始化 `t_Ir`。
5. 对式（3）执行非线性最小二乘联合优化。
6. 检查 Hessian 条件数和杆臂协方差。

如果运动中没有足够旋转，杆臂不可观。此时应固定标定杆臂，只优化 `R_EG, p_EG`，不能让不可观参数自由漂移。

### 11.3 初始化状态机

建议状态：

```text
WAIT_LIVO
WAIT_GNSS
COLLECT_ALIGNMENT
SOLVE_ALIGNMENT
READY
DEGRADED
```

每次状态转换发布原因和质量指标。不能只用“移动距离超过 50 m”作为初始化条件。

P4 退出条件：对齐后的 LIVO 天线轨迹与 SPP/RTK 轨迹方向一致；位置残差无明显时间趋势；杆臂在有激励数据上收敛到物理合理范围。

## 12. 阶段 P5：扩展 ESIKF 状态

### 12.1 24 维误差状态

当前 FAST-LIVO2 状态索引为：

```text
0..2    rotation error
3..5    position error
6       inverse exposure
7..9    velocity error
10..12  gyro bias
13..15  accelerometer bias
16..18  gravity
```

按论文式（1）增加：

```text
19..22  receiver clock biases in meters: GPS/BDS/Galileo/GLONASS
23      common receiver clock drift in m/s
```

因此：

```text
DIM_STATE = 24
```

必须同步修改：

- `StatesGroup` 构造、复制、赋值。
- `operator+`、`operator+=`、`operator-`。
- 初始协方差。
- IMU 传播矩阵和过程噪声。
- LIO/VIO 中所有固定维度矩阵和 block 索引。
- 状态输出和调试日志。

### 12.2 时钟传播

若四星座共享同一接收机频率漂移，离散模型为：

```text
b_clk[k+1] = b_clk[k] + 1_4 * d_clk[k] * dt + w_b
d_clk[k+1] = d_clk[k] + w_d
```

传播矩阵增加：

```text
F(19..22, 23) = dt
```

时钟偏差和漂移过程噪声由配置提供。论文未给数值，需从静态或开阔 GNSS 数据的钟差序列估计初值。

### 12.3 不可观星座处理

若某星座当前没有观测：

- 不为它生成观测行。
- 保留过程传播。
- 协方差随时间增长。
- 长时间重新出现时，对该星座时钟偏差进行重初始化或使用宽松门限。

P5 退出条件：GNSS 全关时 24 维版本与原 19 维 FAST-LIVO2 轨迹基本一致；状态运算单元测试通过；协方差保持有限、对称和半正定。

## 13. 阶段 P6：事件驱动传播和 GNSS 更新时序

论文在 GNSS 观测时刻执行 ESIKF 更新。高动态场景不能简单把 10 Hz GNSS 吸附到最近 LiDAR 帧。

### 13.1 推荐事件顺序

统一按修正后的本地时间排序：

```text
IMU propagation -> GNSS update -> LiDAR update -> visual update
```

若多个传感器时间相同，应固定顺序并写入测试。每个 GNSS 历元：

1. 从上一滤波时刻用 IMU 传播到 GNSS 时刻。
2. 执行 GNSS 前端和 RAIM。
3. 读取最近一次有效 LIVO 退化指标。
4. 构造 Doppler 和可用 TDCP 残差。
5. 执行自适应门控。
6. 执行 GNSS ESIKF 更新。
7. 保存当前 GNSS 后验为下一 TDCP 固定锚点。
8. 继续传播到下一个 LiDAR/图像事件。

### 13.2 禁止的实现方式

- 不得把 GNSS 位置直接写入 `state.pos_end`。
- 不得在 GNSS 到来时把完整协方差清零。
- 不得仅更新位置而忽略时钟、速度和姿态相关雅可比。
- 不得用离线 GTSAM 结果回灌在线滤波来宣称完成 FAST-LIVGO。

P6 退出条件：人工构造的异步 IMU/GNSS 序列中，每次更新时间精确等于 GNSS 时间；改变 GNSS 相对 LiDAR 相位不应产生系统性轨迹偏差。

## 14. 阶段 P7：Doppler 观测更新

### 14.1 观测模型

对卫星 `j` 和主频波长 `lambda_j`，将测得 Doppler 转成距离率。接收机预测速度必须包含杆臂项：

```text
v_Gr = v_GI + R_GI * (omega_I x t_Ir)
v_Er = R_EG * v_Gr
```

预测量由以下部分组成：

- 卫星到接收机的 LOS 相对速度。
- 卫星钟漂。
- 接收机公共钟漂状态。

论文和 `gnss_comm` 对 Doppler 正负号的写法可能不同。实现时必须选择一个内部残差约定，并使用静止接收机和有限差分测试确认：

```text
residual = measured_range_rate - predicted_range_rate
```

不要通过“轨迹看起来更好”判断符号是否正确。

### 14.2 雅可比

Doppler 至少对以下误差状态敏感：

- IMU 速度。
- 姿态，通过 `R_EG * R_GI * (omega x t_Ir)` 杆臂速度项。
- 陀螺零偏，通过校正角速度进入杆臂速度项。
- 接收机时钟漂移。

若第一版固定 `R_EG` 和 `t_Ir`，它们不是滤波状态，只作为常量进入雅可比。

### 14.3 更新方式

把同一历元通过门控的所有 Doppler 残差堆叠为：

```text
r_D, H_D, R_D
```

执行独立 GNSS ESIKF 更新。第一版可以只迭代 1--3 次；每次迭代必须重新计算接收机位置、LOS、杆臂速度和残差。卫星状态在同一历元可固定。

P7 退出条件：

- 静止数据的估计速度接近零。
- 动态数据的 Doppler 速度与 LIVO 速度在对齐后方向一致。
- 解析雅可比相对误差达到预设阈值，例如 `1e-5`。
- Doppler-only 消融不会导致 FAST-LIVO2 精度恶化或滤波发散。

## 15. 阶段 P8：固定锚点 TDCP

### 15.1 卫星跟踪键

TDCP 不能只按卫星 ID 配对，应使用：

```text
(satellite ID, frequency, signal code)
```

每个键保存上一有效 GNSS 后验时刻的：

- 时间戳。
- 载波相位。
- 卫星位置和钟差。
- 接收机后验位置、姿态和对应星座时钟偏差。
- 使用的频率、波长和星历标识。
- 连续跟踪状态。

### 15.2 周跳和断链检测

论文假设相邻历元无周跳，但工程实现必须检测。出现任一条件时丢弃该 TDCP 对并重建锚点：

- `lli != 0`。
- 载波相位有效 bit 失效。
- signal code 或频率发生变化。
- 时间间隔超过配置上限。
- 星历不连续或卫星健康状态改变。
- `lambda * delta_phase` 与 Doppler 积分差显著不一致。

最后一项阈值属于复现假设，需用开阔数据统计确定。

### 15.3 固定锚点残差

论文将上一 GNSS 后验：

```text
X_anchor = {p_GI,k, R_GI,k, clock_bias_k}
```

视为常量，只对当前 `k+1` 状态线性化。对同一卫星：

```text
delta_phase_m = lambda * (Phi[k+1] - Phi[k])
delta_sat_clk = c * (dt_sat[k+1] - dt_sat[k])
delta_range   = rho[k+1] - rho[k]
delta_rcv_clk = b_rcv[k+1] - b_rcv_anchor
```

残差按论文式（11）组合。具体正负号必须与载波相位和卫星钟差约定统一，并通过合成数据验证。

锚点状态在一次迭代更新中保持固定，不能随着当前状态迭代一起变化。当前历元更新完成后，才将后验复制为下一锚点。

### 15.4 TDCP 协方差

初始测量方差至少包含两个历元的载波相位方差：

```text
sigma_tdcp^2 = lambda^2 * (sigma_cp,k^2 + sigma_cp,k+1^2)
```

还应为未建模多路径、相位缠绕和短期大气变化保留噪声下限。不能因为载波相位标称精度很高就把协方差设为接近零。

P8 退出条件：

- 无周跳开阔数据中，TDCP 残差以零为中心且连续。
- 人工加入整数周跳后对应卫星立即断链，不进入更新。
- TDCP 解析雅可比通过有限差分。
- `Doppler+TDCP` 相比 `Doppler-only` 降低相邻历元相对位移误差。

## 16. 阶段 P9：LIVO 退化度量

### 16.1 Hessian 来源

LiDAR 更新已有白化信息矩阵形式：

```text
Lambda_L = H_L^T * R_L^-1 * H_L
```

视觉更新也可取得对应 pose block 信息矩阵。论文称使用 LIVO Hessian 最小特征值，但没有说明 LiDAR/视觉 Hessian 的合并、尺度归一化和阈值。

复现时先同时记录：

```text
lambda_min_lidar
lambda_min_visual
lambda_min_sum_raw
effective_lidar_features
effective_visual_patches
pose_covariance_diagonal
```

不要在没有数据分布的情况下直接复用 YAML 中用于平面判定的 `lio.min_eigen_value`；它不是论文退化门限。

### 16.2 推荐复现路径

1. 第一版严格记录原始 6x6 pose Hessian 最小特征值。
2. 在正常、几何退化、纹理退化和双退化区间标注数据。
3. 比较 raw sum、按观测数归一化、旋转/平移尺度归一化三种指标。
4. 选择对场景最稳定的指标和 `lambda_th`。
5. 使用固定长度滑窗计算均值，符合论文描述。
6. 主实验不额外加迟滞；若加迟滞，应作为工程增强单独报告。

### 16.3 模式输出

发布或记录：

```text
mode = WELL_CONDITIONED | LIVO_DEGRADED
lambda_min_raw
lambda_min_smoothed
lambda_threshold
window_size
```

P9 退出条件：已知退化区间的检测召回率和误报率可统计；模式不会因单帧噪声高频抖动；阈值来自训练/标定序列而不是测试序列逐条调优。

## 17. 阶段 P10：分层完好性与自适应拒真

每个 GNSS 历元先做 RAIM，再根据 LIVO 模式执行第二层门控。

### 17.1 Mode A：LIVO 正常

当滑窗最小特征值均值满足：

```text
lambda_bar >= lambda_th
```

对每颗卫星的 Doppler/TDCP 残差执行创新卡方检验：

```text
NIS_j = r_j^T * (H_j * P_prior * H_j^T + R_j)^-1 * r_j
```

仅当 `NIS_j < chi_square_threshold` 时进入最终堆叠更新。Doppler 和 TDCP 可分别门控，避免一个观测异常导致同卫星另一个健康观测被连带排除。

### 17.2 Mode B：LIVO 退化

当：

```text
lambda_bar < lambda_th
```

停止依赖 LIVO 先验的严格创新门控，接纳所有通过 RAIM、物理筛选和周跳检测的 GNSS 观测。这是论文防止“错误 LIVO 先验把健康 GNSS 全部拒绝”的关键。

Mode B 不是无条件信任所有输入。以下检查始终保留：

- 星历健康。
- C/N0 和仰角。
- 消息有效 bit。
- SPP/RAIM。
- 周跳和时间连续性。
- 最小卫星数量。
- 数值有限性和最大物理残差保护。

### 17.3 故障保护

无论模式如何，若更新后出现以下情况，应拒绝整个 GNSS 历元并恢复先验：

- 状态包含 NaN/Inf。
- 协方差不对称或出现明显负特征值。
- 单次位置、速度或时钟增量超过物理上限。
- GNSS 更新后创新代价反而显著上升。

P10 退出条件：

- LIVO 正常、GNSS 含多路径时，Mode A 能排除异常。
- LIVO 退化、GNSS 健康时，Mode B 不会错误拒绝全部 GNSS。
- GNSS 和 LIVO 同时退化时，系统不发散，并明确输出不可用状态。

## 18. 配置设计

建议新增独立 `gnss` 配置段，避免继续复用旧 `gps` PVT 参数：

```yaml
gnss:
  enable: true
  meas_topic: /ublox_driver/gnss_meas
  ephem_topic: /ublox_driver/gnss_ephem
  glo_ephem_topic: /ublox_driver/gnss_glo_ephem
  iono_topic: /ublox_driver/iono_params
  time_pulse_topic: /ublox_driver/time_pulse_info

  min_cn0_dbhz: 35.0
  min_elevation_deg: 15.0
  max_ephemeris_age_sec: 7200.0
  max_epoch_gap_sec: 0.2

  time_alignment:
    enable_dtw: true
    min_duration_sec: 30.0
    min_speed_mps: 0.5
    resample_period_sec: 0.1

  extrinsic:
    estimate_lever_arm: true
    t_imu_receiver: [0.0, 0.0, 0.0]

  clock:
    bias_init_std_m: 100.0
    drift_init_std_mps: 10.0
    bias_random_walk: TBD
    drift_random_walk: TBD

  raim:
    alpha: 0.001
    max_exclusions: 3
    min_redundancy: 1

  tdcp:
    enable: true
    max_gap_sec: 0.2
    phase_doppler_consistency_m: TBD
    noise_floor_m: TBD

  degeneracy:
    window_size: TBD
    lambda_threshold: TBD

  gating:
    chi_square_probability: 0.99
    max_position_increment_m: TBD
    max_velocity_increment_mps: TBD
```

所有 `TBD` 必须通过标定或实验填写。程序启动时若关键参数仍为非法值，应直接报错，不要静默使用任意默认值。

参数来源必须随配置归档：

| 参数 | 来源 | 使用要求 |
|---|---|---|
| `min_cn0_dbhz=35` | 论文明确给出 | 主复现实验固定 |
| `min_elevation_deg=15` | 论文明确给出 | 主复现实验固定 |
| `alpha=0.001` | 本文建议的 RAIM 初值 | 通过标定序列验证后冻结 |
| `chi_square_probability=0.99` | 本文建议的门控初值 | 报告自由度和最终取值 |
| `max_ephemeris_age_sec` | 星座接口规范和数据频率 | 应按星座分别配置 |
| `max_epoch_gap_sec` | GNSS 实际采样周期 | 应由 bag 审计结果确定 |
| 时钟过程噪声 | 数据统计 | 使用静态/开阔数据估计 |
| TDCP 噪声与周跳阈值 | 数据统计 | 使用独立标定序列估计 |
| Hessian 窗口和阈值 | 退化标注数据 | 测试序列上不得重新调优 |

## 19. 测试计划

### 19.1 单元测试

至少添加：

```text
test_state_boxplus.cpp
test_clock_propagation.cpp
test_ephemeris_selection.cpp
test_coordinate_transform.cpp
test_doppler_residual.cpp
test_doppler_jacobian.cpp
test_tdcp_residual.cpp
test_tdcp_jacobian.cpp
test_cycle_slip.cpp
test_raim.cpp
test_dtw_offset.cpp
test_integrity_mode.cpp
```

雅可比测试采用中心有限差分，并分别扰动姿态、位置、速度、陀螺零偏、时钟偏差和时钟漂移。

### 19.2 合成测试

构造已知轨迹、卫星位置和时钟状态，生成无噪声/带噪声 Doppler、载波相位。覆盖：

- 静止接收机。
- 匀速直线。
- 带旋转的杆臂运动。
- 多星座时钟偏差。
- 单卫星周跳。
- 单卫星大伪距异常。
- GNSS 时间偏差。

### 19.3 回放测试

每次合并前运行 `smoke` bag，检查：

- 初始化是否完成。
- 有效卫星数。
- RAIM 排除数。
- Doppler/TDCP 残差均值和标准差。
- 模式切换次数。
- GNSS 更新接受率。
- 是否出现 NaN、时间回退或队列无限增长。

### 19.4 故障注入

离线修改观测副本或在 adapter 中启用测试开关，注入：

- 1--10 秒 GNSS 中断。
- 固定/随机时间偏差。
- 单颗和多颗卫星伪距偏差。
- Doppler 离群值。
- 载波相位整数周跳。
- C/N0 突降。
- LiDAR 几何退化区间。
- 图像全黑或运动模糊区间。

## 20. 实验与消融矩阵

每个论文序列至少运行：

| 编号 | 配置 | 目的 |
|---|---|---|
| A | FAST-LIVO2 | 局部里程计基线 |
| B | FAST-LIVO2 + legacy PVT | 对照现有 RTK 方案 |
| C | FAST-LIVO2 + Doppler | 验证速度紧耦合 |
| D | FAST-LIVO2 + Doppler + TDCP | 验证高精度相对约束 |
| E | C/D + RAIM，关闭自适应拒真 | 对应拒真消融 |
| F | 完整 FAST-LIVGO | 最终结果 |

输出指标：

- ATE RMSE、均值、中位数和最大值。
- 相对位姿误差 RPE。
- 速度 RMSE。
- 最终位置漂移。
- GNSS 中断前后状态跳变量。
- 地图重影/重复结构的定量或可重复可视化。
- 每类 GNSS 观测接受率和拒绝原因。
- LIVO 退化检测混淆矩阵。
- 初始化时间和成功率。
- LiDAR、视觉、GNSS 预处理、RAIM、GNSS 更新耗时。

论文报告 GNSS 总开销约 11.78 ms、完整系统约 37.77 ms。复现应报告相同模块划分，但不能在硬件不同的情况下只比较绝对毫秒数；同时给出 CPU 型号、线程数和编译类型。

## 21. 推荐开发顺序与提交边界

按以下顺序提交，每个提交都应可构建：

1. `test: add baseline replay and metric scripts`
2. `feat: add raw GNSS subscriptions and epoch types`
3. `feat: add ephemeris matching and satellite-state frontend`
4. `feat: add SPP and RAIM integrity checks`
5. `feat: add DTW temporal alignment`
6. `feat: add ENU and lever-arm initialization`
7. `refactor: extend ESIKF state with GNSS clock states`
8. `feat: add event-driven GNSS update scheduling`
9. `feat: add Doppler ESIKF update`
10. `feat: add TDCP tracking and update`
11. `feat: expose LIVO Hessian degeneracy metric`
12. `feat: add adaptive dual-mode GNSS rejection`
13. `test: add fault injection and ablation pipeline`
14. `docs: record parameters and reproduction results`

不要在一个提交中同时完成状态扩维、调度改写和两个 GNSS 观测模型；这会使数值回归无法定位。

## 22. 阶段里程碑

### M0：基线可重复

- 原 FAST-LIVO2 和当前 RTK 版本完整回放。
- 数据、配置、轨迹和耗时归档。

### M1：GNSS standalone

- 原始 GNSS 数据闭环。
- SPP、RAIM、Doppler 速度、TDCP 残差可离线输出。
- 所有数学雅可比测试通过。

### M2：Doppler 紧耦合 MVP

- 24 维状态和时钟传播完成。
- GNSS 精确时刻更新完成。
- Doppler-only 可稳定运行完整序列。

### M3：完整 FAST-LIVGO

- TDCP 固定锚点完成。
- 退化检测和双模式拒真完成。
- GNSS 中断、周跳和异常注入不发散。

### M4：论文复验

- M3DGR 定量表格。
- 全部消融和实时性能统计。
- 公开所有配置、版本、数据转换和评测命令。

## 23. 已知风险与处理策略

### 23.1 原始数据不可用

这是当前最高优先级风险。没有原始观测和星历时，只能完成 PVT 融合，无法验证 Doppler/TDCP。数据准备必须在状态扩维前完成。

### 23.2 论文参数缺失

周跳阈值、时钟噪声、TDCP 噪声下限、Hessian 滑窗和退化阈值未给出。处理方式是：

- 配置化。
- 用训练/标定序列确定。
- 在测试序列上冻结。
- 在复现报告中列出。

### 23.3 空间初始化不可观

直线匀速运动不足以联合估计完整旋转和杆臂。应检查激励和 Hessian 条件数；不可观时固定离线标定杆臂。

### 23.4 固定锚点一致性

论文将上一后验视为常量，避免状态扩增，但忽略了锚点与当前状态的相关性。复现主线按论文实现，同时记录 NEES/NIS 和协方差一致性；不得擅自改成滑窗后仍称为同一方法。

### 23.5 过强 TDCP 权重

TDCP 标称精度高，但城市环境中的多路径变化会造成厘米到米级异常。必须保留噪声下限、RAIM、周跳检测和自适应门控。

### 23.6 旧 RTK 路径干扰

建议增加明确开关：

```text
fusion_mode = livo | legacy_pvt | fast_livgo
```

`fast_livgo` 模式下禁止调用 `GetRTKCorrectedState()`、PVT LIO 残差和离线 GTSAM 回灌。旧路径只作为对照实验保留。

## 24. 完成定义

只有同时满足以下条件，才认为 FAST-LIVGO 复现完成：

1. 使用原始 GNSS Doppler 和载波相位，而不是 PVT 位置替代。
2. 24 维状态和时钟传播已进入同一 ESIKF 协方差。
3. GNSS 在自身时间戳执行更新。
4. DTW、空间对齐和杆臂初始化均有质量检查。
5. TDCP 使用上一 GNSS 后验固定锚点，并处理断链/周跳。
6. RAIM 和 LIVO 退化双模式拒真可通过故障注入验证。
7. GNSS 关闭时不破坏 FAST-LIVO2 基线。
8. M3DGR 完整序列和消融实验可用单一命令重复执行。
9. 所有参数、数据版本、代码 commit 和评测命令已归档。
10. 结果与论文差异有可验证解释，而不是只展示一张 RViz 地图。

## 25. 首轮执行清单

开始编码前，依次完成：

- [ ] 提交或保存 `FAST-LIVO2-RTK` 当前未提交修改。
- [ ] 建立 `fast-livgo-reproduction` 分支。
- [ ] 固定 ROS、编译器、Eigen、Sophus、GTSAM 和 glog 版本。
- [ ] 下载并审计包含原始 GNSS 的 M3DGR 数据。
- [ ] 验证 ROS 2 bag 中所有 `gnss_comm` 话题可反序列化。
- [ ] 生成 smoke/debug/full 数据区间清单。
- [ ] 运行并归档 FAST-LIVO2 基线。
- [ ] 建立 GNSS standalone 测试可执行程序。
- [ ] 先完成卫星状态、SPP、RAIM 和 Doppler 速度输出。
- [ ] GNSS standalone 验收后再扩展 FAST-LIVO2 状态。

该顺序的核心原则是：先证明 GNSS 观测本身正确，再允许它修改 FAST-LIVO2 的共享状态和协方差。
