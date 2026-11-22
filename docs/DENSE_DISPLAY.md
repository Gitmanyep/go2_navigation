# 原始 Livox 稠密显示分支

`lidar-dense-preview` 读取原始 `/livox/lidar` CustomMsg 和已有 `/aft_mapped_to_init`，单独提供较密的世界系回波给原来的未标定滚动预览。它不修改雷达驱动或 Point-LIO，不写 ROS 参数、不输出运动或接管请求，也不绕过正式地图的标定保护。

## 启动与回退

先让原 Livox 驱动和 Point-LIO 按原方式运行，停止本仓库已有 `lidar-preview`，避免同名显示节点重复启动。然后在机载终端运行：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav lidar-dense-preview
```

继续使用原来的跟随视图：

```bash
source /opt/ros/noetic/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
rviz -d /home/unitree/go2_navigation_github/config/lidar_preview.rviz
```

前台 Ctrl+C 会结束本次转换器和预览两个子进程；不停止原雷达或定位，更不会发送机器人停止指令。后台运行通过既有 go2-test 记录器启动，按对应 run 目录停止；SSH 退出不会结束该后台记录器。不要泛用 pkill。

若需对比稀疏注册点云，停止稠密入口后重新运行 `lidar-preview`。两个入口使用同一个预览节点／显示话题，不同时运行。切换只影响显示。

## 配准来源与限制

当前现场 Point-LIO 源码表明，`camera_init → body` 里程计表示内部 IMU 参考点；原始 Livox 点先用 `mapping/extrinsic_R`、`mapping/extrinsic_T` 转到该参考系。转换器读取 `/laserMapping` 的参数快照，每 2 秒复核，不把它们当 Go2 机身外参。

```text
p_world(t) = R_world_IMU(t) × (R_IMU_lidar × p_lidar + t_IMU_lidar) + t_world_IMU(t)
```

按原始消息 header／timebase 和每个点的 offset_time（纳秒）计算源时间，对里程计平移做线性插值、旋转做 SLERP。必须有前后位姿包围点时间，最大位姿间隔 0.15 秒，不外推，也不使用单个最新位姿变换整帧。原始点中的非有限值、超出显示距离、无效 Livox tag／line 会被过滤。

这仍是**使用约 10 Hz 里程计的近似逐点去畸变**，没有复现 Point-LIO 的高频 IMU 状态估计。快速转向、腿部震动或定位漂移可能形成重影，不能用该显示证明精确标定、同步或运动性能。内部外参在线估计开启、参数改变、坐标变化、定位跳变或时钟回退会拒绝继续，需要先查明原因再重启。硬件与机身标定状态仍未验收。

## 点数与负载

原定位继续使用已有 0.5 米表面体素。显示分支默认在世界系以 **0.04 米**体素合并重复点，最多约 10 Hz 发布，原始扫描队列最多 4 帧；有更新的可配对帧时跳过旧显示帧，避免长队列积压。滚动预览仍为 6×6 米，最多保存 3 秒及 60000 点；达到点数上限时实际保留时间会缩短。稠密显示增加的 CPU、ROS 传输和渲染负载仍需机载测量，软件测试不代表硬件已跟得上。

显示子进程将数学库限制为单线程，只解码已有位姿覆盖的最新原始扫描；原始消息按已核对的 CustomMsg MD5 和二进制布局通过 NumPy 解码，先裁剪局部窗口再做显示体素；转换器与预览通过 NumPy 二进制读写 PointCloud2，避免多帧解码和 Python 逐点打包的积压。发布 `/go2/lidar_dense/registered` 和 `/go2/lidar_dense/status`，预览仅将前者用于显示，正式导航入口不订阅它。没有旧点时不补假点，不把未知格标为可走。

输出点云 header 是该原始扫描的结束源时间，不是发布时刻；逐点采集跨度和最早点年龄另有记录。是否过期按扫描开始源时间计算，不用较晚的结束时间延长有效期。源点或里程计超过 0.5 秒后发布空点云，保持旧源时间，让下游预览撤下显示。

## 日志与验证

统一 `LOG` 下：

- `perception/dense_terminal.log`、`preview_terminal.log`：两个子进程的输出。
- `perception/metadata.json`、`report.json`：启动参数、PID、退出状态与隔离声明。
- `perception/dense/metadata.json`：实际内部外参快照及参数；`frames.csv` 保存扫描起止、原始／输出点数和投影用时；`status.jsonl` 保存丢帧、时效和近似方法说明。
- `perception/preview/`：原预览的时效、已观测格与未知格记录。

先静止比对原始点数、稠密输出、最终窗口点数和源年龄，观察墙面是否与原注册点云一致；再在遥控器恢复正常后由操作者低速移动。转换器不启动或试探运动。不通过增加超时阈值掩盖原定位积压。

## 可选前方 130°模式

全周默认配置保留。使用 `config/lidar_sector_preview.json` 可对显示分支启用前方左右各 65°精细点和 1.5 米近距离全周粗点；正式地图另有机身参考配置。启动、边界和限制见 [SECTOR_ROI.md](SECTOR_ROI.md)。
