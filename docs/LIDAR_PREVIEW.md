# 未标定的雷达参考滚动预览

入口 `lidar-preview` 只订阅已有 Point-LIO 数据，以里程计参考点为中心生成回波网格，供原厂遥控器手动测试时观察。它不是 Go2 机身地图，不使用机身外参和碰撞尺寸，不调用 TerrainMap 地面／坡度／可通行性分类，不规划或输出速度，不调用 Unitree SDK，不改变控制权。

默认窗口为 6×6 米、分辨率 0.12 米，位置按格子对齐滚动，保留最近 3 秒的回波；最多保存 60000 个点。只有实际点落入的格子才有观测，每格最低／最高回波不代表地面／障碍顶部。桌面、头架和机身回波仍可能在图中，不自动剔除。未知不补成可通行区域。此入口用于检查注册点云、窗口迁移和断流；正式 TerrainMap 仍需测量与标定后另行验收。

## 启动

先确认原厂遥控器已恢复正常；遥控异常时保持静态检查。机载上先运行原雷达驱动与 Point-LIO（以下两条在分别加载 Noetic 和 sound_lio_ws 的终端运行）：

```bash
roslaunch livox_ros_driver2 msg_MID360s.launch
roslaunch point_lio mapping_mid360.launch rviz:=false
```

然后在机载 NoMachine 的两个终端分别运行：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav lidar-preview
```

```bash
source /opt/ros/noetic/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
rviz -d /home/unitree/go2_navigation_github/config/lidar_preview.rviz
```

无需填写或伪造标定文件。正式 `mapping` 的未标定保护保持原样。预览独占节点名 `go2_lidar_reference_preview`；不与另一个同名预览重复启动。停止只需在预览终端按 Ctrl+C；它不发送停止或其它机器人指令。前台启动的 SSH 会话断开可能结束程序；需要后台运行时明确使用 nohup，并从启动日志读取 PID 后针对该 PID 停止，不用 pkill 泛杀。

## RViz 的含义

- 黄色方框：滚动窗口边界，跟随 Point-LIO 参考点的世界 XY；不代表机器人外形。
- 彩色点：最近 3 秒内、当前窗口中的真实注册回波，按世界 Z 着色。显示 Decay Time 保持 0，保留时间由数据端管理。
- 橙色箭头：Point-LIO 里程计参考点及朝向，不是标定后的 Go2 机身。
- 黄色文字：始终显示 UNCALIBRATED / NO CONTROL；过期时显示 STALE / WAITING，点云清空。
- `Lowest return per cell - NOT GROUND` / `Highest return per cell`：可单独启用观察原始回波高度范围。
- `Observed cells - NO FREE SPACE CLAIM`：可启用查看有回波的格子；其余无观测区域为空缺，不代表可走。

Fixed Frame 使用输入的真实世界系，当前默认 `camera_init`。视角默认固定在该世界系，可中键平移；若已有 Point-LIO TF 链，可在 Views → Target Frame 选其真实参考帧（当前输出 child_frame_id 为 `body`），让视角跟随。这里的 `body` 名称不构成 Go2 机身标定证明。不要通过把 Fixed Frame 改成运动中的参考帧来判断固定物体是否漂移。

## 输出和日志

所有输出均在 `/go2/lidar_preview/`：`points`、`lowest_returns`、`highest_returns` 为 PointCloud2；`window`、`warning`、`observed_cells`、`reference_pose` 为 Marker；`status` 为诊断 JSON。没有占据概率、可通行地图、Go2 机身位姿或控制输出；不向 `/go2/perception/` 写正式地图。

`LOG` 目录保存完整命令、提交、设置、标定模板原样副本、终端输出和 `perception/{inputs.csv,status.jsonl,report.json}`。标定缺项始终保留，status 的 `navigation_usable` 恒为 false。聚合点云 header 使用最近接受帧的源时间，历史点最多保留 3 秒，并不表示所有点同时采集。`map_data_valid` 在本模式仅表示回波预览时效条件满足。数据超过 0.5 秒、快照工作线程超过 0.3 秒时撤下点与参考箭头，保留原始源时间；位姿跳变、参考帧变化、ROS 时钟回退要求重启预览。

需要回放时，在另一个已加载 Noetic 的终端保存原始输入（文件留在本地，不上传 Git）：

```bash
mkdir -p /home/unitree/go2_navigation_github/local
rosbag record -O /home/unitree/go2_navigation_github/local/lidar_preview.bag /cloud_registered /aft_mapped_to_init /tf /tf_static
```

先静止看墙面，再由操作者用原厂遥控器低速前进、退回和转向。看固定物体是否保持世界位置、窗口是否跟随、边缘新区域是否为空缺；实际误差应结合已知距离或录像。遥控失效未查明时不做移动测试。
