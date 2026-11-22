# 130°前方精细处理与近距离全周保留

这是可选的本地软件配置。保留原有全周入口；本次未连接或更新已关机的机载电脑，未进行新的实机验收。

## 处理范围

- 前方左右各 65°，水平距离 3 米内，使用 0.04 米三维体素。
- 前方扇区以外，水平半径 1.5 米内，使用 0.12 米三维体素保留全周回波。
- 两者之外不参与当前输出；正式地图的对应栅格为未知，不能作为无障碍区域。
- 不额外裁剪垂直高度，保留地面、机身高度和悬空结构的原始三维回波。高度阈值、地面分类及可通行性仍由正式建图模块处理。

距离均按当前参考点的水平 XY 距离计算。方位角使用参考 +X 在水平世界平面上的投影，不以世界坐标 +X 为恒定正前方，不因坡面的整体抬升而直接删掉高处点。参考前向接近竖直时拒绝猜测方向。

默认参数位于 `config/lidar_sector_preview.json` 和 `config/perception_sector.json` 的 `spatial_roi` 中：

```json
{
  "frame": "pointlio_reference",
  "front_angle_deg": 130.0,
  "front_range_m": 3.0,
  "near_radius_m": 1.5,
  "fine_voxel_m": 0.04,
  "near_voxel_m": 0.12
}
```

`front_angle_deg` 是总角度。正式机身地图配置的 `frame` 为 `body`；两个参考类型不能互换。未知键、错误单位范围、非有限值和非法尺寸会拒绝启动，避免配置拼写错误后静默退回全周。

## 下次在机载启动未标定预览

先同步本地仓库到机载，再按原方式启动 Livox 驱动和 Point-LIO。结束已有全周/扇区预览后，在新的机载终端运行：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav lidar-dense-preview --config /home/unitree/go2_navigation_github/config/lidar_sector_preview.json
```

在已经加载 Noetic 环境、连接同一个 ROS master 的机载终端打开：

```bash
rviz -d /home/unitree/go2_navigation_github/config/lidar_sector_preview.rviz
```

继续使用越肩跟随视角，新增青绿色的 130°扇区与 1.5 米圆环边界。边界仅标出处理范围，不能理解成已经观测到地面或空间可通行。断流/超时后撤下点云和扇区边界。

此时使用 Point-LIO 内部参考点的位置与航向，**尚未证明与狗头方向、机身中心一致**。仍是未标定回波预览，不做地形分类、机身碰撞验收或自主运动。原始 Livox 点先按逐点时间近似配准到世界系，然后在显示体素与滚动网格计算之前裁剪；不把最新位姿当整帧位姿，也不提前按未补偿的原始点错误裁掉转弯中的回波。

累计的历史点会按最新位置和朝向再次裁剪。转身后，当前范围以外的旧点不会留在画面上造成仍在观察的假象；保留下来的点仍保有原世界位置和源时间。

回退全周显示：Ctrl+C 结束本次预览，再运行原命令，使用原 RViz 配置：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav lidar-dense-preview
rviz -d /home/unitree/go2_navigation_github/config/lidar_preview.rviz
```

两种配置共用显示话题与节点名，不能同时运行。

## 正式机身地图

只有完成真实外参、机身/头架尺寸和里程计参考确认后，才能使用：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav mapping --config /home/unitree/go2_navigation_github/config/perception_sector.json --calibration /home/unitree/go2_navigation_github/local/calibration.json
```

该分支先通过原标定保护、配准检查和实体机身剔除，再按机身位置/航向裁剪和体素保留，然后接入现有 TerrainMap。最新机身范围外的历史栅格在发布时设为未知，高程和障碍点同步裁剪。输入点少、近处地面支撑不足时保持未知/阻止通行，不以插值补出安全地面。

0.12 米粗体素保留一个真实返回点，不生成体素中心或填满自由空间，也不保证体素内每个细小结构均保留。后续接运动规划前，必须把体素代表点误差纳入障碍膨胀和机身扫掠范围检查；还需验证细树枝、台阶和横向净空。本次没有完成这些物理验收。1.5 米范围也没有经过制动距离验收，后退、侧向移动或原地转向前需检查相应方向的覆盖和规划有效性。

## 本地离线验证

笔记本无需 ROS 或机器人即可运行已有合成场景：

```bash
./bin/go2-nav perception-demo --config config/perception_sector.json
./bin/go2-nav test
```

演示使用清楚标记为 SYNTHETIC 的场景尺寸，不写入真实标定文件。正式配置中的标定模板仍为未测量。

## 开销与记录

Point-LIO 继续接收全周原始点与 IMU。软件裁剪不改变雷达扫描模式、网络原始数据量、Point-LIO 参数、扫描频率或前方采样密度。新增成本控制主要作用于点云体素、滚动融合、ROS 输出与 RViz 绘制；逐点近似配准仍处理全部有效原始点，固定大小的地形栅格计算也不会按角度同比减少。

因此不能用 130/360 直接推算整机 CPU 降幅。周围近距离回波密度也会影响保留点数，实际收益待机载复测。

原有 0.5 秒源数据超时、0.3 秒工作快照期限、时间/坐标/跳变保护保持不变。每次启动继续保存命令、配置和终端输出；稠密状态、预览状态以及正式地图状态新增 `spatial_roi` 统计，包含前方点数、近距离点数、排除点数、体素尺寸和参考类型。点数是相应处理阶段的统计，不能把它当 CPU 百分比或障碍检测成功率。

## 机载排序修正记录

首轮机载对照中，累计点云三维行排序使预览工作周期过长，触发超时撤下；该轮结果不能作为显示稳定性或降负载通过的依据。已改用等价的整数体素键排序，并检查大跨度键的溢出回退，保留相同原始代表点。参数与超时阈值均未改变。修正后的真实点数、源年龄、CPU 和告警结果另存于部署日志。
