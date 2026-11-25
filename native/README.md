# 原厂雷达电脑可视化

在机载仓库执行 `./bin/go2-native`，通过 NoMachine 查看 RViz。Ctrl+C 退出。短时检查可用 `./bin/go2-native --seconds 30`，无窗口用 `--no-rviz`。

每次启动在仓库 `logs/native_lidar_时间/` 保存来源路径、提交号、配置、SDK 副本路径、子进程日志和接收状态。代码、启动配置、DDS 消息与 SDK 源码都从本仓库加载。修改 `native/` 和 `config/native_lidar.*` 后再启动。

SDK 来源和逐文件校验在 `vendor/unitree_sdk2_python/snapshot.json`，许可证在该目录 `LICENSE`。编译后的 CycloneDDS 运行库沿用机载安装位置，路径可在 JSON 配置中修改。

启动发送一次原厂 `rt/utlidar/switch ON` 雷达开关，然后订阅已有数据。电脑 ROS 话题在 `/go2/native/`，桥接沿用原厂时间戳和坐标帧。TF 来自原厂里程计。点云帧若没有原厂 TF，在 RViz 中先将 Fixed Frame 改成点云消息实际的 frame_id。

Native cloud: waiting for data 表示尚未收到点云，原厂端点存在并不保证此刻在出数据。原厂 SLAM 建图保存、重定位和导航接口仍需单独验证。


用 `native/check.py --seconds 8`（先 source Noetic）可以在仓库中采样并保存原厂点云、定位和近地覆盖报告。时间统计中的 header-to-wall 差值包含机器人控制侧与机载电脑的时钟偏移，只有核实时间基准后才可作为处理延迟。桥接保留原始时间戳，和其他雷达融合前需先验证同步。位置变化统计要结合采样时是否发生站起、趴下或搬动来解释。


原厂建图服务状态可用 `./bin/go2-native-slam status` 查询，反馈原文保存在 `logs/native_slam_status_时间.json`。命令字符串参考已有客户端协议实现 https://github.com/legion1581/unitree_ui/blob/main/docs/slam.md ，以实机反馈为准。此命令只查询前端、建图、定位、导航、巡检状态和地图 ID。


确认原厂地图可被替换并留档后，使用 `./bin/go2-native-slam mapping-start` 开始建图，`./bin/go2-native-slam mapping-stop` 停止并保存。新建地图可能覆盖机器人的单个地图槽。应先检查状态并导出现有地图，原始点云备份无法代替完整导航配置备份。实时建图点云发布到 `/go2/native/mapping_cloud`，保存后的地图发布到 `/go2/native/slam_cloud`。命令成功与否需核对原厂反馈的 success/failed 和 get_status。


## 原生雷达滚动局部地图（当前项目入口）

在机载仓库运行 `./bin/go2-native --rolling`。这一入口读取原生去畸变点云和原生里程计，打开越肩跟随视角 RViz，同时显示地形通行代价与三维障碍；可选打开高程或原始回波图层。无需建立全局地图，不发送运动命令、控制权切换或原厂建图命令。原厂遥控器控制另行操作。

地图处理目标频率为 10 Hz，显示 2 Hz；融合窗口 1.2 秒，逐点证据保留原始年龄，实时输入超时仍为 0.5 秒。范围和更新参数集中在 `config/native_rolling.json`：前向 130°、最远 4 米，近处 1.5 米保留全向回波；这不是“只能看 1.5 米”。高程历史最多 10 秒，障碍证据单独保留 3 秒。地图窗口随原生里程计位置滚动，新区域保持未知。

这是**真实数据的未标定预览**。输入点已经在 `odom`，不套用 Livox 外参，也不再次变换点云。`base_link` 仅沿用原厂消息定义，其实物位置、轴向、头架和机身碰撞范围仍待验收。初始近水平场景地面由连续回波估计，不能代替外参测量；初次启动应在平整地面保持静止。当前虚拟地面参考与原生机身高度分离，避免站起/趴下直接把平地判为障碍。斜坡、台阶和移动一致性需后续实测。

颜色：绿色为暂定模型下低代价，黄/橙为更高代价，红色为暂定模型下禁止通行；灰色和未绘制区域都不能当作可走。未知边缘会保守膨胀。**绿色尚不能作为自主行走授权**，状态明确输出 `traversability_suitable_for_control=false`。三维障碍层是有地面支撑的非地面回波，其余回波可在 Native deskewed 图层查看，缺少地面支撑时保持未知。

原始时间戳保持不变。机器人与机载时钟偏移不当作处理延迟；接入会预热检查原生时间是否推进，并按固定初始锚点检查后续积压、按本机单调时钟检查断流。不能证明启动前传输延迟或硬件同步。检测到积压、重复时间、异常帧或超时会隐藏地图，定位跳变超过 1 米后清空并要求重启。

每次 `logs/native_lidar_时间/` 记录启动提交、命令、参数、`rolling_inputs.jsonl` 和 `rolling_status.jsonl`，包括点云/位姿配对误差、处理耗时、丢帧与有效性。Ctrl+C 关闭本入口创建的进程。`--seconds 20 --no-rviz` 可做限时后台检查。

原生点云中的精确零向量按无效/填充回波剔除（即便真实原点上恰好有一个点，也保守丢弃该点），不把它融合成地面；剔除数记录在状态中。较低趴姿地面初始化需连续三次拟合一致，估计不出时保持未知。


## 原生去畸变点云的零点修复

`native/bridge.py` 在发布 `/go2/native/cloud_deskewed` 前，使用 `native/pointcloud.py` 剔除 XYZ 精确零向量和 NaN/Inf。有效点的完整记录（含强度等字段）、顺序、原始时间戳、坐标帧均保留；删除点后同步更新 width、height、row_step。兼容行尾填充、大小端和 XYZ float32/float64。原始 `/go2/native/cloud` 保持透传；原厂 DDS 发布器未改动。

按需订阅 `/go2/native/cloud_deskewed_unfiltered` 可取得清理前的同帧副本；无订阅者时不额外发布。`/go2/native/status` 的 `details.cloud_deskewed.cleanup` 与 `cloud_deskewed_cleanup_totals` 记录前后点数、字节数、剔除数量及清理耗时。地图入口的检查仍保留。

这是对原厂消息中无效记录的接入修复，不能增加真实回波，也不代表已经修复了原厂控制器内部的点云生产逻辑。不要直接删除“前10000点”；本实现检查坐标，后续原厂消除占位点后，有效数据仍能正常透传。
