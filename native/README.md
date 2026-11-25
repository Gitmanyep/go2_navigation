# 原厂雷达电脑可视化

在机载仓库执行 `./bin/go2-native`，通过 NoMachine 查看 RViz。Ctrl+C 退出。短时检查可用 `./bin/go2-native --seconds 30`，无窗口用 `--no-rviz`。

每次启动在仓库 `logs/native_lidar_时间/` 保存来源路径、提交号、配置、SDK 副本路径、子进程日志和接收状态。代码、启动配置、DDS 消息与 SDK 源码都从本仓库加载。修改 `native/` 和 `config/native_lidar.*` 后再启动。

SDK 来源和逐文件校验在 `vendor/unitree_sdk2_python/snapshot.json`，许可证在该目录 `LICENSE`。编译后的 CycloneDDS 运行库沿用机载安装位置，路径可在 JSON 配置中修改。

启动发送一次原厂 `rt/utlidar/switch ON` 雷达开关，然后订阅已有数据。电脑 ROS 话题在 `/go2/native/`，桥接沿用原厂时间戳和坐标帧。TF 来自原厂里程计。点云帧若没有原厂 TF，在 RViz 中先将 Fixed Frame 改成点云消息实际的 frame_id。

Native cloud: waiting for data 表示尚未收到点云，原厂端点存在并不保证此刻在出数据。原厂 SLAM 建图保存、重定位和导航接口仍需单独验证。


用 `native/check.py --seconds 8`（先 source Noetic）可以在仓库中采样并保存原厂点云、定位和近地覆盖报告。时间统计中的 header-to-wall 差值包含机器人控制侧与机载电脑的时钟偏移，只有核实时间基准后才可作为处理延迟。桥接保留原始时间戳，和其他雷达融合前需先验证同步。位置变化统计要结合采样时是否发生站起、趴下或搬动来解释。
