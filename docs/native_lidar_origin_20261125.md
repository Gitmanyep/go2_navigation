# 原生雷达型号与零点排查：2026-11-25（已授权公开）

全程只读采样，未发送运动或模式切换指令。用户确认当前趴姿，看不到铭牌。

1. DDS发现表明原始点云与去畸变点云来自机器人内部控制器的 unitree_lidar_server，机载Jetson是订阅者。控制器SSH端口拒绝连接，未尝试密码或扫描其他端口，没有取得发布器源码。
2. 独立C++ SDK探针6秒收到原始点云32条、去畸变32条、状态30条。各打印3帧：原始零点0；去畸变前10000点XYZ=0，齐次位置w=1，intensity=0。与此前Python DDS的35帧结果一致。这证明默认点记录在本项目之前已存在；具体是否错误resize/容器构造或固定占位设计，尚无源码证据。
3. C++ LidarState有18字段，Python原有SDK缺dirty_percentage。临时相符类型4秒收到20条。固件1.2.3、软件1.0.0.38、SDK1.0.9。新增native/state_message.py匹配C++定义，原有SDK和vendor快照未改。已部署；持续桥接72条状态、接收年龄示例0.018秒、错误为空。
4. SDK1.0.9属于官方L1 SDK版本线，软件证据偏向L1；没有直接产品型号、硬件ID或SN，不能据此确认硬件或PM/RM子型号。需实物可辨识照片、序列号或出货记录最终确认。
5. 趴姿20条状态：dirty_percentage73至74，error_state0，cloud_size5658至5766，cloud_frequency3.79至9.83，cloud_packet_loss_rate原始值3.31至15.47（单位未核实）。后续持续状态dirty_percentage示例72。这个检测指标与固定10000零点独立；可能与窗口脏污、遮挡、趴姿近地/机身影响有关，尚不能认定物理原因。
6. 官方L1 SDK说明dirty percentage为保护罩脏污导致剔除的点比例。应先检查整圈窗口、保护膜、支架、线缆，之后再做短时站立对照，不长时间站立等待。

参考：https://github.com/unitreerobotics/unilidar_sdk ，https://github.com/unitreerobotics/unilidar_sdk2/blob/main/VERSION.md 。
原始日志：native_model_discovery_20261125.json、native_cpp_zero_audit_20261125.log、native_lidar_state_schema_audit_20261125.json、go2_native_zero_probe_20261125.cpp，均在本地logs目录。修复后干净检出的tests目录154项通过，状态另外由C++与Python真实DDS采样核对。

用户已明确授权公开本轮发布进程、固件/SDK版本、污染指标和修复。原始网络发现和采样日志仍保留在被Git忽略的logs中；此诊断摘要纳入docs公开。
