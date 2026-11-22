Go2 山地寻声导航框架 v3（2026-11-22）

目标与任务约定
面向“带头部框架、根据连续声源方向自主接近”。用户已确认：队友目前只提供方向，完成由人工确认；外接雷达装在后背，头架与安装外参尚未测量。
流程：连续方向与会话管理 → 短时点云融合/支撑地面识别 → 地形与朝向规划 → 完整外形预测检查 → 简化速度响应。
任务层：WAITING → STOP_LISTEN → APPROACH；方向丢失为 LOST，超时转 SEARCH（停车等重新获取，不盲走）；独立人工确认后进入 VERIFY_STOP，收到持续停稳的里程计才进入 COMPLETE。
没有真实 Go2 运动后端、没有 SDK/DDS 运动输出；这里只验证导航与任务逻辑，不是足端接触/打滑/负载动力学模型。

启动
本目录执行：
  python3 simulation.py --gui
或：
  ./run_gui.sh
界面左侧显示场景真值，右侧显示实际感知；可选择占据、坡度或净空视图。深灰表示未知，橙点为后背雷达，紫点为麦克风，黄色外形为未测头架示例。
“人工确认找到 → 停稳后完成”按钮是独立人工输入；距离真值不会触发这个按钮或完成状态。
当前工具环境无法连接桌面显示，窗口已写好但未完成实际显示验收；离线图可直接查看。
无桌面运行：
  python3 -m unittest discover -p 'test*.py'
  python3 simulation.py --scenario gentle_slope
  python3 simulation.py --suite
  python3 simulation.py --scenarios goal_jitter,random_obstacles --seeds 21,22,23
每次新建 runs/年月日_时分秒，拒绝覆盖同名目录。保存逐步轨迹、地图、状态转换、参数、运行耗时与报告。默认最长220s仿真时间，U形绕行比单障碍需要更长时间。
依赖沿用 Python3、NumPy、SciPy、Matplotlib；界面另需 Tk。本次未安装系统包。

针对评审已改进的内容
1. 地面不再要求“同一帧每格3点”。在0.7s窗口融合已配准的世界点云，每格至少3个样本才能建立观测；旧样本过期，不用新时间戳重盖。注册点云不能再次套用雷达安装变换。
2. 地面需与机身支撑高度一致：机身参考点名义离地0.30m、允许误差0.12m，并使用完整姿态估计支撑面；局部拟合及连通性限制防止平桌面/屋顶被直接认成地面。整片抬高2m且机身不动的样本被拒绝。
3. 缺失地面仍为未知。0.5m近场盲区会触发近场地面未观测/轮廓未知停车；多帧融合不能凭空恢复从未见过的区域。真实解除该限制需要独立近场观测或经过验证的初始环境证据，不能把脚下圆盘直接填空闲。
4. 同格中保留地面与非地面高度区间，较高横梁不再自动压成地面障碍；低横梁/树枝要和头架高度、朝向一起检查。仍不是完整多层地图，桥下/树冠等复杂结构不能保证正确解释。
5. 加入机身和头架分开的可配置包络、16个朝向碰撞掩码、8个航向的A*状态及转向中间角检查。跟踪时再按完整四元数和预测位姿检查3D点与包络；独立真值审计另检查墙体、横梁和转弯扫掠。
6. 方向滤波采用 alpha=1-exp(-dt/tau)，tau=0.5s；dt取源时间差。相同45°阶跃0.3s后，10Hz和50Hz均约20.3°，不随消息频率改变。
7. 方向增加 source_id/session_id。新进程须使用新会话ID，只有旧会话源数据到期才允许序号从0重新开始；退休会话不能复活。声源ID改变时清除旧目标并停听。旧消息格式仍可使用，但发布进程重启时须补新session_id。
8. 仿真使用固定三维声源和麦克风安装位置产生方向，竖直分量不再固定写0.3。声源真值只进入传感器模拟和评分，导航核心与任务状态机不读取它。
9. 只凭方向不能知道真实距离，因此不会宣布找到。独立人工确认还必须有匹配任务ID、新鲜时间和递增序号，并等待新鲜里程计持续停稳0.8s。声源方向消息中的 confirmed/target_confirmed 字段不能代替人工通道。
10. 被遮挡时保持一个中间目标，减少左右绕行反复切换；有访问记录和失败端点惩罚。直接目标不可达时允许先远离声音、退出U形开口，再尝试绕行。探索有无进展超时和总时限，不保证任意山地拓扑都能找到出路。

本轮v3修正
1. 高处障碍证据与0.7s地面融合窗口分离：保存绝对高度区间、实际3D障碍点和最后实际回波时间，默认保持3s；短暂无回波不能清除。过期后该区域转为未知，继续检查地面也不能把它变成空闲。后续真正看到障碍会刷新证据。稀疏初始帧中的障碍若到多帧融合后才被识别，也会按原始帧/点索引回填3D证据，不能用识别时刻续期。
   clear_obstacle_volume仅接受显式certified、覆盖完整旧高度区间且比旧证据更新的空闲体积证明；当前ROS点云入口不调用它，因为单纯没收到回波不是证明。清空水位会屏蔽融合窗口里的旧障碍点，更新的实际障碍回波仍优先。证据随滚动地图迁移，不在全局无限存储。
2. 跟踪按有序(position,heading)状态推进，保留重复位置上的独立转向步骤，逐步执行45°转向；不能用最近位置索引跳过转向；0.4m前瞻只用于同一朝向的连续直线段，到原地转向节点必须停下。初始栅格朝向仅是量化锚点，不能每次重规划都强制停下重新对齐。原地转向期间保持当前计划，扫掠被挡则停车保留该步骤，不另选反方向。预测检查保留实际3D障碍点，并按实际连续姿态检查；未知区域按完整外形检查，不用相邻离散朝向掩码的并集代替实际转角。探索停滞计时以每前进一个地图栅格更新，显式转向的耗时仍受25s无进展与180s探索阶段预算约束；即使正在跟踪已提交的中间目标也检查阶段超时，耗尽后锁定停车直到重置任务，不能下一次搜索又恢复运动。
3. 坡度可行性向量化预计算；不可达目标的完整搜索结果供候选终点复用；多候选只做一次搜索，同时保留原先的终点优先级，不能因最先找到近处终点就改变绕行方向。几何核缓存、同地图/证据有效期内的分析缓存、空掩码快速跳过，且无路可走时也遵守0.4s重规划周期。
   planner_budget_s=0.055是软件搜索预算，超预算返回零候选并标记planning_budget_exhausted，不是硬实时保证。
   runtime.py中导航工作线程独占地图，ROS回调只存有界队列（位姿300条、其它各最新1条）；发布线程只读取结果。候选租期从计算开始计，最多0.15s且不超过原始传感器/方向期限。计算期间的新人工命令/错误使旧批次结果不能覆盖停止状态；普通高频观测排入下一批，不使结果发布饿死，人工命令先使候选归零。发布使用主机单调时钟，不被/clock暂停或规划锁拖住。新到消息不能延长旧结果租期。
   图像/地图预览发布降至2Hz，候选发布名义10Hz；只有独立的本地线程探针，尚未实际运行Noetic发布器或验证机载10Hz。
4. 每条里程计直接维护静止连续区间，移动、失效、源时间/接收时间间隔超过task_observation_gap_s=0.25s均重置。Navigator即使点云失效也更新任务状态并重置停稳计时；0.8s必须同时由连续源时间与观察时间覆盖，不能只靠定时器经过0.8s。重新人工确认也重新计时。
5. 独立审计先归一化角度差再插值：179°→-179°只扫过2°，反方向同样处理，位置仍线性插值。测试检查中间姿态及避免虚假的头架碰撞。

机身与安装参数（config.json）
官方 Go2 参数：站立外形0.70×0.31×0.40m，含电池约15kg。这是标准整机参数，不包含用户的头架、外接雷达、载荷与行走腿部摆动扩张。
官方页面：https://www.unitree.com/go2/
官方 URDF：https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf
URDF机身base的简化碰撞box为0.3762×0.0935×0.114m，IMU相对base的示例平移为[-0.02557,0,0.04232]m；它不能代替整机外形。模型保留0.45m支撑/地面障碍圆形余量，并另外检查机身与头架包络。
原厂URDF的radar在前部，不能套给用户后背的外接Livox。
本版在机身坐标中把外接雷达原点暂设为[-0.15,0,+0.15]m，即后背偏后、朝向暂与机身一致；绝不把它写成已测外参。麦克风位置暂为[+0.35,0,+0.50]m。头架示例box为x=[0.20,0.65]、y=[-0.28,0.28]、z=[0.10,0.65]m，相对机身参考点。
lidar_extrinsics_calibrated=false、geometry_calibrated=false 标记尚未测量。尺寸、安装高度、机身参考高度都需测量替换。
自体回波剔除默认关闭。原始雷达点转换必须有标定外参；只有明确已标定外形才允许剔除已知实体内部点，不按周围一圈半径删除真实障碍。
simulate_frame_occlusion 可启用射线与框架包络的保守遮挡模拟，默认关闭：框架包络不是实体杆件模型，不能把整个框架空心部分都当成实心遮挡；有实际杆件尺寸后应替换。
纵坡15°、横坡10°、台阶0.12m、响应时间0.15s和制动减速度0.25m/s²都是仿真假设；官方极限参数不代表带头架后的可用能力，本版没有据官方极限放宽它们。

输入协议
保持ROS std_msgs/String JSON：
  {"seq":101,"stamp_sec":采集时刻,"frame_id":"body","kind":"direction", "vector":[x,y,z],"confidence":0.9,"valid":true,"source_id":"bird-a","session_id":"发布进程启动时生成的UUID"}
+x向前、+y向左、+z向上。vector是方向，不是位置或距离。
mic_array坐标仍要求真实mic→body旋转标定。消息按采集时间匹配位姿，拒绝重放、过期、未来超差、NaN、低置信度和近垂直的不可用水平投影。
人工确认独立接口：
  {"task_id":"从status读取当前任务ID","seq":1,"stamp_sec":当前确认时刻,"action":"confirm_found","confirmed":true}
每次任务创建随机task_id，旧任务确认不能用于新任务。只有人工确认且观察到停稳才会完成；重置GUI即创建新任务。

Noetic预览适配
包名改为 go2_framework_sim_v3，可与v1/v2源码并存，避免catkin包名重复。
在独立catkin工作空间src下链接本目录，编译并source后：
  roslaunch go2_framework_sim_v3 simulation.launch scenario:=gentle_slope
  roslaunch go2_framework_sim_v3 pointlio_preview.launch
订阅已有 /cloud_registered、/aft_mapped_to_init、/go2/navigation/direction。
v3输出位于 /go2/framework_v3/{map,path,candidate_velocity,status,simulated_odom}，人工输入 /go2/framework_v3/operator。
候选速度是TwistStamped，executable=false；不发布真实 /cmd_vel。
Point-LIO的“body”往往指雷达IMU参考点，不能仅按child_frame_id字符串认作Go2机身中心。本版默认 --odom-reference lidar，需要转换为机身位置；未测外参时默认不通过转换，并在status.adaptation_error说明原因。
只有明确做假设预览时可手动执行：
  python3 ros_node.py --mode preview --odom-reference lidar --allow-provisional-extrinsics
它使用暂定后背偏移，仍然标记未标定和不可执行；不能据此认定实机导航通过。
如果输入已真正换算为机身参考点，显式用 --odom-reference body，不能再次减去雷达偏移。
当前本机仅有ROS2 Humble；Noetic、catkin、实际话题和真实bag未完成运行验收。Python语法与XML检查不能替代这些验收。

真实数据入口与限制
v1/v2与历史山地点云仍在原目录，本版独立保存，不覆盖旧版本。
  python3 evidence.py --project /项目实际路径/dog_project --output /新的证据目录
历史二维地图回放仍仅检验二维接口，不代表3D地形识别成功。
  python3 inspect_pcd.py /路径/map.pcd --stride 8 --output /新的目录
PCD来源与身体参考高度未知时，v3可能全部判未知；不能为了让它出路径自动把任意高程当支撑地面。
  python3 replay_bag.py /路径/stationary.bag --output /新的回放目录
项目内存在历史TEB/DRL bag，尚未确认它们包含当前Go2的Point-LIO感知数据；此前实测记录引用的stationary.bag未在本地找到。当前未声称已回放真实新感知链路。replay_bag保留源/接收时间，可读方向与人工确认，但轨迹固定，是开环回放。
bag回放同样默认 --odom-reference lidar，未标定时会明确报错停止。若位姿已经换算成机身点，显式加 --odom-reference body；明确做假设预览时加 --allow-provisional-extrinsics，报告保留未标定标记。

验收与证据
validation/regression_tests.txt：原有52项与本轮26项回归测试，合计78项。
validation/before_regressions.txt：修复前，树枝丢回波和停稳连续性的4个复现全部失败。
validation/final_acceptance.json：最终汇总all_passed=true。78项测试、25主场景+6额外种子全部通过，31例独立几何审计无碰撞。21例方向接近、9例安全拒绝、1例人工确认停稳完成；方向用例错误自行完成数为0。
同机独占120步树枝场景：导航p50 225.9→6.9ms、p95 227.5→16.0ms、最大239.0→16.3ms；v3感知+导航最大25.9ms。并发功能批次最大导航计算61.9ms。500ms工作线程CPU阻塞探针中，约0.205s归零、最大发布间隔约100.0ms。以上均为本地软件测量，机载/Noetic 10Hz验收仍为false。
本轮功能验收用validate_cases.py逐进程隔离运行，并核对每例源码哈希；功能批次并发4例，其耗时不能当作独占运行基准。
  python3 validate_cases.py --output /新的主场景目录 --workers 4
  python3 validate_cases.py --output /新的种子目录 --scenarios goal_jitter,random_obstacles,localization_drift --seeds 22,23 --workers 4
独占对照与看门狗复现：
  python3 benchmark.py --source /实际v2目录 --output /tmp/go2_benchmark_v2.json
  python3 benchmark.py --source /实际v3目录 --output /tmp/go2_benchmark_v3.json
  python3 watchdog_probe.py --output /tmp/go2_watchdog.json
validation/verified_suite/report.json：25个主场景（含新增间歇树枝回波）；validation/verified_seed_sweep/report.json：额外种子。
validation/benchmark_v2.json 与 benchmark_v3.json：同机120步树枝场景的修复前后测量。
validation/watchdog_probe.json：规划工作线程CPU忙500ms时，独立发布循环的间隔与归零时间；不含ROS运行环境。
validation/model_layout.png：官方机身外形与暂定后背雷达/头架布置；validation/perception_comparison.png：真值与感知地图对照。
报告区分：
- proximity_reached：仅用真值评分，机身到声源地面投影的水平距离≤0.65m；不是程序知道距离，也不是三维空间碰到鸟。
- task_completed/task_success：程序自身收到人工确认并停稳完成；task_success还需真值接近与无碰撞。
- safe_rejection：危险/未知/窄通道拒绝推进，单独统计；不当成找到目标。
- arrival_stopped：完成状态下的实际模拟速度也已接近零。
只有方向的测试应保持task_completed=false，即使评分已接近声源；人工确认用例有显式的固定时刻操作事件，不由距离触发。
真值接近只用于评估窗口截取（接近后再保留5s），不终止导航控制器、不令任务COMPLETE；控制器的停止/完成依据是自己的状态和输入。
每次调用记录导航计算耗时、感知+导航耗时的分位数及最大值，并记录一阶响应下的停止距离。它们是本地软件模型测量，不是机载稳定10Hz或实测狗的刹车距离。real_time_10hz_verified固定为false。
可运行 python3 audit_truth.py /运行目录 做独立墙体与框架扫掠检查；该审计不向规划器提供真值。
before_*、first_*、second_*为复现/开发日志，最终结果以final_acceptance.json为准。

仍需现场数据完成的工作
头部框架/雷达/麦克风安装外参测量，真实近场盲区与自体回波验证；动态点云与位姿包；真实速度响应、停距、定位漂移；草木与坡顶遮挡、多层结构、动态障碍；框架质量/重心/足端接触和打滑的动力学验证。
当前是传统规划加原厂控制分层的导航仿真，不需要为了方向接近先训练步态。真实原厂运动后端还未接入，所有示例运动都发生在仿真模型中。
