# Go2 仿真与实机手动测试工程

这是独立的 Git 仓库，包含仿真、候选路径预览、真实数据审计与局部建图、实机手动控制和只读监测。`simulation/` 以 `tools/go2_framework_sim_v3_20261122` 为基线，用于验证连续三维声源方向、地形分析、朝向规划、超时与未知区域处理。

**`simulation/` 是仿真与规划验证代码，不是已经可用的实机自主导航程序。** 其中的 ROS `preview` 可以读取真实雷达／定位数据，但只输出规划路径和候选速度，尚未接通实机自主运动后端。仿真验收不能代替实机验收。头架尺寸、雷达和麦克风外参仍未测量，配置保持未标定。

实机短动作与只读反馈工具在 `control/`：`manual-live` 由操作者点击后才发送运动请求；`monitor` 只订阅状态。仿真 `gui`／`sim` 和 ROS `preview` 不会调用实机运动接口。实机目前能使用手动面板运动，不等于仿真规划已经能控制实物自主避障。

## 文件与原件

| 内容 | 位置 | 是否纳入 Git |
|---|---|---|
| 仿真、规划算法、GUI、回放、只输出候选结果的 ROS 预览与回归测试 | `simulation/` | 是 |
| v3 原始场景报告、轨迹、地图快照、图像和验收材料 | `simulation/validation/` | 是，作为该版本的历史验证基线 |
| 手动面板、只读监测、早期方向转换工具与测试 | `control/` | 是；方向转换工具是历史接口示例，导航使用自身 DirectionManager |
| P0 数据审计、P1 真实滚动地图接入与只读 ROS 输出 | `perception/` | 是；测量与实机验收仍待完成 |
| 唯一启动入口、日志与部署工具 | `bin/`、`scripts/` | 是 |
| 固定依赖、ROS 包与启动配置 | `config/`、`package.xml`、`launch/` | 是 |
| 导入来源、逐文件 SHA256、历史验收摘要 | `provenance/` | 是 |
| 每次运行的参数、命令、版本及终端输出 | `logs/` | 否，保留在运行电脑上 |
| 虚拟环境、离线包、部署记录和 bundle | `.venv/`、`artifacts/` | 否 |
| 旧 backup、原工作空间、SDK、bag、点云地图、模型 | 原有目录 | 不复制进本仓库 |

原始 `backup/`、`tools/` 版本和机载旧目录保持原样。`simulation/` 中导入的 v3 源码保持原版；手动面板的后续适配见 CHANGELOG。首次来源见 [import_manifest.json](provenance/import_manifest.json)，完整 v3 补齐记录见 [v3_full_import.json](provenance/v3_full_import.json)。`simulation/manifest.json` 保存原版逐文件 SHA256。目录由 `navigation/` 更名为 `simulation/`，目录内文件内容未变；历史导入清单保留当时路径，新旧路径映射和哈希复核见 [directory_rename_20261122.json](provenance/directory_rename_20261122.json)。`docs/*original*` 是历史说明，工程统一启动命令以本 README 为准。

笔记本仓库：`/media/leon/新加卷/dog_project/go2_navigation`。
机载仓库：`/home/unitree/go2_navigation`。
从 GitHub 新克隆并运行的机载仓库：`/home/unitree/go2_navigation_github`；前一个路径保留首次部署版本，不会随 GitHub 自动更新。以下手动实机测试使用带 `_github` 的新目录。
共享仓库：[Gitmanyep/go2_navigation](https://github.com/Gitmanyep/go2_navigation)。

## 使用哪个版本

Git 不会自动改变机器人调用的程序。使用以下绝对路径，执行的就是新仓库：

```bash
/home/unitree/go2_navigation/bin/go2-nav doctor
/home/unitree/go2_navigation/bin/go2-nav preview
```

入口显示 `SOURCE`、`COMMIT`、`MODE` 和 `LOG`。新入口不依赖旧 ROS 工作空间覆盖路径，也不修改 `.bashrc`、旧桌面快捷方式、`~/bin/go2-manual` 或开机服务。新工程没有设置开机启动。

`preview` 需要机载 ROS Noetic 和可用的 ROS master。它读取已有 Point-LIO 输出，不替你启动旧雷达或定位程序。外参未标定时会拒绝把雷达里程计当机身位姿；这是预期保护。暂用外参查看候选时可以显式追加 `--allow-provisional-extrinsics`，该选项不代表外参已经验收。

输出沿用 v3 话题 `/go2/framework_v3/{map,path,candidate_velocity,status}`，不会发布机器人 `/cmd_vel`。避免与旧 v3 ROS 预览同时运行，以免同名节点或话题混淆。

输入为 `/cloud_registered`、`/aft_mapped_to_init` 和 `/go2/navigation/direction`。方向 JSON 包含 `seq, stamp_sec, frame_id, kind, vector, confidence, valid, source_id, session_id`；`vector=[x,y,z]` 是方向，机身坐标 x 前、y 左、z 上，不是已知距离。发布者每次重启使用新 session_id，时间戳为实际采集 Unix 秒。详细约定和任务状态见 `docs/v3_original_README.txt`。

## 笔记本测试与仿真

**指定的 `tools/go2_framework_sim_v3_20261122` 版本已完整收入 `simulation/`**，共 661 个原版文件（不含 Python 缓存），内容逐一相同。不是只有 ROS 接口：包含交互 GUI、25 个场景、仿真模型、头架几何、规划器、独立审计、可视化工具和原版验证材料。

```bash
cd /media/leon/新加卷/dog_project/go2_navigation
./bin/go2-nav test
./bin/go2-nav sim --scenario gentle_slope
./bin/go2-nav gui
```

队友克隆仓库后，在仓库根目录运行 `./bin/go2-nav gui` 即可进入原版 v3 仿真界面；需要安装 Python 依赖和 Tk。完整安装、批量场景和历史验收说明见 [SIMULATION.md](docs/SIMULATION.md)。无需连接机器狗或安装 Unitree SDK；纯 Python 仿真也不要求 ROS。

每次运行新建 `logs/时间_模式_随机编号/`，包含 `metadata.json`（完整命令、Git 提交、工作区是否有修改、Python 及退出码）、参数副本 `config.json` 和 `terminal.log`。仿真结果在该目录的 `simulation/`。日志不提交 Git。

`simulation/validation/` 是随原版导入的历史记录。以后新产生的运行日志、轨迹和地图仍留在被忽略的 `logs/` 或 `simulation/runs/`，不会自动混入历史基线。

`doctor` 和 `test` 不导入 SDK、不发送运动指令。机载系统 Python 原先存在 NumPy/SciPy 版本冲突，新部署使用本仓库 `.venv`，不会升级原系统或用户包。`bin/go2-nav` 优先使用 `.venv/bin/python3`，禁用用户 site-packages；ROS 路径仅在预览子进程显式加载 Noetic。

## P0 数据审计与 P1 真实局部建图

新增 `perception/` 专门接入真实 Point-LIO 数据，复用 TerrainMap 算法，独立于 `simulation/` 的演示入口。它不导入 Unitree SDK，不发布速度，不启动原雷达或定位程序。当前机载未连接，先完成离线软件验证，**未完成真实外参标定、时间同步、尺寸测量或移动验收**。

机载更新后，先让原雷达与 Point-LIO 按既有方式运行，再检查 30 秒数据：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav data-audit --seconds 30
```

确认测量和坐标关系后，使用自己的标定文件启动地图：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav mapping --calibration /home/unitree/go2_navigation_github/local/calibration.json
rviz -d /home/unitree/go2_navigation_github/config/perception.rviz
```

默认未标定模板会阻止地图输出，保留未知状态，并显示缺项。不要为出图而把标定声明改成 true。输出包括占据／未知栅格、已观测高程、三维障碍和机身位姿；源时间保留，断流或工作线程超时后地图变未知。实际 ROS 输入、坐标与标定步骤见 [P0_P1.md](docs/P0_P1.md)。这两个真实数据入口需要机载 Noetic，笔记本 ROS 2 不能直接运行。

笔记本可先用 `./bin/go2-nav perception-demo` 做不依赖 ROS 的合成移动检查。日志在本次 `LOG/perception/`，不上传 Git；演示通过不代表物理标定或实机建图通过。

## 未标定雷达参考滚动预览

机载原雷达与 Point-LIO 已运行时，可使用独立显示入口：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav lidar-preview
rviz -d /home/unitree/go2_navigation_github/config/lidar_preview.rviz
```

它显示 6×6 米滚动窗口、最近 3 秒注册回波和每格最低／最高回波，**不使用机身外参、不做地面或可通行性分类、不调用 TerrainMap 或运动接口**。全部输出位于 `/go2/lidar_preview/`，明确标为未标定，断流清空显示；原厂遥控器仍由操作者使用。它只用于初步观察滚动及数据稳定性，不能作为正式 TerrainMap 或机身碰撞验收。遥控异常先做静态检查。详细启动、图例、日志和验收方法见 [LIDAR_PREVIEW.md](docs/LIDAR_PREVIEW.md)。

## NoMachine 手动面板

在 NoMachine 的机载终端启动：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav manual-preview
```

这是预览。明确需要实际短动作时使用：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav manual-live --iface eth0
```

启动只查询 mcf 模式、服务版本、原厂避障开关，并订阅机身反馈。若原厂避障开启，短动作使用 `obstacles_avoid` 通道；若关闭，使用 `sport` 通道。面板不修改原厂避障开关，不自动站起或切换腿控模式。

先由操作者确认站稳，再点击“接管本次短动作”，5 秒内点击一个方向。避障通道仅在明确接管时切换 API 指令源；到期、停止、失焦和关闭会请求零速度、Sport 停止，并尝试交还 API 指令源。每次短动作后需重新接管。接管失败或停止/交还失败会锁定方向按钮，不自动解除或重复非零指令。实物遥控接管、制动和原厂避障效果仍需现场验收，不能把 API 交还应答当成物理停止证明。

界面显示实际机身速度、接收年龄和原始状态字段。`Move` 的 0 返回值仅表示本地发送无即时错误；它不是服务端执行确认，也不能据此宣布狗已移动。SDK 返回的未知状态值不推断为健康或就绪。详细诊断见 [MANUAL_HARDWARE.md](docs/MANUAL_HARDWARE.md)。

实机入口明确使用 `/home/unitree/miniforge3/envs/rl/bin/python` 和 `/home/unitree/unitree_sdk2_python`，并记录 SDK 提交和修改状态。面板未接入本仓库的导航避障，导航候选速度不会自动送给它。

手动面板按钮事件另存在 `~/go2_logs/manual_panel_*/`。程序仅防止自身副本的重复实例，无法排除其它旧工具或其它电脑的控制程序。

## 只读速度监测

监测窗口与手动面板独立，只订阅 `rt/sportmodestate`，不接管或发送控制指令。机载未连接时可先在笔记本验证演示：

```bash
./bin/go2-nav monitor-demo
```

机载更新到包含监测工具的提交后，在 NoMachine 的另一个终端运行：

```bash
/home/unitree/go2_navigation_github/bin/go2-nav monitor --iface eth0
```

测试前点击“重置统计起点”，切回手动面板进行一次动作，完成后回监测窗口点击“保存当前统计”。手动面板失去焦点会请求停止，因此应在动作之前切回它，动作过程中不要切换窗口。也可让队友使用原厂遥控器控制，监测窗口一直只读。

显示平面速度、原始速度分量、转向角速度、速度峰值、相对起点直线位移、累计航向变化及历史曲线。位移与航向均来自机内估计，可能漂移或重置，不是独立地面真值；未标定坐标系时不把原始 vx 当作机身前向速度。缺少的字段显示不可用，接收超过 0.3s 未更新时当前速度显示过期，不显示为零。接收新鲜也不能证明源状态未缓存。

原始状态按约 20Hz 保存到本次 `logs/.../telemetry/feedback.csv`，统计在 `segments.jsonl`，峰值是采样峰值。重置只开始新统计段，不删除旧数据。首轮仍应以已知距离或录像检查估计是否可信。本窗口不自动计算制动距离，也不保证记录每次 DDS 更新；它不改变运动时长、速度、原厂避障或关节参数。

无桌面时可以限定时长只记录：`./bin/go2-nav monitor --iface eth0 --headless --seconds 30`。离线对应 `monitor-demo --headless --seconds 2`，演示数据始终标记为 synthetic。

## SSH 断开与停止

前台运行与终端会话绑定，不应假定断开 SSH 后继续。只需预览在 SSH 断开后继续时，可显式运行：

```bash
nohup /home/unitree/go2_navigation/bin/go2-nav preview > /home/unitree/go2_navigation/logs/preview_console.log 2>&1 < /dev/null &
```

正常停止用运行终端的 Ctrl+C；后台运行则读取对应 `metadata.json` 的 `launcher_pid`，确认命令路径后 `kill -TERM PID`。入口转发信号并记录退出状态。本次没有设置常驻服务或后台自主运动。

## 部署与 Git 留档

笔记本 `origin` 连接上述 GitHub 仓库，提交后使用 `git push` 共享代码和仿真。机载 `origin` 目前仍指向首次部署用的离线 bundle；向 GitHub 推送不会自动更新机载文件。两边修改都需要在各自仓库提交，再通过 Git 同步和合并。

首轮部署工具只接受干净的 `main` 分支，目标目录必须不存在；遇到已有目录会停止，避免覆盖。离线 wheel 安装只写新仓库的 `.venv`。

```bash
python3 scripts/deploy.py --host unitree@当前IP \
  --socket /home/leon/.ssh/go2_deploy_20261122.sock \
  --wheels /tmp/go2_nav_wheels_py38_aarch64_20261122
```

部署生成 `artifacts/提交号/deployment.json`，核对双方 Git 提交及所有跟踪文件 SHA256，并保留源 bundle 和依赖 wheel SHA256。机载发布材料保留在 `/home/unitree/go2_deployments/提交号/`。

以后修改在笔记本仓库做 `git diff`、`./bin/go2-nav test`、`git add`、`git commit`。更新机载前先记录当前提交、检查 `git status`；使用 bundle 的 `git fetch` 和 `git merge --ff-only`，禁止直接覆盖目录。若更新改变依赖，先使用独立新环境验证。回到历史版本使用 `git switch --detach 提交号`，先停止该仓库的运行程序，且只在工作区干净时切换；不使用 `reset --hard`。

依赖参考及未完成项目见 [DEPENDENCIES.md](docs/DEPENDENCIES.md)，本次范围见 [CHANGELOG.md](CHANGELOG.md)。
