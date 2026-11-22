# Go2 导航工程

这是独立的导航 Git 仓库。以 `tools/go2_framework_sim_v3_20261122` 为导航基线，保留连续三维声源方向、地形分析、朝向规划、超时与未知区域停车逻辑；另含原手动面板的独立副本。

导航目前只输出规划路径和候选速度，尚未接通实机自主运动后端。仿真验收不能代替实机验收。头架尺寸、雷达和麦克风外参仍未测量，配置保持未标定。

## 文件与原件

| 内容 | 位置 | 是否纳入 Git |
|---|---|---|
| 完整 v3 仿真、导航源码、GUI、回放与回归测试 | `navigation/` | 是 |
| v3 原始场景报告、轨迹、地图快照、图像和验收材料 | `navigation/validation/` | 是，作为该版本的历史验证基线 |
| 手动面板、早期方向转换工具与测试 | `control/` | 是；方向转换工具是历史接口示例，导航使用自身 DirectionManager |
| 唯一启动入口、日志与部署工具 | `bin/`、`scripts/` | 是 |
| 固定依赖、ROS 包与启动配置 | `config/`、`package.xml`、`launch/` | 是 |
| 导入来源、逐文件 SHA256、历史验收摘要 | `provenance/` | 是 |
| 每次运行的参数、命令、版本及终端输出 | `logs/` | 否，保留在运行电脑上 |
| 虚拟环境、离线包、部署记录和 bundle | `.venv/`、`artifacts/` | 否 |
| 旧 backup、原工作空间、SDK、bag、点云地图、模型 | 原有目录 | 不复制进本仓库 |

原始 `backup/`、`tools/` 版本和机载旧目录保持原样。导入的源码未修改；本次新增工程入口、日志、依赖隔离、部署脚本和文档。首次来源见 [import_manifest.json](provenance/import_manifest.json)，完整 v3 补齐记录见 [v3_full_import.json](provenance/v3_full_import.json)。`navigation/manifest.json` 保存原版逐文件 SHA256。`docs/*original*` 是历史说明，工程统一启动命令以本 README 为准。

笔记本仓库：`/media/leon/新加卷/dog_project/go2_navigation`。
机载仓库：`/home/unitree/go2_navigation`。
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

**指定的 `tools/go2_framework_sim_v3_20261122` 版本已完整收入 `navigation/`**，共 661 个原版文件（不含 Python 缓存），内容逐一相同。不是只有 ROS 接口：包含交互 GUI、25 个场景、仿真模型、头架几何、规划器、独立审计、可视化工具和原版验证材料。

```bash
cd /media/leon/新加卷/dog_project/go2_navigation
./bin/go2-nav test
./bin/go2-nav sim --scenario gentle_slope
./bin/go2-nav gui
```

队友克隆仓库后，在仓库根目录运行 `./bin/go2-nav gui` 即可进入原版 v3 仿真界面；需要安装 Python 依赖和 Tk。完整安装、批量场景和历史验收说明见 [SIMULATION.md](docs/SIMULATION.md)。无需连接机器狗或安装 Unitree SDK；纯 Python 仿真也不要求 ROS。

每次运行新建 `logs/时间_模式_随机编号/`，包含 `metadata.json`（完整命令、Git 提交、工作区是否有修改、Python 及退出码）、参数副本 `config.json` 和 `terminal.log`。仿真结果在该目录的 `simulation/`。日志不提交 Git。

`navigation/validation/` 是随原版导入的历史记录。以后新产生的运行日志、轨迹和地图仍留在被忽略的 `logs/` 或 `navigation/runs/`，不会自动混入历史基线。

`doctor` 和 `test` 不导入 SDK、不发送运动指令。机载系统 Python 原先存在 NumPy/SciPy 版本冲突，新部署使用本仓库 `.venv`，不会升级原系统或用户包。`bin/go2-nav` 优先使用 `.venv/bin/python3`，禁用用户 site-packages；ROS 路径仅在预览子进程显式加载 Noetic。

## NoMachine 手动面板

在 NoMachine 的机载终端启动：

```bash
/home/unitree/go2_navigation/bin/go2-nav manual-preview
```

这是预览。以后明确需要实际短动作时使用 `manual-live`，点击按钮才调用原厂高层 SDK；不会自动站起或切换控制模式。实机入口明确使用 `/home/unitree/miniforge3/envs/rl/bin/python` 和 `/home/unitree/unitree_sdk2_python`，并记录 SDK 提交和修改状态。手动面板不自动绕障；导航候选速度不会自动送给它。本次部署不会执行 `manual-live`。

手动面板按钮事件另存在 `~/go2_logs/manual_panel_*/`。程序仅防止自身副本的重复实例，无法排除其它旧工具或其它电脑的控制程序。

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
