# v3 仿真使用说明

仓库的 `navigation/` 就是用户指定的 `tools/go2_framework_sim_v3_20261122` 完整副本；原目录保留不动。导入共 661 个文件，逐一校验相同，排除 Python 缓存。这里同时提供导航部署所用模块和纯软件仿真，避免再维护两份规划源码。

## 队友克隆后启动

仿真需要 Python、NumPy、SciPy、Matplotlib，GUI 还需要 Tk，不需要机器狗、SDK、ROS 或强化学习模型。以下安装示例适用于 Ubuntu 的 Python 3.10；Python3.8 的固定版本见 `config/requirements-py38.lock`。

先在仓库根目录创建独立环境：

```bash
python3 -m venv .venv
.venv/bin/python3 -m pip install -r navigation/requirements.txt
./bin/go2-nav gui
```

如果系统缺少 venv 或 Tk，由电脑维护者安装 `python3-venv` 和 `python3-tk`。已配置好依赖的电脑可以直接启动。

GUI 显示场景真值与实际感知；支持选择场景、查看地形和净空、人工确认找到目标。完整功能和限制见 `navigation/README_zh.txt`。未测量的头架和传感器外参仍保持未标定，不因为仿真导入而视为实机标定完成。

## 常用入口

在仓库根目录：

```bash
# 导航、假客户端及工程入口检查，不向机器狗发指令
./bin/go2-nav test

# 原版交互式仿真
./bin/go2-nav gui

# 单个场景；运行记录保存在 logs/ 下
./bin/go2-nav sim --scenario gentle_slope

# 全部 25 个场景
./bin/go2-nav sim --suite

# 指定场景和多个种子
./bin/go2-nav sim --scenarios goal_jitter,random_obstacles --seeds 21,22,23
```

单次运行输出目录会打印为 `LOG`。离线仿真完成后，结果位于该目录的 `simulation/`。查看 `report.json` 中的 `all_passed` 和各场景结果。方向本身不包含距离，任务完成仍需独立人工确认；仿真只到达评分接近范围，不代表已经自动确认找到。

也可以使用原版入口：

```bash
cd navigation
../.venv/bin/python3 simulation.py --gui
# 或 ./run_gui.sh（使用 PATH 中的 python3）
```

## 历史验证材料与复核

`navigation/validation/` 保留原版所有报告、逐步轨迹、地图快照、图像、种子对照、性能记录和源文件哈希。原版 78 个测试、25 个主要场景及 6 个种子案例的汇总是历史验收记录，不是队友电脑或机器狗的新测试结果。

`navigation/manifest.json` 列出原版 660 个文件的 SHA256，清单自身另由 `provenance/v3_full_import.json` 记录校验值。历史源码一致性也可使用原版汇总工具复核：

```bash
.venv/bin/python3 navigation/acceptance.py
```

该命令读取历史材料并重新生成 `navigation/validation/final_acceptance.json`，不会重新运行所有场景；新的功能验收使用上面的 `test` 和 `sim` 命令。原版 ROS 包与启动文件也完整保存在 `navigation/`，工程的机载预览继续使用根目录 `launch/pointlio_preview.launch` 和 `bin/go2-nav preview`。

## Git 与后续修改

仿真源码、默认参数及原版验证材料随 Git 上传。新的 `logs/`、`runs/`、`.venv/` 和 Python 缓存仍被忽略。只有 `navigation/validation/` 中的原版地图快照与性能记录被明确允许纳入 Git。

未来修改规划或仿真，请在本仓库 `navigation/` 中修改、测试和提交。外面的 `tools/go2_framework_sim_v3_20261122` 是保留的原件，不会自动同步修改。本次新增提交先保存在笔记本；原机载仓库保持此前部署的版本，后续使用 Git 同步提交。
