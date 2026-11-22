# 外部依赖与边界

机载现有环境为 Ubuntu 20.04 / aarch64 / ROS Noetic。笔记本是 Ubuntu 22.04；该仓库的 ROS 适配是 ROS1，不会因笔记本有 ROS2 Humble 就改成 ROS2。纯 Python 仿真可以在笔记本运行。

Point-LIO、Livox 驱动和机身状态来源仍由机载原工程提供。本次不复制其 build/devel、地图或 SDK，也不重新构建或修改它们。原路径、哈希核对留在项目 `tools/code_comparison_20261122/`。

原 SDK 参考：

- C++ `/home/unitree/unitree_sdk2`：`794e0c2116be7a5a7e71af84660b6858c0c40cbe`，已有本地修改，不是干净基线；导航预览不调用。
- Python `/home/unitree/unitree_sdk2_python`：`86755a444046ef9b6d610c0b51d450aa7eadbec4`，有未跟踪示例和模型；只有显式 `manual-live` 使用其高层 SDK。
- DDS `/home/unitree/cyclonedds/install`：仅手动实机入口使用，不更改。

Python3.8 的离线部署版本完整列在 `config/requirements-py38.lock`。wheel 校验值写部署记录，不把二进制依赖提交进 Git。新 `.venv` 继承系统 apt 包供 rospy/Tk 使用，禁止用户 site-packages，私有 NumPy/SciPy/Matplotlib 优先于系统旧版。

尚未完成：头架和传感器外参测量、真实 Point-LIO bag 对新版本的回放验收、机载预览频率与负载验收、真实运动跟踪/制动/接管/坡地避障。原 v3 的 31 场景与 78 测试是历史仿真结果，本次运行结果另记日志，不能将其写成实机通过。
