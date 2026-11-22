# 2026-11-22 本地及机载部署记录

本次只在笔记本和机载电脑创建独立 Git 工程，没有发布远程云仓库。

- 笔记本：`/media/leon/新加卷/dog_project/go2_navigation`。
- 机载：`unitree@10.29.31.104:/home/unitree/go2_navigation`，主机 `ubuntu`，aarch64。
- 初次部署及运行验证提交：`468f3ab720da195db43ca45482af766ac2e115aa`。
- 初次部署 44 个跟踪文件逐一 SHA256 匹配；随后提交本记录和 ROS 安装清单修正，最终提交与再次核对结果保存于 `artifacts/提交号/final_verification.json`。
- 笔记本和机载均通过 78 项导航测试、43 项假客户端接口检查、7 项部署入口测试，共 128 项；测试没有导入实机 SDK，也没有发送运动指令。
- 机载 `.venv` 的 NumPy 1.24.4、SciPy 1.10.1、Matplotlib 3.7.5 已确认实际加载路径；Noetic 的 rospy、rosbag 和消息接口导入通过。
- 依赖安装时 pip 还提示系统既有 launchpadlib/onnx-graphsurgeon 的缺失依赖；本次不使用或修改这些旧包。新导航依赖导入及测试通过。

本次新增机载 `/home/unitree/go2_navigation/` 和 `/home/unitree/go2_deployments/` 发布材料；没有修改原感知/运动工程、SDK、原 `backup/`、旧启动入口、开机服务或标定参数。

导航和手动控制源码副本与 `provenance/import_manifest.json` 中的原件哈希一致。新工程增加的是工程管理、启动、日志、依赖隔离和部署代码，没有将 v3 预览接成实机自主运动。

运行日志：两端各自的 `go2_navigation/logs/`。机载测试日志还会复制到笔记本 `artifacts/最终提交号/onboard_logs/` 便于留档。部署 bundle 和 wheel 校验值写在 `artifacts/468f3ab720da195db43ca45482af766ac2e115aa/deployment.json`；最终更新记录额外注明复用的依赖发布目录。

本次未启动原雷达或 Point-LIO、未启用导航预览常驻服务，也未启动 `manual-live`。传感器实时输入、实机运行频率、实际运动和 GUI 显示尚未因此获得验收。
