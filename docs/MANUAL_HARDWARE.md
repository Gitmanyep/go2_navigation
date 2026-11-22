# 2026-11-22 手动实机入口诊断与适配

用户第一次点击前进时报告“已经站稳，但完全没动”。原版面板日志记录 6 次 Sport Move 本地返回 0，约 0.3 秒后发送零速度返回 0、Sport StopMove 返回 -1，因此故障保持锁定。之后用户点击停止，StopMove 返回 0，面板仍保持此前故障，避免未经核实恢复运动。

只读检查确认 mcf 模式，Sport 服务与本地 SDK 的 API 版本均为 1.0.0.1；原厂避障 SwitchGet 返回 true，避障服务 API 为 1.0.0.2。说明 DDS 和服务查询可通信，但这些查询不能证明速度指令已被执行。

SDK 源码中 Sport Move 使用无应答发送，0 仅表示本地发送成功；StopMove 使用有应答请求。因此不能把它们的返回值混成同一种执行确认。-1 的具体服务端原因尚未确定，状态反馈中的 error_code=100 也不擅自解释为某个故障或就绪值。

原面板没有适配已开启的原厂避障通道，这是需要修正的兼容缺口；仍需由操作者再次验证，不能声称它是本次不运动的唯一原因。

## 修正后的操作

1. 使用原厂方式站稳，确认场地及遥控接管条件。关闭旧的故障面板。
2. 运行新仓库的 `bin/go2-nav manual-live --iface eth0`。启动阶段只查询状态，不取得 API 指令源。
3. 看界面通道、原厂避障状态和接收年龄，点击“我已确认站稳：接管本次短动作”。
4. 5 秒内点击一次方向，保持原来的 0.1m/s、0.3s 上限。程序不通过提高速度、延长持续时间或关闭避障来解决拒绝。
5. 短动作结束后，程序请求零速度和 Sport StopMove，并尝试 `UseRemoteCommandFromApi(false)` 交还；继续动作需重新接管。

避障通道接管先发送零速度，再显式切到 API 来源，再发送零速度；这些动作只发生在操作者点击接管后。未接管、反馈接收过期、避障开关变化或 mcf 模式不匹配时拒绝非零请求。接管中反馈变旧会结束请求并尝试停止与交还。源时间/固件就绪契约仍未完成验证，接收新鲜不等于物理就绪。

接管改变的是原厂避障速度指令来源；它可能影响遥控输入的处理，必须现场验收。不能保证主机崩溃/SIGKILL 后还能执行软件交还，也不能把本地 5 秒有效期视为机器人端硬件看门狗。未知控制状态下不自动重新接管。

停止错误保持锁定，即使后一次 StopMove 应答成功也不自动解除。界面只显示请求和实际反馈，不宣称已执行或物理停止。若狗没有执行，保存这次 events.jsonl 和日志，继续比较真实反馈，不重复发非零指令探测。

## 留档与源码边界

程序与参数日志在新仓库 `logs/`，按钮和 SDK 事件在 `~/go2_logs/manual_panel_*/`。本次仅修改独立 Git 仓库内的手动面板与适配模块，不修改原厂 SDK、避障开关、关节参数或原 `backup/`。真实运动、停止和接管验收不能用假客户端测试替代。

官方源码参考：[SportClient](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/sport/sport_client.py)、[ObstaclesAvoidClient](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/obstacles_avoid/obstacles_avoid_client.py)。实际调用以机载已安装版本的读取结果为准。
