# 无 GUI 运行与 Windows SSH 显示

> 功能来源：[Kuroneko-avi/ORB_SLAM3](https://github.com/Kuroneko-avi/ORB_SLAM3)。本分支的可执行文件在构建树：下方直接调用的 `./Examples/...` 应使用 `./build/Examples/...`。启动脚本通过 `BUILD_DIR` 选择构建目录（默认 build/），通过 `RECORDING_DIR` 选择回放输入。

## 无 GUI 的实机测试

在 Linux 本机或普通 SSH 终端执行，不需要 DISPLAY：

```bash
~/ORB_SLAM3/run_recovery_test.sh --headless
```

也可以使用配置：

```bash
~/ORB_SLAM3/run_recovery_test.sh --config ~/ORB_SLAM3/config/recovery_headless.conf
```

程序持续接收 D435i 的双目红外和 IMU。每条指令输入后回车：

| 指令 | 对应操作 | 什么时候输入 |
| --- | --- | --- |
| `first` 或 `1` | First Init Done，保存首次地图 | 第一次 `end VIBA 2` 后、跟踪正常 |
| `reset` 或 `2` | Start Second Map，全量重置并重新初始化 | 首次 `Atlas save succeeded` 后 |
| `second` 或 `3` | Second Init Done | 第二次 `end VIBA 2` 后、跟踪正常 |
| `load` 或 `4` | Read First Map | 出现 `SECOND_READY` 后 |
| `status` | 阶段、存图状态、最近位姿、读档评分 | 随时 |
| `pose` | 初始 IMU 世界系中的 IMU XYZ、距离、四元数 | 随时 |
| `help` | 指令说明 | 随时 |
| `quit` / `exit` / `q` | 停止相机采集，关闭 SLAM 工作线程 | 结束测试 |

`queued` 只代表请求排队。等待对应成功状态后，再输入下一条推进指令。
错序指令会拒绝；过早存图会报告失败，等待真正完成 VIBA 2 后可以重新输入 `first`。
`FILE LOAD SUCCESS` 表示文件已读入，`ALIGNMENT SUCCESS` 才表示几何对齐通过。
终端输出保存在每次测试目录的 `run.log` 中。默认不录制原始传感器包。

输入轮询不阻塞图像处理；相机没有新帧时也会每 50 ms 检查指令和退出信号。
`quit`、Ctrl+C 或 SIGTERM 在主循环中执行清理；正在执行的优化/保存需要先返回。
无 GUI 实机模式在标准输入 EOF（例如终端断开）时退出；GUI 模式输入关闭后仍可通过窗口操作。
`--debug` 可保留异常回溯，Ctrl+C 会传递给实机程序用于正常退出。

如果直接调用可执行文件：

```bash
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i \
  Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/RealSense_D435i.yaml \
  d435i_recovery --headless --console
```

`--gui` 恢复窗口，`--no-console` 关闭终端控制。无 GUI 实机模式不能同时关闭终端控制。
离线 `--replay` 保持自动推进阶段，同时可以输入 `status`、`pose`、`quit`；它不连接 D435i。
主动 `quit` 会输出 `REPLAY_STOPPED_BY_USER`，这不等于完整恢复测试通过。

## 将 GUI 显示到 SSH 连接的 Windows

数据采集和 SLAM 仍在连接 D435i 的 Linux 上执行，SSH 转发图形到 Windows。
Windows 可以使用带内置 X server 的 MobaXterm：

1. 打开 MobaXterm，确认 X server 已启动。
2. 建立 SSH Session，填写 Linux 地址和用户名，在 Advanced SSH settings 中启用 **X11-Forwarding**。
   也可以在 MobaXterm 本地终端运行 `ssh -Y lyx@Linux地址`。
3. 登录后在 Linux 终端检查 `echo "$DISPLAY"`。应由 SSH 设置，常见为 `localhost:10.0`。
4. 在这个 SSH 终端中运行：

```bash
~/ORB_SLAM3/run_recovery_test.sh --config ~/ORB_SLAM3/config/recovery_ssh_windows.conf
```

此配置固定 Pangolin 的 X11 后端、OpenCV/Qt 的 xcb 后端，使用 Mesa 软件渲染，
保留 SSH 提供的 DISPLAY 和 XAUTHORITY，不设置 Windows IP，也不要求开放 X11 TCP 端口或执行 `xhost +`。
命令行同样可以使用 `--ssh-x11`。
仅使用 Windows PowerShell 终端而没有 X server / 支持 X11 转发的 SSH 客户端，不能显示 Linux 窗口。

Linux SSH 服务需要允许 X11 转发并安装 xauth。本仓库不自动修改系统 SSH 服务。
若服务未开启，可由服务器管理员检查 `/etc/ssh/sshd_config` 中的 `X11Forwarding yes`，并按主机部署方式重载服务。
不要把远程 DISPLAY 手动改成 `:0`，那会指向 Linux 本机桌面。

Pangolin 在本机使用 X11/EGL；远端 X server 和 Linux Mesa 仍需支持相应渲染路径。
`--check` 检查配置与动态库，不代表远端图形已经可用。实际启动时若安装了 xdpyinfo，脚本会检查 X11 连接。
若图像窗口能开但 Pangolin 报 EGL 错误，可检查 Mesa 软件渲染与 Windows X server；必要时使用无 GUI 终端控制，
或使用支持 OpenGL 的远程桌面。没有真实 Windows SSH 会话时，只能验证本地 X11 后端，不能声称端到端转发已经通过。

官方使用说明：[MobaXterm documentation](https://mobaxterm.mobatek.net/documentation.html)、
[OpenSSH ssh 的 X11 转发选项](https://man.openbsd.org/ssh.1)。

## 配置优先级与输出

默认配置为 `config/recovery_local.conf`。`--config` 选择文件，显式命令行参数覆盖文件。
例如 `--config config/recovery_ssh_windows.conf --headless` 最终使用无 GUI 模式。
配置只接受以下键的简单值，不执行 shell 代码：

```text
DISPLAY_MODE=local       # local / headless / ssh-x11
TERMINAL_CONTROL=on      # on / off
SOFTWARE_RENDERING=1    # 1 / 0，只在 ssh-x11 模式设置
```

上面的注释用于解释，实际文件请像仓库样例一样将注释单独写一行。
每次运行保存源配置 `launcher.conf` 和实际生效的 `effective_launcher.conf`，以及包含实际启动命令的 `launch.txt`。
这些配置控制显示与输入，不修改相机标定或初始化阈值。

```bash
~/ORB_SLAM3/run_recovery_test.sh --headless --check
~/ORB_SLAM3/run_recovery_test.sh --config ~/ORB_SLAM3/config/recovery_ssh_windows.conf --check
```

2026-10-28 验证：入口编译通过；无 DISPLAY 环境下 3/3 回归测试通过，完整真实数据离线恢复通过；
脚本 + GDB 的标准输入指令和主动退出通过。本地 X11/EGL 软件渲染窗口启动和终端退出通过。
本次未连接 D435i，也没有 Windows SSH 会话，因此不声明新的实机初始化或 Windows 端到端显示已验证。
详细证据在 `recordings/test_runs/console_ssh_validation_20261028/README.md`。
