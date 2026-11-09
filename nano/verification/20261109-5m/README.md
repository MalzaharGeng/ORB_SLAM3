# 2026-11-09 晚间：航线距离由 10 m 改为 5 m

设备 `~/.config/pilot-navigation.json` 只改两个键：`distance_m` 10.0 → 5.0、
`max_radius_m` 11.0 → 6.0。围栏半径跟着改，保持比航线距离大 1 m，否则
`navigation_radius_exceeded` 要到 11 m 才触发，等于放松保护。速度 0.25 m/s 不变，
`waypoint_timeout` 按 `distance/speed*4+20` 从 180 s 自动缩到 100 s。

原配置备份在设备 `~/.config/pilot-navigation.json.bak-10m`，
SHA256 `3386e8fd1582ba6a9247ce11229e8d985f11b593fb70aa4c2e5203f9a07cd6a6`。
改后配置 SHA256 `458a8cbb28b3084f576f56bfb0af9a7934a9b40d58cbf5ee4004ae41a9371d40`，
本目录 `pilot-navigation.json` 是按字节从设备取回的副本。

验证：

- `pilot_navigation_boot.py --config ~/.config/pilot-navigation.json --check` 退出 0，
  输出参数含 `"--distance", "5.0", "--max-radius", "6.0"`。
- `sudo systemctl restart pilot-navigation.service` 后 `is-active=active`、`Restart=no`，
  MainPID 6323；`pgrep` 只有一个 `px4_square_bt.py` 实例，命令行带
  `--distance 5.0 --max-radius 6.0`，日志停在 `Waiting: observe_altitude_detent`。
- 旧进程收到 SIGTERM 时 rclpy 打印 `ExternalShutdownException` 堆栈，随后 systemd 记录
  `Deactivated successfully` 与 `Started`，不是启动失败。
- 版本目录 `releases/20261109-5ae37233bf98/` 的 10 个文件未改，`current` 仍指向它。

仓库影响：`pilot-navigation/pilot_navigation_nano.json` 已同步为 5 m，与设备运行配置一致，
按 README 部署映射重新下发得到的也是 5 m。`verification/20261109/deployment_manifest.json`
记录的是 11-09 部署时的 10 m 原版哈希，对应提交 `e2180ae`；当前 HEAD 按该清单核对为
9/10，10 m 原版可从该提交或设备版本目录取回。那份清单与同目录 5 份日志是当时的验证证据，
本轮未改动。

边界：本次只改运行参数，没有改门控、VIO 初始化、飞控模式切换逻辑，也没有实机飞行。
5 m 到位精度、接管时延、断电自启动仍未验证；11-09 的 44 项回归和 1 项 VIO 门控是在
10 m 配置下跑的，改参数后未重跑。服务重启后回到 `WAIT_PILOT`，两个人工档位需重新观察。
设备 UTC 未与 NTP 同步，状态目录出现晚于系统时间的文件时间戳，日志时间不能当墙钟使用。
