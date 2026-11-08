# 2026-11-09 部署与验证记录

本目录是试验机 Nano 上「三档遥控器授权任务服务」部署当天的输出归档，只记录设备侧结果，
不含本地开发环境的编译产物或依赖配置。流程与未验证边界见
[docs/pilot-navigation.md](../../docs/pilot-navigation.md)。

| 文件 | 内容 |
|---|---|
| `deployment_manifest.json` | 版本目录 10 个文件的 SHA256；仓库 `pilot-navigation/` 与之逐字节一致 |
| `baseline_manifest.json` | 部署前 5 份原有文件的 SHA256（定位单元、守护脚本、两份 VIO/相机配置、校时脚本）；部署后复算一致 |
| `nano_tests.log` | Nano 上 44 项逻辑与隔离 ROS 回归的原始输出，末行 `Ran 44 tests ... OK`；首行是启动参数校验结果 |
| `vio_gate_test.log` | `ctest -R px4_vio_regression` 输出，1/1 通过 |
| `install_service.log` | 单元安装时的 enabled、`Restart=no`、MainPID 与 enable 软链接记录 |
| `final_services.log` | 两个服务 enabled/active 状态、MainPID 与任务服务启动后的等待日志 |
| `live_wait_observer.log` | 实际控制域只读观测 5 s：25 条 WAIT 状态，0 心跳、0 轨迹、0 VehicleCommand，无开关与 VIO 健康发布者 |

测试入口是版本目录内 `test_pilot_authority.py`、`test_pilot_boot.py`、
`test_pilot_navigation_ros.py`，由 unittest 收集；隔离 ROS 用例只在 ROS_DOMAIN_ID=44、
`/pilot_test` 主题前缀下运行，未安装仿真健康桩。日志中的时间戳按设备本地时钟记录，
设备 UTC 当时未与 NTP 同步，不能据此推断墙钟时间。

结果边界：44 项回归和 1 项 VIO 门控都是软件用例；本轮无相机及有效 DDS 输入，
未做实机 VIO 初始化、十米航线精度、接管时延、重启或断电自启动验证。
