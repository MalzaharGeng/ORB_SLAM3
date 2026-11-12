# 录制、地图与现场命令

## 开机运行

px4-vio.service 依次启动 DDS Agent、连续状态内录和 headless SLAM。
内录输出 INREC_READY 后才启动 SLAM；缺失或无法打开内录时拒绝无内录启动。
pilot-navigation.service 独立发布导航健康状态，行为树仍受飞手授权和导航门控约束。

DECXIN 日常服务持续记录原始 IMU、飞控状态、导航健康、验证结果和 SLAM 日志，
原始状态按大小滚动分段。为减少 CPU 编码负担，整段和滚动图像默认均关闭。
诊断时可开启最近 10 秒图像环，稀疏概览关闭；图像写 PNG 在独立线程进行，
内存队列最多 6 帧，队列满时丢弃图像副本。图像编码仍消耗 CPU，异步不等于零开销。

## 连续状态内录

~/inrec/inrec_<session>_partN.csv 保存送达本进程的 DDS 消息：
原始 SensorCombined、飞控位姿、姿态、状态、人工输入、着陆检测、电池、故障、
时间同步、外部视觉观测和导航健康状态。每行携带完整 JSON 载荷、消息时间、
主机时间和单调时间。未发布或未送达的飞控消息不能由记录器补造。

写盘使用有界异步队列，每约一秒刷盘并 fsync，正常停止排空队列。
原始状态数据每约 128 MiB 分段，当前 CSV 路径保持不变；封存分段位于相邻的
`<文件名>.csv.parts/`，受标记管理的封存段总预算为 8 GiB，超过后删除最旧封存段。
每次淘汰会写入 `evicted.jsonl`，不会假称旧数据仍然存在；正常关闭的最后一段也计入预算，
正在写的 CSV 和已有未标记历史资料不被自动删除。预算不包含其他逐帧导航 CSV、地图或历史数据；磁盘剩余低于
2 GiB 时显式报告失败。可在某段 `.parts/` 中放置 `PIN` 文件保留故障段，固定数据
占满预算时记录器报错，不静默覆盖。压缩历史段需在封存后另做，避免高频回调中压缩。
.csv.summary.json 给出接收、入队、写出数量、各来源计数、原始 IMU 间隔异常、
队列和错误。closed_cleanly=true 且 writer_complete=true 才表示已接收数据完整落盘；
这不证明飞控到 DDS 链路没有丢包，也不表示被滚动淘汰的旧段仍在磁盘。
摘要的 `storage` 字段说明保留范围和容量；大于 30 ms 或非正 IMU 间隔会单独记录。

每秒一条 verification 检查 IMU、飞控状态、局部位置和时间同步的新鲜度，
并记录导航就绪状态。写盘失败或队列满会显式退出；守护进程保留原分段并新开分段。
无法恢复内录时停止当前 SLAM。SLAM 重启期间连续状态内录继续运行。
停止服务时先停 SLAM，再排空内录，最后退出 Agent。

## 每次 SLAM 的精确导航记录

~/ORB_SLAM3/recordings/test_runs/<run>/ 包含：

| 文件 | 内容 |
|---|---|
| run.log | SLAM 和桥接控制台输出 |
| navigation.csv | 每次 TrackStereo 的状态、位姿、速度、有效性、参考 epoch 和时间 |
| navigation.csv.summary.json | 异步记录完整性、接受和写出包数、错误 |
| px4_telemetry.csv | 桥接逐帧遥测 |
| settings.yaml、px4.yaml | 本次实际使用的配置 |
| effective_launcher.conf | 处理档位、图像开关、CPU/CUDA 和层级并行选择 |

启用经过标定的惯性入口时，navigation.csv 还逐条保存实际交给 TrackStereo 的 IMU
批次；连续原始 DDS IMU 保存在 ~/inrec/。两者均不依赖向飞控发布观测。
导航记录队列溢出或写盘错误会结束当前 SLAM，并留下不完整状态。

## 按需图像内录

`VIO_ROLLING_IMAGE_RECORDING=1` 保留配置中的有界滚动图像缓存；设为 0 时关闭。
旧 `VIO_IMAGE_RECORDING` 是兼容别名。整段图像录制由启动器的显式 `--record` 参数控制，
常驻服务不传该参数。滚动队列溢出会计数并丢弃副本，不能称为完整连续视觉数据集。
需要改变开机服务参数时使用 systemd 环境覆盖，随后重启服务。

~~~bash
systemctl status px4-vio pilot-navigation
tail -f ~/vio_daemon.log
# 临时停止常驻服务后，按需运行图像诊断
VIO_ROLLING_IMAGE_RECORDING=1 bash run_recovery_test.sh --headless --px4-config config/decxin_px4.yaml
# 仅诊断时显式录制整段视觉输入，占用空间随运行时长增长
bash run_recovery_test.sh --headless --record --px4-config config/decxin_px4.yaml
~~~

图像事件由 vio_image_retention.py 管理；连续状态 CSV 不参与图像自动清理。
不要删除未确认的事故数据。地图与数据集不提交到代码分支。

## 验证边界

文件完整性、输入新鲜度、视觉跟踪、惯性初始化和飞控融合是不同检查。
完整内录不能替代 VIBA2、连续性、外部真值或实飞验收。
当前 VIO 服务通过 VIBA2、时序及一致性检查后才发布视觉观测；行为树另需实际融合与物理拨档授权，不能仅据进程存在授权飞行。
