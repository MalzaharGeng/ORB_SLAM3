# 录制、地图与现场命令

## 开机运行

px4-vio.service 依次启动 DDS Agent、连续状态内录和 headless SLAM。
内录输出 INREC_READY 后才启动 SLAM；缺失或无法打开内录时拒绝无内录启动。
pilot-navigation.service 独立发布导航健康状态，行为树仍受飞手授权和导航门控约束。

DECXIN 的日常服务默认关闭图像编码与图像写盘，连续保存原始 IMU、飞控状态、
导航状态、验证结果和 SLAM 日志。需要图像才能复跑视觉算法；关闭图像内录后，
状态 CSV 不能替代视觉数据集。已有标定和运动图像数据不受此配置影响。

## 连续状态内录

~/inrec/inrec_<session>_partN.csv 保存送达本进程的 DDS 消息：
原始 SensorCombined、飞控位姿、姿态、状态、人工输入、着陆检测、电池、故障、
时间同步、外部视觉观测和导航健康状态。每行携带完整 JSON 载荷、消息时间、
主机时间和单调时间。未发布或未送达的飞控消息不能由记录器补造。

写盘使用有界异步队列，每约一秒刷盘并 fsync，正常停止排空队列。
.csv.summary.json 给出接收、入队、写出数量、各来源计数、原始 IMU 间隔异常、
队列和错误。closed_cleanly=true 且 writer_complete=true 才表示已接收数据完整落盘；
这不证明飞控到 DDS 链路没有丢包。大于 30 ms 或非正 IMU 间隔会单独记录。

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

VIO_IMAGE_RECORDING=1 保留配置中的环形图像内录：
最近 20 秒实际输入双目及同批 IMU，另有稀疏背景。图像编码在独立线程进行，
队列溢出会计数，不能称为完整连续图像数据集。--record 可显式录制整段视觉输入。
需要改变开机服务参数时使用 systemd 环境覆盖，随后重启服务。

~~~bash
systemctl status px4-vio pilot-navigation
tail -f ~/vio_daemon.log
# 临时停止常驻服务后，按需运行图像诊断
VIO_IMAGE_RECORDING=1 bash run_recovery_test.sh --headless --px4-config config/decxin_px4.yaml
~~~

图像事件由 vio_image_retention.py 管理；连续状态 CSV 不参与图像自动清理。
不要删除未确认的事故数据。地图与数据集不提交到代码分支。

## 验证边界

文件完整性、输入新鲜度、视觉跟踪、惯性初始化和飞控融合是不同检查。
完整内录不能替代 VIBA2、连续性、外部真值或实飞验收。
默认纯双目服务的飞控观测发布仍被禁止；不得据此直接授权自主飞行。
