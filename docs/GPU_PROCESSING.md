# DECXIN 的 GPU 处理后端

## 处理边界

CUDA 是可选构建功能，当前提供 FAST-9 和双目立体校正两个独立开关。
默认 CPU 路径保持可用；不要求 OpenCV 编译 CUDA 图像模块。

本阶段保持相机 MJPEG 2720×960 @30、单 V4L2 缓冲，处理每目 960×720、
1000 配置特征、8 层金字塔、FAST 阈值 20/7。
校正先输出每目 1280×960，再由既有 Settings 缩放并同步内参和 bf。
外参、物理基线和图像时间戳生成位置不变。
完整原始状态内录、每帧导航和自动验证持续运行，日常图像编码仍关闭。

## 双目校正实现

CudaStereoRectifier 直接消费 CPU 原有的 CV_16SC2 坐标和 CV_16UC1 分数表，
使用 1/32 插值网格、相同整数舍入和相同零值边界。
图像越界以单目裁剪区域为准，不采样另一只镜头或填充区域。

校正表、显存和非阻塞流在每个采集线程内复用。
每帧只上传一次解码后的完整灰度图，再取回左右校正图。
原始输入可有行跨度，不要求连续内存。
本阶段仍需要回到 CPU 完成缩放、金字塔、方向、描述子和跟踪。

实例只属于一个采集或回放线程，初始化后表内容和几何必须保持不变。
检测到尺寸、裁剪或表身份改变，以及 CUDA 运行错误时，禁用该实例的 GPU 路径，
明确输出 CPU fallback，由调用者使用原有 cv::remap 重算整帧。

本实现追求与原输入字节一致，不通过改插值、裁视场、减少特征或修改标定换速度。
逐像素相等覆盖已验证输入，仍不能替代长期 VIO 和带真值的精度验证。

## 构建与运行

~~~bash
cmake -S . -B build -DORB_SLAM3_WITH_CUDA_FAST=ON \
  -DORB_SLAM3_CUDA_ARCH=87 -DORB_SLAM3_BUILD_TESTS=ON
cmake --build build --target stereo_uvc cuda_rectify_regression -j1

# 停止其他相机使用者后，临时选择 GPU 校正，FAST 仍由 CPU 执行
ORB_SLAM3_RECTIFY_BACKEND=cuda ORB_SLAM3_FAST_BACKEND=cpu \
VIO_PROCESSING_PROFILE=light ORB_SLAM3_PARALLEL_LEVELS=1 \
bash run_recovery_test.sh --headless --px4-config config/decxin_px4.yaml

# CPU 回退
ORB_SLAM3_RECTIFY_BACKEND=cpu bash run_recovery_test.sh --headless --check
~~~

构建开关沿用 ORB_SLAM3_WITH_CUDA_FAST，同时包含校正模块。
未构建 CUDA 的程序继续使用 CPU 路径。
effective_launcher.conf 记录两个后端选择；实际执行还应核对
run.log 中的 CUDA fixed-point remap active 或 CPU fallback。

## 回归入口

~~~bash
ctest --test-dir build -R '^cuda_rectify_regression$' --output-on-failure
build/tests/cuda_rectify_regression runtime_rectify.yaml motion_dataset 100000
~~~

无参数测试检查四组合成输入、全部 1024 种分数系数、图像和校正表的非连续行、
单目边界、重复使用、关闭 GPU 以及校正表变更回退。
带数据集参数时检查实际图像；最后一个参数控制最大采样数，
大于数据集帧数时逐帧检查全部输入。图像和参考输出只用于检查，不提交代码分支。

## 验证限制

回放没有外部真值；地图与建图线程调度会使不同运行的轨迹发生差异。
输入像素一致、有效位姿率和独立 PnP 残差是分别检查的证据。
纯双目回放不替代惯性初始化和导航融合。
未完成新后端下的长期 VIO、GUI、整机重启或实飞验收。
飞控观测发布与飞手授权门控保持原条件。

## 第一阶段实测

Jetson Orin Nano，25W，6 核 CPU 上限 1.344 GHz，GPU 上限 918 MHz。
以下服务对照使用原 schedutil 调度策略，40 秒资源采样，完整状态内录和自动验证均开启。

| 配置 | 服务帧率中位数 | 校正中位耗时 | 六核平均 CPU 占用 | 有效导航帧 |
|---|---:|---:|---:|---:|
| CPU 校正 + CPU FAST | 13.8 fps | 19.7 ms | 64.1% | 679/679 |\n| GPU 校正 + CPU FAST | 12.9 fps | 5.4 ms | 55.2% | 647/647 |\n| GPU 校正 + GPU FAST | 13.2 fps | 4.9 ms | 51.3% | 652/652 |\n
三组静态导航均保持单一参考 epoch，正常停止后写出包数与帧数一致。
GPU 校正节约 CPU，但本轮整套服务帧率没有提升；不能把单步骤耗时下降写成整体加速。

全部 557 组真实双目输入与四组合成图的校正输出逐像素相等，差异像素为 0。
合成图包括全部 1024 种分数系数、黑白/常量/随机输入、边界和非连续行。
独立完整数据检查的 CPU/GPU 校正中位耗时为 10.02/6.44 ms。

同一段运动数据分别回放 CPU 和 GPU 校正，共 1114 帧；
两轮均为 556/557 帧有效、1 帧丢失、末帧有效，未发生完整地图重置。
独立 PnP 帧间旋转残差 P95 为 0.167°/0.244°；
该参考并非外部真值，回放的地图优化和线程调度存在运行间差异。
计算与解码在离线回放中争用同一资源，总回放吞吐为 11.06/11.04 fps，
本阶段没有证明整体吞吐提高。
